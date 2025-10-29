#include "plugin.hpp"
#include "dsp/dsp.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace {
static constexpr int NUM_DELAY_LINES = 4;

struct DelayLine {
        std::vector<float> buffer;
        int writeIndex = 0;

        void init(size_t size) {
                buffer.assign(size, 0.f);
                writeIndex = 0;
        }

        float read(float delay) const {
                if (buffer.empty())
                        return 0.f;

                float size = (float)buffer.size();
                float readIndex = (float)writeIndex - delay;
                while (readIndex < 0.f)
                        readIndex += size;
                while (readIndex >= size)
                        readIndex -= size;

                int index0 = (int)std::floor(readIndex);
                int index1 = (index0 + 1) % buffer.size();
                float frac = readIndex - (float)index0;
                float a = buffer[index0];
                float b = buffer[index1];
                return rack::math::crossfade(a, b, frac);
        }

        void write(float sample) {
                if (buffer.empty())
                        return;
                buffer[writeIndex] = sample;
                writeIndex++;
                if (writeIndex >= (int)buffer.size())
                        writeIndex = 0;
        }
};

// Delay-based pitch shifter for shimmer reverb (octave up)
struct PitchShifter {
        static constexpr int GRAIN_SIZE = 1024;  // Smaller grains for more responsive shimmer
        static constexpr int NUM_GRAINS = 4;     // More grains for smoother output

        std::vector<float> buffer;
        int writePos = 0;
        float grainPhases[NUM_GRAINS] = {0.f, 0.25f, 0.5f, 0.75f}; // Evenly spaced grains

        void init(int sampleRate) {
                buffer.assign(GRAIN_SIZE * 6, 0.f);
                writePos = 0;
                for (int i = 0; i < NUM_GRAINS; ++i) {
                        grainPhases[i] = (float)i / (float)NUM_GRAINS;
                }
        }

        void write(float sample) {
                buffer[writePos] = sample;
                writePos = (writePos + 1) % buffer.size();
        }

        float processOctaveUp(float sampleRate) {
                if (buffer.empty())
                        return 0.f;

                float output = 0.f;
                float grainLen = GRAIN_SIZE;

                // Process overlapping grains with window crossfading
                for (int g = 0; g < NUM_GRAINS; ++g) {
                        // Grain phase advances at half speed (octave up = 2x pitch = 0.5x playback speed)
                        grainPhases[g] += 0.5f;
                        if (grainPhases[g] >= grainLen)
                                grainPhases[g] -= grainLen;

                        // Calculate read position with delay
                        float readOffset = grainPhases[g];
                        int readPos = writePos - (int)readOffset - GRAIN_SIZE * 2;
                        while (readPos < 0)
                                readPos += buffer.size();
                        readPos = readPos % buffer.size();

                        // Linear interpolation for sample reading
                        int nextPos = (readPos + 1) % buffer.size();
                        float frac = readOffset - std::floor(readOffset);
                        float sample = rack::math::crossfade(buffer[readPos], buffer[nextPos], frac);

                        // Hann window for smooth grain envelope (reduces artifacts)
                        float windowPhase = grainPhases[g] / grainLen;
                        float window = 0.5f - 0.5f * std::cos(2.f * M_PI * windowPhase);
                        output += sample * window;
                }

                return output * (1.f / NUM_GRAINS); // Normalize for NUM_GRAINS overlapping
        }
};

} // namespace

struct Ahriman : Module {
        enum ParamIds {
                BLEND_PARAM,
                TONE_PARAM,
                REGEN_PARAM,
                SPEED_PARAM,
                INDEX_PARAM,
                SIZE_PARAM,
                DENSE_PARAM,
                FSU_PARAM,
                MODE_PARAM,
                RESPONSE_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                IN_L_INPUT,
                IN_R_INPUT,
                BLEND_CV_INPUT,
                TONE_CV_INPUT,
                REGEN_CV_INPUT,
                SPEED_CV_INPUT,
                INDEX_CV_INPUT,
                SIZE_CV_INPUT,
                DENSE_CV_INPUT,
                FSU_GATE_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                OUT_L_OUTPUT,
                OUT_R_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                BOOT_LEFT_LIGHT,
                BOOT_LEFT_CENTER_LIGHT,
                BOOT_RIGHT_CENTER_LIGHT,
                BOOT_RIGHT_LIGHT,
                FSU_LIGHT,
                NUM_LIGHTS
        };

        DelayLine delayLines[NUM_DELAY_LINES];
        std::array<float, NUM_DELAY_LINES> delayTimes{};
        // Prime number-based delay multipliers for sparse FDN (less metallic resonances)
        std::array<float, NUM_DELAY_LINES> baseMultipliers{0.37f, 0.53f, 0.73f, 0.97f};
        std::array<float, NUM_DELAY_LINES> modScales{1.0f, -0.8f, 0.6f, -0.5f};

        // Hadamard matrix for FDN (orthogonal mixing)
        // H4 = [[1,1,1,1], [1,-1,1,-1], [1,1,-1,-1], [1,-1,-1,1]]
        static constexpr float hadamard[NUM_DELAY_LINES][NUM_DELAY_LINES] = {
                {0.5f,  0.5f,  0.5f,  0.5f},
                {0.5f, -0.5f,  0.5f, -0.5f},
                {0.5f,  0.5f, -0.5f, -0.5f},
                {0.5f, -0.5f, -0.5f,  0.5f}
        };

        PitchShifter shimmerL;
        PitchShifter shimmerR;

        float sampleRate = 44100.f;
        int bufferSize = 0;

        float lfoPhase = 0.f;
        float randomValue = 0.f;
        float randomTarget = 0.f;
        float randomTimer = 0.f;

        float inputEnv = 0.f;
        float toneLowL = 0.f;
        float toneLowR = 0.f;
        float toneHighL = 0.f;
        float toneHighR = 0.f;
        float bootTimer = 1.2f;
        bool bootActive = true;

        Ahriman() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(BLEND_PARAM, 0.f, 1.f, 0.5f, "Blend", "%", 0.f, 100.f);
                configParam(TONE_PARAM, -1.f, 1.f, 0.f, "Tone");
                configParam(REGEN_PARAM, 0.f, 1.f, 0.45f, "Regen");
                configParam(SPEED_PARAM, 0.f, 1.f, 0.5f, "Speed");
                configParam(INDEX_PARAM, -1.f, 1.f, 0.f, "Index");
                configParam(SIZE_PARAM, 0.f, 1.f, 0.5f, "Size");
                configParam(DENSE_PARAM, 0.f, 1.f, 0.5f, "Dense");
                configButton(FSU_PARAM, "FSU");
                configSwitch(MODE_PARAM, 0.f, 2.f, 0.f, "Reverb style", {"LIM", "DST", "SHM"});
                configSwitch(RESPONSE_PARAM, 0.f, 2.f, 0.f, "Mod response", {"BND", "LRP", "JMP"});

                configInput(IN_L_INPUT, "Left audio");
                configInput(IN_R_INPUT, "Right audio");
                configInput(BLEND_CV_INPUT, "Blend CV");
                configInput(TONE_CV_INPUT, "Tone CV");
                configInput(REGEN_CV_INPUT, "Regen CV");
                configInput(SPEED_CV_INPUT, "Speed CV");
                configInput(INDEX_CV_INPUT, "Index CV");
                configInput(SIZE_CV_INPUT, "Size CV");
                configInput(DENSE_CV_INPUT, "Dense CV");
                configInput(FSU_GATE_INPUT, "FSU gate");

                configOutput(OUT_L_OUTPUT, "Left output");
                configOutput(OUT_R_OUTPUT, "Right output");

                onSampleRateChange();

        }

        void onSampleRateChange() override {
                sampleRate = APP->engine->getSampleRate();
                int required = (int)std::ceil(sampleRate * 3.5f) + 8;
                if (required != bufferSize) {
                        bufferSize = required;
                        for (auto &line : delayLines) {
                                line.init(bufferSize);
                        }
                }
                for (size_t i = 0; i < delayTimes.size(); ++i) {
                        delayTimes[i] = sampleRate * 0.1f * baseMultipliers[i];
                }
                shimmerL.init(sampleRate);
                shimmerR.init(sampleRate);
        }

        float getBipolarCv(Input &input) {
                float volts = input.getVoltage();
                return rack::math::clamp((volts - 2.5f) / 2.5f, -1.f, 1.f);
        }

        float getUnipolarCv(Input &input) {
                return rack::math::clamp(input.getVoltage() / 5.f, 0.f, 1.f);
        }

        void process(const ProcessArgs &args) override {
                sampleRate = args.sampleRate;

                float blend = rack::math::clamp(params[BLEND_PARAM].getValue() + getUnipolarCv(inputs[BLEND_CV_INPUT]), 0.f, 1.f);
                float tone = rack::math::clamp(params[TONE_PARAM].getValue() + getBipolarCv(inputs[TONE_CV_INPUT]), -1.f, 1.f);
                float regen = rack::math::clamp(params[REGEN_PARAM].getValue() + getUnipolarCv(inputs[REGEN_CV_INPUT]), 0.f, 1.f);
                float speed = rack::math::clamp(params[SPEED_PARAM].getValue() + getUnipolarCv(inputs[SPEED_CV_INPUT]), 0.f, 1.f);
                float index = rack::math::clamp(params[INDEX_PARAM].getValue() + getBipolarCv(inputs[INDEX_CV_INPUT]), -1.f, 1.f);
                float size = rack::math::clamp(params[SIZE_PARAM].getValue() + getUnipolarCv(inputs[SIZE_CV_INPUT]), 0.f, 1.f);
                float dense = rack::math::clamp(params[DENSE_PARAM].getValue() + getUnipolarCv(inputs[DENSE_CV_INPUT]), 0.f, 1.f);

                int mode = (int)std::round(params[MODE_PARAM].getValue());
                int response = (int)std::round(params[RESPONSE_PARAM].getValue());
                bool fsu = params[FSU_PARAM].getValue() > 0.5f || inputs[FSU_GATE_INPUT].getVoltage() > 2.f;

                lights[FSU_LIGHT].setSmoothBrightness(fsu ? 1.f : 0.f, args.sampleTime * 4.f);

                if (bootActive) {
                        bootTimer -= args.sampleTime;
                        float brightness = bootTimer > 0.f ? 1.f : 0.f;
                        lights[BOOT_LEFT_LIGHT].setBrightness(brightness);
                        lights[BOOT_LEFT_CENTER_LIGHT].setBrightness(brightness);
                        lights[BOOT_RIGHT_CENTER_LIGHT].setBrightness(brightness);
                        lights[BOOT_RIGHT_LIGHT].setBrightness(brightness);
                        if (bootTimer <= 0.f) {
                                bootActive = false;
                        }
                } else {
                        lights[BOOT_LEFT_LIGHT].setBrightness(0.f);
                        lights[BOOT_LEFT_CENTER_LIGHT].setBrightness(0.f);
                        lights[BOOT_RIGHT_CENTER_LIGHT].setBrightness(0.f);
                        lights[BOOT_RIGHT_LIGHT].setBrightness(0.f);
                }

                float inL = inputs[IN_L_INPUT].getNormalVoltage(0.f);
                float inR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : inL;

                float inSum = 0.5f * (inL + inR);
                float inDiff = 0.5f * (inL - inR);

                inputEnv += 0.0025f * (((std::fabs(inL) + std::fabs(inR)) * 0.5f) - inputEnv);

                float regenShape = (regen < 0.55f) ? (0.9f * regen) : (0.495f + (regen - 0.55f) * 1.35f);
                float feedback = rack::math::clamp(0.25f + 0.75f * regenShape, 0.f, 0.995f);

                if (regen > 0.75f) {
                        float duck = rack::math::clamp((regen - 0.75f) / 0.25f, 0.f, 1.f);
                        feedback *= (1.f - duck * rack::math::clamp(inputEnv * 0.25f, 0.f, 1.f));
                }

                float denseShape = rack::math::crossfade(0.4f, 0.9f, dense);
                float inputGain = rack::math::crossfade(0.15f, 0.35f, dense);

                float sizeShaped = size * size;
                float baseSeconds = 0.03f + sizeShaped * 1.8f;
                float baseSamples = baseSeconds * sampleRate;

                float modDepth = std::abs(index);
                float freq = 0.05f * std::pow(2.f, speed * 5.5f);
                freq = rack::math::clamp(freq, 0.02f, 12.f);

                float modSignal = 0.f;
                if (modDepth > 0.f) {
                        if (index >= 0.f) {
                                lfoPhase += freq * args.sampleTime;
                                if (lfoPhase >= 1.f)
                                        lfoPhase -= 1.f;
                                modSignal = std::sin(2.f * M_PI * lfoPhase);
                        } else {
                                randomTimer -= args.sampleTime;
                                if (randomTimer <= 0.f) {
                                        randomTarget = rack::random::uniform() * 2.f - 1.f;
                                        float speedNorm = rack::math::clamp(speed, 0.f, 1.f);
                                        float holdBase = 0.015f + (0.24f - 0.015f) * (1.f - speedNorm);
                                        randomTimer += holdBase;
                                }
                                randomValue += 0.005f * (randomTarget - randomValue);
                                modSignal = randomValue;
                        }
                } else {
                        randomValue *= 0.999f;
                        modSignal = 0.f;
                }

                float modSeconds = (0.0015f + sizeShaped * 0.014f) * modDepth;
                float smoothing = 1.f;
                if (response == 0) {
                        smoothing = 0.0025f;
                } else if (response == 1) {
                        smoothing = 0.015f;
                }

                if (fsu) {
                        feedback = 0.995f;
                        inputGain = 0.f;
                }

                // Read delay taps with modulation
                std::array<float, NUM_DELAY_LINES> taps{};
                for (int i = 0; i < NUM_DELAY_LINES; ++i) {
                        float target = baseSamples * baseMultipliers[i];
                        target *= rack::math::crossfade(0.6f, 1.5f, dense);
                        float mod = modSignal * modSeconds * sampleRate * modScales[i];
                        target = rack::math::clamp(target + mod, 8.f, (float)(bufferSize - 8));

                        // Response modes: BND (smooth), LRP (interpolated), JMP (instant)
                        if (response == 2) {
                                delayTimes[i] = target;  // JMP: instant jumps
                        } else {
                                delayTimes[i] += (target - delayTimes[i]) * smoothing;
                        }
                        taps[i] = delayLines[i].read(delayTimes[i]);
                }

                // Apply Hadamard matrix mixing (FDN feedback matrix)
                std::array<float, NUM_DELAY_LINES> mixed{};
                for (int i = 0; i < NUM_DELAY_LINES; ++i) {
                        mixed[i] = 0.f;
                        for (int j = 0; j < NUM_DELAY_LINES; ++j) {
                                mixed[i] += hadamard[i][j] * taps[j];
                        }
                }

                // Stereo output derived from FDN
                float wetL = mixed[1] * 0.6f + mixed[0] * 0.25f + mixed[3] * 0.15f;
                float wetR = mixed[2] * 0.6f + mixed[0] * 0.25f - mixed[3] * 0.15f;

                // Shimmer mode: octave-up pitch shift with feedback
                float shimmerOutL = 0.f;
                float shimmerOutR = 0.f;
                if (mode == 2) {
                        // Feed reverb output into pitch shifter
                        shimmerL.write(wetL);
                        shimmerR.write(wetR);

                        // Get octave-up shifted output
                        shimmerOutL = shimmerL.processOctaveUp(sampleRate);
                        shimmerOutR = shimmerR.processOctaveUp(sampleRate);

                        // Blend shimmer into wet output (demonic quality)
                        wetL = rack::math::crossfade(wetL, shimmerOutL, 0.35f);
                        wetR = rack::math::crossfade(wetR, shimmerOutR, 0.35f);
                }

                // FDN feedback with nonlinear processing per delay line
                for (int i = 0; i < NUM_DELAY_LINES; ++i) {
                        float content = mixed[i];

                        // Nonlinear processing based on mode (applied at each 4x4 mix node)
                        if (mode == 0) {
                                // LIM: Hard limiting
                                content = rack::math::clamp(content, -1.15f, 1.15f);
                        } else if (mode == 1) {
                                // DST: Soft saturation/distortion
                                content = std::tanh(content * 1.5f) * 0.9f;
                        } else {
                                // SHM: Lighter saturation for shimmer
                                content = std::tanh(content * 1.2f);
                        }

                        // Stereo input injection
                        float stereoSpread = (i % 2 == 0) ? 1.f : -1.f;
                        float injection = inputGain * (inSum * 0.7f + inDiff * stereoSpread * 0.3f);

                        // Shimmer mode: feed pitch-shifted signal back into tank
                        if (mode == 2) {
                                float shimmerFeed = (i % 2 == 0) ? shimmerOutL : shimmerOutR;
                                injection += 0.3f * shimmerFeed;
                        }

                        // Write to delay line with feedback and dense control
                        float writeSample = injection + feedback * content * denseShape;
                        delayLines[i].write(writeSample);
                }

                auto toneProcess = [&](float &sample, float &lowState, float &highState) {
                        // Bipolar control: left = lowpass, right = highpass, center = disabled
                        float amount = std::abs(tone);

                        // Dead zone at center for filter disabled
                        if (amount < 0.05f) {
                                // Filter disabled, pass through
                                return;
                        }

                        if (tone < 0.f) {
                                // Left side: lowpass filter
                                float freq = rack::math::crossfade(20000.f, 400.f, amount);
                                float alpha = std::exp(-2.f * M_PI * freq / sampleRate);
                                lowState = rack::math::clamp(lowState + (1.f - alpha) * (sample - lowState), -12.f, 12.f);
                                sample = rack::math::crossfade(sample, lowState, amount);
                        } else {
                                // Right side: highpass filter
                                float freq = rack::math::crossfade(20.f, 4000.f, amount);
                                float alpha = std::exp(-2.f * M_PI * freq / sampleRate);
                                highState = rack::math::clamp(highState + (1.f - alpha) * (sample - highState), -12.f, 12.f);
                                sample = rack::math::crossfade(sample, sample - highState, amount);
                        }
                };

                toneProcess(wetL, toneLowL, toneHighL);
                toneProcess(wetR, toneLowR, toneHighR);

                wetL = std::tanh(wetL * 0.8f) * 5.f;
                wetR = std::tanh(wetR * 0.8f) * 5.f;

                float outL = rack::math::crossfade(inL, wetL, blend);
                float outR = rack::math::crossfade(inR, wetR, blend);

                outputs[OUT_L_OUTPUT].setVoltage(outL);
                outputs[OUT_R_OUTPUT].setVoltage(outR);
        }
};

struct BackgroundImage : Widget {
	std::string imagePath = asset::plugin(pluginInstance, "res/TextureDemonMain.png");
	widget::SvgWidget* svgWidget;

	BackgroundImage() {
		// Create & load SVG child safely
		svgWidget = new widget::SvgWidget();
		addChild(svgWidget);
		try {
			auto svg = APP->window->loadSvg(asset::plugin(pluginInstance, "res/Ahriman.svg"));
			if (svg) {
				svgWidget->setSvg(svg);
			} else {
				WARN("SVG returned null: res/Ahriman.svg");
			}
		} catch (const std::exception& e) {
			WARN("Exception loading SVG res/Ahriman.svg: %s", e.what());
			// Leave svgWidget with no SVG; still safe to run.
		}
        }

	void draw(const DrawArgs& args) override {
		// Draw background image first
                std::shared_ptr<Image> image = APP->window->loadImage(imagePath);
                if (image && box.size.x > 0.f && box.size.y > 0.f) {
			int w = box.size.x;
			int h = box.size.y;

			NVGpaint paint = nvgImagePattern(args.vg, 0, 0, w, h, 0.0f, image->handle, 1.0f);
			nvgBeginPath(args.vg);
			nvgRect(args.vg, 0, 0, w, h);
			nvgFillPaint(args.vg, paint);
			nvgFill(args.vg);
		}
		// SVG will be drawn automatically by the child SvgWidget
		Widget::draw(args);
	}
};

struct AhrimanWidget : ModuleWidget {
        AhrimanWidget(Ahriman *module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Ahriman.svg")));

                auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size;
		addChild(bg);

                addChild(createWidget<ScrewSilver>(mm2px(Vec(1.5f, 1.5f))));
                addChild(createWidget<ScrewSilver>(mm2px(Vec(1.5f, 125.5f))));
                addChild(createWidget<ScrewSilver>(mm2px(Vec(49.3f, 1.5f))));
                addChild(createWidget<ScrewSilver>(mm2px(Vec(49.3f, 125.5f))));

                // Boot lights at header (y=13mm)
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(14.f, 13.f)), module, Ahriman::BOOT_LEFT_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(21.5f, 13.f)), module, Ahriman::BOOT_LEFT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(29.3f, 13.f)), module, Ahriman::BOOT_RIGHT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(36.8f, 13.f)), module, Ahriman::BOOT_RIGHT_LIGHT));

                // Blend (large knob centered, y=20mm)
                addParam(createParamCentered<Davies1900hLargeBlackKnob>(mm2px(Vec(25.4f, 20.f)), module, Ahriman::BLEND_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 20.f)), module, Ahriman::BLEND_CV_INPUT));

                // Row 1: TONE / REGEN (y=34mm, spacing 14mm)
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(16.5f, 34.f)), module, Ahriman::TONE_PARAM));
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(34.3f, 34.f)), module, Ahriman::REGEN_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 43.f)), module, Ahriman::TONE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 43.f)), module, Ahriman::REGEN_CV_INPUT));

                // Row 2: SPEED / INDEX (y=52mm, spacing 12mm)
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(16.5f, 52.f)), module, Ahriman::SPEED_PARAM));
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(34.3f, 52.f)), module, Ahriman::INDEX_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 61.f)), module, Ahriman::SPEED_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 61.f)), module, Ahriman::INDEX_CV_INPUT));

                // Row 3: SIZE / DENSE (y=70mm, spacing 12mm)
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(16.5f, 70.f)), module, Ahriman::SIZE_PARAM));
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(34.3f, 70.f)), module, Ahriman::DENSE_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 79.f)), module, Ahriman::SIZE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 79.f)), module, Ahriman::DENSE_CV_INPUT));

                // FSU button (y=88mm)
                addParam(createParamCentered<VCVButton>(mm2px(Vec(25.4f, 88.f)), module, Ahriman::FSU_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 88.f)), module, Ahriman::FSU_GATE_INPUT));
                addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(10.f, 88.f)), module, Ahriman::FSU_LIGHT));

                // MODE and RESPONSE switches (y=98mm and 106mm)
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(25.4f, 98.f)), module, Ahriman::MODE_PARAM));
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(25.4f, 106.f)), module, Ahriman::RESPONSE_PARAM));

                // Audio I/O (y=116mm and 124mm)
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.5f, 116.f)), module, Ahriman::IN_L_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(38.3f, 116.f)), module, Ahriman::IN_R_INPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(12.5f, 124.f)), module, Ahriman::OUT_L_OUTPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(38.3f, 124.f)), module, Ahriman::OUT_R_OUTPUT));
        }
};

Model *modelAhriman = createModel<Ahriman, AhrimanWidget>("Ahriman");
