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

inline float octaveUpTransform(float x) {
        float s = rack::math::clamp(x, -1.f, 1.f);
        float inner = std::max(0.f, 1.f - s * s);
        return 2.f * s * std::sqrt(inner);
}

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
        std::array<float, NUM_DELAY_LINES> baseMultipliers{0.36f, 0.51f, 0.75f, 1.03f};
        std::array<float, NUM_DELAY_LINES> modScales{0.9f, -0.7f, 0.55f, -0.4f};
        std::array<float, NUM_DELAY_LINES> inputPolarity{1.f, -1.f, 1.f, -1.f};

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
        float shimmerSmoothL = 0.f;
        float shimmerSmoothR = 0.f;
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

                float denseShape = rack::math::crossfade(0.45f, 0.85f, dense);
                float inputGain = rack::math::crossfade(0.18f, 0.32f, dense);

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

                std::array<float, NUM_DELAY_LINES> taps{};
                for (int i = 0; i < NUM_DELAY_LINES; ++i) {
                        float target = baseSamples * baseMultipliers[i];
                        target *= rack::math::crossfade(0.75f, 1.35f, dense);
                        float mod = modSignal * modSeconds * sampleRate * modScales[i];
                        target = rack::math::clamp(target + mod, 8.f, (float)(bufferSize - 8));
                        if (response == 2) {
                                delayTimes[i] = target;
                        } else {
                                delayTimes[i] += (target - delayTimes[i]) * smoothing;
                        }
                        taps[i] = delayLines[i].read(delayTimes[i]);
                }

                float sum0 = taps[0] + taps[1] + taps[2] + taps[3];
                float sum1 = taps[0] - taps[1] + taps[2] - taps[3];
                float sum2 = taps[0] + taps[1] - taps[2] - taps[3];
                float sum3 = taps[0] - taps[1] - taps[2] + taps[3];

                sum0 *= 0.5f;
                sum1 *= 0.5f;
                sum2 *= 0.5f;
                sum3 *= 0.5f;

                std::array<float, NUM_DELAY_LINES> feedbackVector{sum0, sum1, sum2, sum3};

                float wetL = sum1 * 0.65f + sum0 * 0.2f + sum3 * 0.15f;
                float wetR = sum2 * 0.65f + sum0 * 0.2f - sum3 * 0.15f;

                if (mode == 2) {
                        float shimmerL = octaveUpTransform(wetL);
                        float shimmerR = octaveUpTransform(wetR);
                        shimmerSmoothL += 0.08f * (shimmerL - shimmerSmoothL);
                        shimmerSmoothR += 0.08f * (shimmerR - shimmerSmoothR);
                        wetL = rack::math::crossfade(wetL, shimmerSmoothL, 0.45f);
                        wetR = rack::math::crossfade(wetR, shimmerSmoothR, 0.45f);
                }

                for (int i = 0; i < NUM_DELAY_LINES; ++i) {
                        float content = feedbackVector[i];
                        if (mode == 0) {
                                content = rack::math::clamp(content, -1.2f, 1.2f);
                        } else if (mode == 1) {
                                content = std::tanh(content * 1.4f);
                        } else {
                                content = std::tanh(content * 1.1f);
                        }

                        float injection = inputGain * (inSum + inputPolarity[i] * inDiff * 0.6f);
                        if (mode == 2) {
                                float shimmerFeed = (i % 2 == 0) ? shimmerSmoothL : shimmerSmoothR;
                                injection += 0.18f * shimmerFeed;
                        }
                        float writeSample = injection + feedback * (content * denseShape);
                        delayLines[i].write(writeSample);
                }

                auto toneProcess = [&](float &sample, float &lowState, float &highState) {
                        float lowFreq = rack::math::crossfade(900.f, 4500.f, rack::math::clamp((tone + 1.f) * 0.5f, 0.f, 1.f));
                        float highFreq = rack::math::crossfade(1800.f, 400.f, rack::math::clamp((tone + 1.f) * 0.5f, 0.f, 1.f));
                        float lowAlpha = std::exp(-2.f * M_PI * lowFreq / sampleRate);
                        float highAlpha = std::exp(-2.f * M_PI * highFreq / sampleRate);
                        lowState = rack::math::clamp(lowState + (1.f - lowAlpha) * (sample - lowState), -12.f, 12.f);
                        highState = rack::math::clamp(highState + (1.f - highAlpha) * (sample - highState), -12.f, 12.f);
                        float lowPart = lowState;
                        float highPart = sample - highState;
                        float tilt = (tone + 1.f) * 0.5f;
                        float lowGain = rack::math::crossfade(1.6f, 0.6f, tilt);
                        float highGain = rack::math::crossfade(0.6f, 1.6f, tilt);
                        sample = rack::math::clamp(lowPart * lowGain + highPart * highGain, -12.f, 12.f);
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

struct AhrimanWidget : ModuleWidget {
        AhrimanWidget(Ahriman *module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Ahriman.svg")));

                addChild(createWidget<ScrewSilver>(mm2px(Vec(1.5f, 1.5f))));
                addChild(createWidget<ScrewSilver>(mm2px(Vec(1.5f, 125.5f))));
                addChild(createWidget<ScrewSilver>(mm2px(Vec(49.3f, 1.5f))));
                addChild(createWidget<ScrewSilver>(mm2px(Vec(49.3f, 125.5f))));

                addParam(createParamCentered<Davies1900hLargeBlackKnob>(mm2px(Vec(25.4f, 20.f)), module, Ahriman::BLEND_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 34.f)), module, Ahriman::BLEND_CV_INPUT));

                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(16.5f, 44.f)), module, Ahriman::TONE_PARAM));
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(34.3f, 44.f)), module, Ahriman::REGEN_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 54.f)), module, Ahriman::TONE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 54.f)), module, Ahriman::REGEN_CV_INPUT));

                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(16.5f, 62.f)), module, Ahriman::SPEED_PARAM));
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(34.3f, 62.f)), module, Ahriman::INDEX_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 68.f)), module, Ahriman::SPEED_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 68.f)), module, Ahriman::INDEX_CV_INPUT));

                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(16.5f, 80.f)), module, Ahriman::SIZE_PARAM));
                addParam(createParamCentered<Rogan1PSWhite>(mm2px(Vec(34.3f, 80.f)), module, Ahriman::DENSE_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5f, 82.f)), module, Ahriman::SIZE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 82.f)), module, Ahriman::DENSE_CV_INPUT));

                addParam(createParamCentered<VCVButton>(mm2px(Vec(25.4f, 92.f)), module, Ahriman::FSU_PARAM));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3f, 96.f)), module, Ahriman::FSU_GATE_INPUT));

                addParam(createParamCentered<CKSSThree>(mm2px(Vec(25.4f, 106.f)), module, Ahriman::MODE_PARAM));
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(25.4f, 118.f)), module, Ahriman::RESPONSE_PARAM));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.5f, 110.f)), module, Ahriman::IN_L_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(38.3f, 110.f)), module, Ahriman::IN_R_INPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(12.5f, 124.f)), module, Ahriman::OUT_L_OUTPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(38.3f, 124.f)), module, Ahriman::OUT_R_OUTPUT));

                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(14.f, 13.f)), module, Ahriman::BOOT_LEFT_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(21.5f, 13.f)), module, Ahriman::BOOT_LEFT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(29.3f, 13.f)), module, Ahriman::BOOT_RIGHT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(36.8f, 13.f)), module, Ahriman::BOOT_RIGHT_LIGHT));
                addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(25.4f, 101.f)), module, Ahriman::FSU_LIGHT));
        }
};

Model *modelAhriman = createModel<Ahriman, AhrimanWidget>("Ahriman");

