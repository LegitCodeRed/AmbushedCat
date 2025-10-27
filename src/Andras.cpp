#include "plugin.hpp"
#include "dsp/dsp.hpp"
#include <cmath>
#include <algorithm>
#include <array>
#include <vector>

using namespace rack;

namespace {
struct BitTableOsc {
        static constexpr int tableSize = 256;
        static constexpr int wavesPerMode = 8;

        std::array<std::array<std::array<float, tableSize>, wavesPerMode>, 3> tables{};
        bool initialised = false;
        float phase = 0.f;

        static uint32_t lfsrStep(uint32_t state, uint32_t taps) {
                uint32_t lsb = state & 1u;
                state >>= 1;
                if (lsb)
                        state ^= taps;
                return state ? state : 1u;
        }

        void initTables() {
                if (initialised)
                        return;

                // Mode 0: LFSR - linear feedback shift register patterns
                constexpr uint32_t lfsrTapMask = 0xD0000001u;
                constexpr std::array<uint32_t, wavesPerMode> seeds = {
                        0x13579BDFu, 0x2468ACE1u, 0x89ABCDEFu, 0x10293847u,
                        0x55667788u, 0xABCDEF12u, 0x1F2E3D4Cu, 0x0C0FFEE0u};

                for (size_t wave = 0; wave < wavesPerMode; ++wave) {
                        uint32_t state = seeds[wave];
                        float integrator = 0.f;
                        float norm = 0.f;
                        for (int i = 0; i < tableSize; ++i) {
                                state = lfsrStep(state, lfsrTapMask);
                                float bit = (state & 1u) ? 1.f : -1.f;
                                float nibble = static_cast<float>((state >> 1) & 0x7u) / 3.5f - 1.f;
                                float step = 0.55f * bit + 0.45f * nibble;
                                integrator = 0.82f * integrator + 0.18f * step;
                                tables[0][wave][i] = integrator;
                                norm = std::max(norm, std::fabs(integrator));
                        }
                        if (norm < 1e-3f)
                                norm = 1.f;
                        for (float& sample : tables[0][wave])
                                sample /= norm;
                }

                // Mode 1: SQR - square wave AM'd by harmonic series
                // Each waveform blends between different harmonics
                for (size_t wave = 0; wave < wavesPerMode; ++wave) {
                        int harmonic = static_cast<int>(wave) + 1; // Harmonics 1-8
                        for (int i = 0; i < tableSize; ++i) {
                                float phase = static_cast<float>(i) / static_cast<float>(tableSize);
                                float square = phase < 0.5f ? 1.f : -1.f;
                                float modulator = std::sin(2.f * M_PI * phase * harmonic);
                                tables[1][wave][i] = square * (0.5f + 0.5f * modulator);
                        }
                }

                // Mode 2: SQR2 - like SQR but modulating pitch jumps octave per waveform
                for (size_t wave = 0; wave < wavesPerMode; ++wave) {
                        int octaveShift = static_cast<int>(wave); // 0-7 octaves
                        float modFreq = std::pow(2.f, static_cast<float>(octaveShift));
                        for (int i = 0; i < tableSize; ++i) {
                                float phase = static_cast<float>(i) / static_cast<float>(tableSize);
                                float square = phase < 0.5f ? 1.f : -1.f;
                                float modPhase = phase * modFreq;
                                modPhase -= std::floor(modPhase);
                                float modulator = std::sin(2.f * M_PI * modPhase);
                                tables[2][wave][i] = square * (0.5f + 0.5f * modulator);
                        }
                }

                initialised = true;
        }

        void reset(float position = 0.f) {
                phase = position - std::floor(position);
        }

        float process(float freq, float wave, float shape, float timeMod, int mode, float sampleRate, bool sync) {
                initTables();
                mode = rack::math::clamp(mode, 0, static_cast<int>(tables.size()) - 1);
                float dt = freq / sampleRate;
                dt = rack::math::clamp(dt, 1e-5f, 0.5f);
                if (sync)
                        phase = 0.f;

                float warp = (timeMod - 0.5f) * 1.1f;
                float curvature = 1.f + warp * std::sin(2.f * M_PI * phase);
                float phaseStep = dt * rack::math::clamp(curvature, 0.2f, 1.8f);
                phase += phaseStep;
                phase -= std::floor(phase);

                float idx = phase * tableSize;
                int indexA = static_cast<int>(idx) % tableSize;
                int indexB = (indexA + 1) % tableSize;
                float frac = idx - std::floor(idx);

                const auto& modeTables = tables[mode];

                // Waveform parameter selects which waveform in the table (0-7)
                // Shape parameter controls interpolation/morphing between adjacent waveforms
                float tableIndex = rack::math::clamp(wave, 0.f, 0.999f) * (wavesPerMode - 1);
                int baseWave = static_cast<int>(std::floor(tableIndex));
                int nextWave = rack::math::clamp(baseWave + 1, 0, wavesPerMode - 1);

                // Shape controls how much we blend to the next waveform
                float morphAmount = rack::math::clamp(shape, 0.f, 1.f);

                // Sample from base waveform
                float a0 = modeTables[baseWave][indexA];
                float a1 = modeTables[baseWave][indexB];
                float baseSample = rack::math::crossfade(a0, a1, frac);

                // Sample from next waveform
                float b0 = modeTables[nextWave][indexA];
                float b1 = modeTables[nextWave][indexB];
                float nextSample = rack::math::crossfade(b0, b1, frac);

                // Morph between them based on shape parameter
                float output = rack::math::crossfade(baseSample, nextSample, morphAmount);

                return rack::math::clamp(output, -1.1f, 1.1f);
        }
};

struct NoiseMod {
        float heldPhaseJitter = 0.f;
        float heldAmplitude = 1.f;
        float heldAdd = 0.f;

        void update(float noiseAmt, float sampleRate, bool holdActive) {
                (void)sampleRate;
                if (holdActive)
                        return;
                float randPhase = random::normal() * noiseAmt * 0.004f;
                float randAmp = 1.f + random::normal() * noiseAmt * 0.4f;
                float randAdd = random::normal() * noiseAmt * 0.6f;
                heldPhaseJitter = randPhase;
                heldAmplitude = rack::math::clamp(randAmp, 0.2f, 2.2f);
                heldAdd = rack::math::clamp(randAdd, -1.5f, 1.5f);
        }
};

struct SimpleComb {
        std::vector<float> buffer;
        int index = 0;
        float sampleRate = 44100.f;

        void setSampleRate(float sr) {
                sampleRate = std::max(1000.f, sr);
                int desired = static_cast<int>(std::ceil(sampleRate * 0.02f)) + 4;
                if (desired != static_cast<int>(buffer.size())) {
                        buffer.assign(desired, 0.f);
                        index = 0;
                }
        }

        float process(float in, float freq, float amount) {
                if (buffer.empty())
                        return in;

                float polarity = amount - 0.5f;
                float intensity = std::fabs(polarity) * 2.f;
                if (intensity <= 1e-4f)
                        return in;

                float feedback = 0.2f + 0.5f * intensity;
                float sign = polarity < 0.f ? -1.f : 1.f;
                float delay = 1.f / std::max(freq, 40.f);
                delay = rack::math::clamp(delay, 0.0004f, 0.018f);
                float samples = delay * sampleRate;
                float read = static_cast<float>(index) - samples;
                int size = buffer.size();
                while (read < 0.f)
                        read += size;
                int i0 = static_cast<int>(read) % size;
                int i1 = (i0 + 1) % size;
                float frac = read - std::floor(read);
                float delayed = rack::math::crossfade(buffer[i0], buffer[i1], frac);

                float out = rack::math::crossfade(in, in + delayed * sign, intensity);
                float next = rack::math::crossfade(in, in + delayed * feedback * sign, intensity);
                next = rack::math::clamp(next, -3.f, 3.f);
                buffer[index] = next;
                index = (index + 1) % size;
                return out;
        }
};

struct AsymmetricSoftFold {
        float process(float in, float amount) {
                amount = rack::math::clamp(amount, 0.f, 1.f);
                if (amount <= 1e-4f)
                        return in;

                float bias = 0.5f + 0.5f * rack::math::clamp(in * (1.f + amount * 3.f), -1.f, 1.f);
                bias = rack::math::clamp(bias, 0.f, 1.f);
                float x2 = bias * bias;
                float x3 = x2 * bias;
                float x5 = x3 * x2;
                constexpr float a = 1.6f;
                constexpr float b = 0.6f;
                float folded = bias - a * x3 + b * x5;
                folded = rack::math::clamp((folded - 0.5f) * 2.f, -1.2f, 1.2f);
                float blend = rack::math::clamp(amount * 0.95f, 0.f, 1.f);
                return rack::math::crossfade(in, folded, blend);
        }
};

} // namespace

struct Andras : Module {
        enum ParamIds {
                PITCH_PARAM,
                NOISE_PARAM,
                COMB_PARAM,
                SHAPE_PARAM,
                SOFTFOLD_PARAM,
                WAVE_PARAM,
                TIME_PARAM,
                MODE_PARAM,
                RANGE_PARAM,
                HOLD_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                PITCH_INPUT,
                NOISE_INPUT,
                COMB_INPUT,
                SHAPE_INPUT,
                FOLD_INPUT,
                WAVE_INPUT,
                TIME_INPUT,
                SYNC_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                MAIN_OUTPUT,
                SUB_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                NUM_LIGHTS
        };

        BitTableOsc mainOsc;
        SimpleComb comb;
        AsymmetricSoftFold folder;
        dsp::SchmittTrigger syncTrigger;
        NoiseMod noiseState;
        float subPhase = 0.f;
        float heldPitchCv = 0.f;
        float heldNoiseCv = 0.f;
        float heldCombCv = 0.f;
        float heldShapeCv = 0.f;
        float heldFoldCv = 0.f;
        float heldWaveCv = 0.f;
        float heldTimeCv = 0.f;

        Andras() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(PITCH_PARAM, -3.f, 3.f, 0.f, "Pitch", " oct");
                paramQuantities[PITCH_PARAM]->description = "Fine tuning, press encoder for coarse steps";
                configParam(NOISE_PARAM, 0.f, 1.f, 0.0f, "Noise");
                configParam(COMB_PARAM, 0.f, 1.f, 0.5f, "Comb");
                configParam(SHAPE_PARAM, 0.f, 1.f, 0.5f, "Shape");
                configParam(SOFTFOLD_PARAM, 0.f, 1.f, 0.0f, "Soft Fold");
                configParam(WAVE_PARAM, 0.f, 1.f, 0.f, "Waveform");
                configParam(TIME_PARAM, 0.f, 1.f, 0.f, "Time Mod");
                configSwitch(MODE_PARAM, 0.f, 2.f, 0.f, "Mode", {"I", "II", "III"});
                configSwitch(RANGE_PARAM, 0.f, 2.f, 1.f, "Range", {"Bass", "Alto", "Treble"});
                configButton(HOLD_PARAM, "Hold");

                configInput(PITCH_INPUT, "Pitch CV");
                configInput(NOISE_INPUT, "Noise CV");
                configInput(COMB_INPUT, "Comb CV");
                configInput(SHAPE_INPUT, "Shape CV");
                configInput(FOLD_INPUT, "Soft Fold CV");
                configInput(WAVE_INPUT, "Waveform CV");
                configInput(TIME_INPUT, "Time Mod CV");
                configInput(SYNC_INPUT, "Sync");

                configOutput(MAIN_OUTPUT, "Out");
                configOutput(SUB_OUTPUT, "Sub Out");
        }

        void process(const ProcessArgs& args) override {
                float sampleRate = args.sampleRate;
                comb.setSampleRate(sampleRate);
                bool holdActive = params[HOLD_PARAM].getValue() > 0.5f;

                bool sync = syncTrigger.process(inputs[SYNC_INPUT].getVoltage());
                if (sync)
                        subPhase = 0.f;

                auto sampleCv = [&](int inputId, float& storage) {
                        float cv = inputs[inputId].isConnected() ? inputs[inputId].getVoltage() / 5.f : 0.f;
                        if (!holdActive)
                                storage = cv;
                        return storage;
                };

                float pitchCv = sampleCv(PITCH_INPUT, heldPitchCv);
                float noiseCv = sampleCv(NOISE_INPUT, heldNoiseCv);
                float combCv = sampleCv(COMB_INPUT, heldCombCv);
                float shapeCv = sampleCv(SHAPE_INPUT, heldShapeCv);
                float foldCv = sampleCv(FOLD_INPUT, heldFoldCv);
                float waveCv = sampleCv(WAVE_INPUT, heldWaveCv);
                float timeCv = sampleCv(TIME_INPUT, heldTimeCv);

                float rangeShift = params[RANGE_PARAM].getValue();
                float rangeOffset = (rangeShift - 1.f) * 2.f; // -2,0,+2 octaves

                float pitch = params[PITCH_PARAM].getValue() + pitchCv + rangeOffset;
                float freq = dsp::FREQ_C4 * std::pow(2.f, pitch);
                freq = rack::math::clamp(freq, 5.f, sampleRate * 0.45f);

                float noiseAmt = rack::math::clamp(params[NOISE_PARAM].getValue() + noiseCv, 0.f, 1.f);
                float combAmt = rack::math::clamp(params[COMB_PARAM].getValue() + combCv, 0.f, 1.f);
                float shape = rack::math::clamp(params[SHAPE_PARAM].getValue() + shapeCv, 0.f, 1.f);
                float foldAmt = rack::math::clamp(params[SOFTFOLD_PARAM].getValue() + foldCv, 0.f, 1.f);
                float wave = rack::math::clamp(params[WAVE_PARAM].getValue() + waveCv, 0.f, 1.f);
                float timeMod = rack::math::clamp(params[TIME_PARAM].getValue() + timeCv, 0.f, 1.f);

                noiseState.update(noiseAmt, sampleRate, holdActive);
                float jitter = noiseState.heldPhaseJitter * noiseAmt;
                float amp = rack::math::clamp(noiseState.heldAmplitude, 0.2f, 2.2f);
                float noiseAdd = noiseState.heldAdd;

                int modeIndex = rack::math::clamp(static_cast<int>(std::round(params[MODE_PARAM].getValue())), 0, 2);
                float osc = mainOsc.process(freq * (1.f + jitter), wave, shape, timeMod, modeIndex, sampleRate, sync);
                osc = osc * (1.f - noiseAmt * 0.35f) + random::normal() * noiseAmt * 0.12f;
                osc *= amp;
                osc += noiseAdd * 0.1f;

                float folded = folder.process(osc, foldAmt);
                float combed = comb.process(folded, freq, combAmt);

                subPhase += (freq * 0.5f) / sampleRate;
                if (subPhase >= 1.f)
                        subPhase -= 1.f;
                float sub = (subPhase * 2.f - 1.f); // Convert to bipolar saw wave

                float mainOut = rack::math::clamp(combed, -2.5f, 2.5f) * 5.f;
                float subOut = rack::math::clamp(sub, -1.f, 1.f) * 5.f;

                if (outputs[MAIN_OUTPUT].isConnected())
                        outputs[MAIN_OUTPUT].setVoltage(mainOut);
                if (outputs[SUB_OUTPUT].isConnected())
                        outputs[SUB_OUTPUT].setVoltage(subOut);
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
			auto svg = APP->window->loadSvg(asset::plugin(pluginInstance, "res/Andras.svg"));
			if (svg) {
				svgWidget->setSvg(svg);
			} else {
				WARN("SVG returned null: res/Andras.svg");
			}
		} catch (const std::exception& e) {
			WARN("Exception loading SVG res/Andras.svg: %s", e.what());
			// Leave svgWidget with no SVG; still safe to run.
		}
        }

	void draw(const DrawArgs& args) override {
		// Draw background image first
                std::shared_ptr<Image> image = APP->window->loadImage(imagePath);
                if (image && box.size.x > 0.f && box.size.y > 0.f) {
			int w = box.size.x;
			int h = box.size.y;

			NVGpaint paint = nvgImagePattern(args.vg, 250, 0, w, h, 0.0f, image->handle, 1.0f);
			nvgBeginPath(args.vg);
			nvgRect(args.vg, 0, 0, w, h);
			nvgFillPaint(args.vg, paint);
			nvgFill(args.vg);
		}
		// SVG will be drawn automatically by the child SvgWidget
		Widget::draw(args);
	}
};

struct AndrasWidget : ModuleWidget {
        AndrasWidget(Andras* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Andras.svg")));

                auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size;
		addChild(bg);

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                // Top row - large knobs
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(14.f, 26.f)), module, Andras::PITCH_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(47.f, 26.f)), module, Andras::WAVE_PARAM));

                // Mid row 1 - small knobs
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(14.f, 51.f)), module, Andras::NOISE_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(30.5f, 51.f)), module, Andras::SHAPE_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(47.f, 51.f)), module, Andras::TIME_PARAM));

                // Mid row 2 - small knobs
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(14.f, 79.f)), module, Andras::COMB_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(30.5f, 79.f)), module, Andras::SOFTFOLD_PARAM));

                // Mode switch (vertical 3-position)
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(49.f, 82.f)), module, Andras::MODE_PARAM));

                // Bottom controls
                addParam(createParamCentered<CKSSThreeHorizontal>(mm2px(Vec(18.f, 101.f)), module, Andras::RANGE_PARAM));
                addParam(createParamCentered<TL1105>(mm2px(Vec(49.f, 101.f)), module, Andras::HOLD_PARAM));

                // CV Input Row 1
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.f, 112.f)), module, Andras::PITCH_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.f, 112.f)), module, Andras::NOISE_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.f, 112.f)), module, Andras::COMB_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.f, 112.f)), module, Andras::SHAPE_INPUT));

                // CV Input Row 2
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.f, 120.f)), module, Andras::FOLD_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.f, 120.f)), module, Andras::WAVE_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.f, 120.f)), module, Andras::TIME_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.f, 120.f)), module, Andras::SYNC_INPUT));

                // Output Row
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(52.f, 112.f)), module, Andras::SUB_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(52.f, 120.f)), module, Andras::MAIN_OUTPUT));
        }
};

Model* modelAndras = createModel<Andras, AndrasWidget>("Andras");

