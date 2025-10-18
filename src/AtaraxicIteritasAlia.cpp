#include "plugin.hpp"
#include "dsp/dsp.hpp"
#include <cmath>
#include <vector>

using namespace rack;

namespace {
float polyblep(float phase, float dt) {
        if (phase < dt) {
                float t = phase / dt;
                return t + t - t * t - 1.f;
        }
        if (phase > 1.f - dt) {
                float t = (phase - 1.f) / dt;
                return t * t + t + t + 1.f;
        }
        return 0.f;
}

struct PolyBLEPOsc {
        float phase = 0.f;
        float sampleRate = 44100.f;
        float triangleState = 0.f;

        void setSampleRate(float sr) {
                sampleRate = std::max(1.f, sr);
        }

        void reset(float p = 0.f) {
                phase = p;
        }

        float process(float freq, float morph, float shape, float syncAmount, bool hardSync) {
                float dt = freq / sampleRate;
                dt = std::min(dt, 0.5f);
                if (hardSync)
                        phase = 0.f;

                phase += dt;
                if (phase >= 1.f)
                        phase -= 1.f;

                float basePhase = phase;
                float warped = basePhase + shape * 0.45f * std::sin(2.f * M_PI * basePhase);
                warped -= std::floor(warped);

                float sine = std::sin(2.f * M_PI * warped);

                float saw = 2.f * basePhase - 1.f;
                saw -= polyblep(basePhase, dt);

                float pwm = 0.5f + 0.45f * shape;
                pwm = rack::math::clamp(pwm, 0.02f, 0.98f);
                float square = basePhase < pwm ? 1.f : -1.f;
                square += polyblep(basePhase, dt);
                float t = basePhase - pwm;
                if (t < 0.f)
                        t += 1.f;
                square -= polyblep(t, dt);

                float triangleIncrement = square * dt;
                triangleState += triangleIncrement;
                triangleState -= triangleState * dt * 0.05f;
                triangleState = rack::math::clamp(triangleState, -1.2f, 1.2f);
                float triangle = triangleState;

                float blend1 = rack::math::clamp(morph * 2.f, 0.f, 1.f);
                float blend2 = rack::math::clamp((morph - 0.5f) * 2.f, 0.f, 1.f);
                float analog = rack::math::crossfade(sine, saw, blend1);
                analog = rack::math::crossfade(analog, square, blend2);
                float airy = rack::math::crossfade(analog, triangle, 0.35f * shape);

                float syncMix = rack::math::crossfade(airy, sine, syncAmount);
                return syncMix;
        }
};

struct UnisonEngine {
        static constexpr int NUM_VOICES = 7;
        PolyBLEPOsc voices[NUM_VOICES];
        float detuneSpreads[NUM_VOICES] = {0.f, -0.08f, 0.11f, -0.15f, 0.19f, -0.24f, 0.29f};
        float panSpreads[NUM_VOICES] = {0.f, -0.7f, 0.8f, -0.5f, 0.6f, -0.9f, 1.0f};

        void setSampleRate(float sr) {
                for (int i = 0; i < NUM_VOICES; i++) {
                        voices[i].setSampleRate(sr);
                }
        }

        void reset() {
                for (int i = 0; i < NUM_VOICES; i++) {
                        voices[i].reset(i * 0.03f);
                }
        }

        struct StereoOut {
                float left;
                float right;
        };

        StereoOut process(float freq, float morph, float shape, float syncAmount, bool hardSync, float thickness) {
                StereoOut out = {0.f, 0.f};
                int activeVoices = 1 + static_cast<int>(thickness * (NUM_VOICES - 1));

                for (int i = 0; i < activeVoices; i++) {
                        float detune = 1.f + detuneSpreads[i] * thickness * 0.035f;
                        float voiceFreq = freq * detune;
                        float signal = voices[i].process(voiceFreq, morph, shape, syncAmount, hardSync && (i == 0));

                        float pan = panSpreads[i] * thickness * 0.5f;
                        float leftGain = std::cos((pan + 1.f) * 0.25f * M_PI);
                        float rightGain = std::sin((pan + 1.f) * 0.25f * M_PI);

                        out.left += signal * leftGain;
                        out.right += signal * rightGain;
                }

                float norm = 1.f / std::sqrt(static_cast<float>(activeVoices));
                out.left *= norm;
                out.right *= norm;

                return out;
        }
};

struct AnalogSaturator {
        float memory = 0.f;
        float dcBlocker = 0.f;

        float process(float in, float amount) {
                if (amount <= 0.f)
                        return in;
                float drive = 1.f + amount * 9.f;
                float pre = in + memory * 0.45f;

                float hardClip = rack::math::clamp(pre * drive * 0.7f, -1.5f, 1.5f);
                float soft = std::tanh(pre * drive);
                float asymmetric = soft + hardClip * hardClip * 0.15f;

                float out = rack::math::crossfade(soft, asymmetric, amount * 0.6f);

                dcBlocker = dcBlocker * 0.995f + out - memory;
                memory = rack::math::crossfade(memory, out, 0.35f);

                return rack::math::crossfade(in, dcBlocker, rack::math::clamp(amount * 1.4f, 0.f, 1.f));
        }
};

struct TightFilter {
        dsp::RCFilter hp;
        float sampleRate = 0.f;

        void setSampleRate(float sr) {
                if (std::fabs(sr - sampleRate) < 1e-3f)
                        return;
                sampleRate = sr;
                hp.setCutoffFreq(65.f / sr);
        }

        float process(float in, float tightness) {
                hp.process(in);
                float tight = hp.highpass();
                return rack::math::crossfade(in, tight, tightness * 0.85f);
        }
};

struct TiltEQ {
        dsp::RCFilter low;
        dsp::RCFilter high;
        float sampleRate = 0.f;

        void setSampleRate(float sr) {
                if (std::fabs(sr - sampleRate) < 1e-3f)
                        return;
                sampleRate = sr;
                low.setCutoffFreq(220.f / sr);
                high.setCutoffFreq(1800.f / sr);
        }

        float process(float in, float tilt) {
                low.process(in);
                float lowBand = low.lowpass();
                high.process(in);
                float highBand = high.highpass();
                float bassGain = 1.f + tilt * 0.7f;
                float trebleGain = 1.f - tilt * 0.7f;
                return lowBand * bassGain + highBand * trebleGain;
        }
};

struct ThickComb {
        std::vector<float> buffer;
        size_t index = 0;
        float sampleRate = 44100.f;
        float lowpass = 0.f;

        void setSampleRate(float sr) {
                sampleRate = std::max(1000.f, sr);
                size_t desired = static_cast<size_t>(std::ceil(sampleRate * 0.05f)) + 1;
                if (buffer.size() != desired) {
                        buffer.assign(desired, 0.f);
                        index = 0;
                }
        }

        float process(float in, float freq, float amount) {
                if (buffer.empty() || amount <= 1e-4f)
                        return in;
                freq = std::max(freq, 30.f);
                float delayTime = 0.5f / freq;
                delayTime = rack::math::clamp(delayTime, 0.0006f, 0.035f);
                float delaySamples = delayTime * sampleRate;
                float readIndex = static_cast<float>(index) - delaySamples;
                size_t size = buffer.size();
                while (readIndex < 0.f)
                        readIndex += size;
                size_t i0 = static_cast<size_t>(readIndex) % size;
                size_t i1 = (i0 + 1) % size;
                float frac = readIndex - std::floor(readIndex);
                float delayed = rack::math::crossfade(buffer[i0], buffer[i1], frac);

                lowpass = lowpass * 0.7f + delayed * 0.3f;
                float enhanced = delayed + lowpass * 0.4f * amount;

                float feedback = 0.65f + 0.25f * amount;
                float damping = rack::math::crossfade(0.25f, 0.08f, amount);
                float out = in + enhanced * amount * 1.15f;
                float next = rack::math::clamp(in + enhanced * feedback, -4.5f, 4.5f);
                next = rack::math::crossfade(next, in, damping);
                buffer[index] = next;
                index = (index + 1) % size;
                return out;
        }
};

struct LfsrNoise {
        uint32_t state = 0x7f4a7c15u;
        float held = 0.f;
        int counter = 0;

        float process(float density, float flavour, float timeWarp, float sampleRate, bool hold) {
                if (hold)
                        return held;
                float targetHz = rack::math::rescale(density, 0.f, 1.f, 400.f, 22000.f);
                targetHz *= std::pow(2.f, timeWarp * 1.5f);
                targetHz = rack::math::clamp(targetHz, 40.f, sampleRate * 0.45f);
                float interval = sampleRate / targetHz;
                if (counter <= 0) {
                        counter = std::max(1, static_cast<int>(interval));
                        uint32_t taps = ((state >> 0) ^ (state >> 1) ^ (state >> 3) ^ (state >> 12));
                        state = (state >> 1) | (taps << 31);
                        float stepped = ((state & 0xFFFFu) / 32768.f) - 1.f;
                        float chaotic = ((int32_t)state) / 2147483648.f;
                        float analog = random::normal() * 0.3f;
                        float digital = rack::math::crossfade(stepped, chaotic, 0.45f + 0.4f * flavour);
                        held = rack::math::crossfade(analog, digital, flavour);
                }
                counter--;
                return held;
        }
};

float softFold(float x, float amount) {
        if (amount <= 0.f)
                return x;
        float drive = 1.f + amount * 8.5f;
        float shaped = x * drive;

        float clipped = std::tanh(shaped);
        float folded = std::sin(shaped * 0.8f);
        float hardFold = shaped - 4.f * std::floor(shaped / 4.f + 0.5f);
        hardFold = rack::math::clamp(hardFold, -1.f, 1.f);

        float mixed = rack::math::crossfade(clipped, folded, amount * 0.7f);
        mixed = rack::math::crossfade(mixed, hardFold, amount * amount * 0.3f);

        return mixed;
}

float hardLimiter(float x, float threshold) {
        float absX = std::fabs(x);
        if (absX < threshold)
                return x;
        float over = absX - threshold;
        float compressed = threshold + std::tanh(over * 2.5f) * 0.3f;
        return std::copysign(compressed, x);
}

} // namespace

struct AtaraxicIteritasAlia : Module {
        enum ParamIds {
                PITCH_PARAM,
                WAVE_PARAM,
                NOISE_PARAM,
                SHAPE_PARAM,
                SOFTFOLD_PARAM,
                COMB_PARAM,
                TIME_PARAM,
                DIGITAL_PARAM,
                TONE_PARAM,
                THICKNESS_PARAM,
                HOLD_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                PITCH_INPUT,
                WAVE_INPUT,
                DIGITAL_INPUT,
                TONE_INPUT,
                NOISE_INPUT,
                SHAPE_INPUT,
                TIME_INPUT,
                COMB_INPUT,
                FOLD_INPUT,
                THICKNESS_INPUT,
                SYNC_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                MAIN_OUTPUT,
                SUB_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                DIGITAL_LIGHT,
                ANALOG_LIGHT,
                NUM_LIGHTS
        };

        UnisonEngine unison;
        AnalogSaturator saturator;
        TiltEQ toneEq;
        TightFilter tightFilter;
        ThickComb comb;
        LfsrNoise noise;
        dsp::SchmittTrigger holdTrigger;
        dsp::SchmittTrigger syncTrigger;
        bool holdState = false;
        float subPhase = 0.f;
        float subPhase2 = 0.f;

        AtaraxicIteritasAlia() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(PITCH_PARAM, -3.f, 3.f, 0.f, "Pitch", " oct");
                configParam(WAVE_PARAM, 0.f, 1.f, 0.45f, "Waveform Morph");
                configParam(NOISE_PARAM, 0.f, 1.f, 0.25f, "Noise Density");
                configParam(SHAPE_PARAM, 0.f, 1.f, 0.5f, "Shape");
                configParam(SOFTFOLD_PARAM, 0.f, 1.f, 0.4f, "Aggression");
                configParam(COMB_PARAM, 0.f, 1.f, 0.5f, "Comb Resonance");
                configParam(TIME_PARAM, -1.f, 1.f, 0.f, "Time Mod");
                configParam(DIGITAL_PARAM, 0.f, 1.f, 0.5f, "Digital Grit");
                configParam(TONE_PARAM, -1.f, 1.f, 0.f, "Bass/Treble");
                configParam(THICKNESS_PARAM, 0.f, 1.f, 0.65f, "Thickness");
                configButton(HOLD_PARAM, "Hold LFSR");

                configInput(PITCH_INPUT, "Pitch CV");
                configInput(WAVE_INPUT, "Wave Morph CV");
                configInput(DIGITAL_INPUT, "Grit CV");
                configInput(TONE_INPUT, "Tilt CV");
                configInput(NOISE_INPUT, "Noise CV");
                configInput(SHAPE_INPUT, "Shape CV");
                configInput(TIME_INPUT, "Time Mod CV");
                configInput(COMB_INPUT, "Comb CV");
                configInput(FOLD_INPUT, "Aggression CV");
                configInput(THICKNESS_INPUT, "Thickness CV");
                configInput(SYNC_INPUT, "Sync");

                configOutput(MAIN_OUTPUT, "Main");
                configOutput(SUB_OUTPUT, "Sub");
        }

        void process(const ProcessArgs& args) override {
                float sampleRate = args.sampleRate;
                unison.setSampleRate(sampleRate);
                comb.setSampleRate(sampleRate);
                toneEq.setSampleRate(sampleRate);
                tightFilter.setSampleRate(sampleRate);

                if (holdTrigger.process(params[HOLD_PARAM].getValue()))
                        holdState = !holdState;

                bool sync = syncTrigger.process(inputs[SYNC_INPUT].getVoltage());

                float pitch = params[PITCH_PARAM].getValue();
                if (inputs[PITCH_INPUT].isConnected())
                        pitch += inputs[PITCH_INPUT].getVoltage();
                float freq = dsp::FREQ_C4 * std::pow(2.f, pitch);
                freq = rack::math::clamp(freq, 10.f, sampleRate * 0.45f);

                float wave = params[WAVE_PARAM].getValue() + inputs[WAVE_INPUT].getVoltage() / 5.f;
                float shape = params[SHAPE_PARAM].getValue() + inputs[SHAPE_INPUT].getVoltage() / 5.f;
                float aggressionAmt = params[SOFTFOLD_PARAM].getValue() + inputs[FOLD_INPUT].getVoltage() / 5.f;
                float combAmt = params[COMB_PARAM].getValue() + inputs[COMB_INPUT].getVoltage() / 5.f;
                float timeMod = params[TIME_PARAM].getValue() + inputs[TIME_INPUT].getVoltage() / 5.f;
                float digitalAmt = params[DIGITAL_PARAM].getValue() + inputs[DIGITAL_INPUT].getVoltage() / 5.f;
                float tone = params[TONE_PARAM].getValue() + inputs[TONE_INPUT].getVoltage() / 5.f;
                float noiseAmt = params[NOISE_PARAM].getValue() + inputs[NOISE_INPUT].getVoltage() / 5.f;
                float thickness = params[THICKNESS_PARAM].getValue() + inputs[THICKNESS_INPUT].getVoltage() / 5.f;

                wave = rack::math::clamp(wave, 0.f, 1.f);
                shape = rack::math::clamp(shape, 0.f, 1.f);
                aggressionAmt = rack::math::clamp(aggressionAmt, 0.f, 1.3f);
                combAmt = rack::math::clamp(combAmt, 0.f, 1.f);
                timeMod = rack::math::clamp(timeMod, -1.f, 1.f);
                digitalAmt = rack::math::clamp(digitalAmt, 0.f, 1.f);
                tone = rack::math::clamp(tone, -1.f, 1.f);
                noiseAmt = rack::math::clamp(noiseAmt, 0.f, 1.f);
                thickness = rack::math::clamp(thickness, 0.f, 1.f);

                float timeWarp = std::pow(2.f, timeMod * 1.2f);
                float warpedFreq = freq * timeWarp;
                warpedFreq = rack::math::clamp(warpedFreq, 10.f, sampleRate * 0.45f);

                UnisonEngine::StereoOut oscOut = unison.process(warpedFreq, wave, shape, digitalAmt * 0.45f, sync, thickness);
                float osc = (oscOut.left + oscOut.right) * 0.5f;

                float grit = noise.process(noiseAmt, digitalAmt, timeMod, sampleRate, holdState);
                float mixed = osc * (1.f - noiseAmt * 0.35f) + grit * noiseAmt * 0.8f;

                float folded = softFold(mixed, aggressionAmt * 0.85f);
                float saturated = saturator.process(folded, 0.3f + 0.7f * aggressionAmt);
                float combed = comb.process(saturated, freq, combAmt);
                float tight = tightFilter.process(combed, aggressionAmt * 0.7f);
                float toned = toneEq.process(tight, tone);

                float limited = hardLimiter(toned, 0.85f);

                subPhase += (freq * 0.5f) / sampleRate;
                if (subPhase >= 1.f)
                        subPhase -= 1.f;
                subPhase2 += (freq * 0.25f) / sampleRate;
                if (subPhase2 >= 1.f)
                        subPhase2 -= 1.f;

                float sub1 = std::sin(2.f * M_PI * subPhase);
                float sub2 = std::sin(2.f * M_PI * subPhase2);
                sub1 = std::tanh(sub1 * (1.3f + aggressionAmt * 0.5f));
                sub2 = std::tanh(sub2 * 1.2f);
                float sub = sub1 * 0.7f + sub2 * 0.3f;

                float mainOut = limited * 5.5f;
                float subOut = (sub * 4.2f) + grit * 1.5f * noiseAmt * (1.f - digitalAmt * 0.3f);
                subOut = hardLimiter(subOut, 0.9f);

                if (outputs[MAIN_OUTPUT].isConnected())
                        outputs[MAIN_OUTPUT].setVoltage(mainOut);
                if (outputs[SUB_OUTPUT].isConnected())
                        outputs[SUB_OUTPUT].setVoltage(subOut);

                lights[DIGITAL_LIGHT].setBrightnessSmooth(digitalAmt, args.sampleTime * 2.f);
                lights[ANALOG_LIGHT].setBrightnessSmooth(1.f - digitalAmt, args.sampleTime * 2.f);
        }
};

struct AtaraxicIteritasAliaWidget : ModuleWidget {
        AtaraxicIteritasAliaWidget(AtaraxicIteritasAlia* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/AtaraxicIteritasAlia.svg")));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(14.f, 24.f)), module, AtaraxicIteritasAlia::PITCH_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(47.f, 24.f)), module, AtaraxicIteritasAlia::WAVE_PARAM));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(30.5f, 52.f)), module, AtaraxicIteritasAlia::THICKNESS_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(30.5f, 79.f)), module, AtaraxicIteritasAlia::SOFTFOLD_PARAM));

                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(14.f, 60.f)), module, AtaraxicIteritasAlia::NOISE_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(47.f, 60.f)), module, AtaraxicIteritasAlia::TIME_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(14.f, 88.f)), module, AtaraxicIteritasAlia::COMB_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(47.f, 88.f)), module, AtaraxicIteritasAlia::DIGITAL_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(14.f, 45.f)), module, AtaraxicIteritasAlia::SHAPE_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(30.5f, 104.f)), module, AtaraxicIteritasAlia::TONE_PARAM));

                addParam(createParamCentered<TL1105>(mm2px(Vec(30.5f, 94.f)), module, AtaraxicIteritasAlia::HOLD_PARAM));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.f, 99.f)), module, AtaraxicIteritasAlia::PITCH_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.f, 99.f)), module, AtaraxicIteritasAlia::WAVE_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(36.f, 99.f)), module, AtaraxicIteritasAlia::DIGITAL_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(48.f, 99.f)), module, AtaraxicIteritasAlia::TONE_INPUT));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.f, 111.f)), module, AtaraxicIteritasAlia::NOISE_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.f, 111.f)), module, AtaraxicIteritasAlia::SHAPE_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(36.f, 111.f)), module, AtaraxicIteritasAlia::TIME_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(48.f, 111.f)), module, AtaraxicIteritasAlia::COMB_INPUT));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.f, 123.f)), module, AtaraxicIteritasAlia::FOLD_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(18.f, 123.f)), module, AtaraxicIteritasAlia::THICKNESS_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.f, 123.f)), module, AtaraxicIteritasAlia::SYNC_INPUT));

                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(36.f, 123.f)), module, AtaraxicIteritasAlia::SUB_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(48.f, 123.f)), module, AtaraxicIteritasAlia::MAIN_OUTPUT));

                addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(23.f, 72.f)), module, AtaraxicIteritasAlia::ANALOG_LIGHT));
                addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(38.f, 72.f)), module, AtaraxicIteritasAlia::DIGITAL_LIGHT));
        }
};

Model* modelAtaraxicIteritasAlia = createModel<AtaraxicIteritasAlia, AtaraxicIteritasAliaWidget>("AtaraxicIteritasAlia");

