#include "plugin.hpp"
#include "dsp/dsp.hpp"
#include <array>
#include <cmath>

using namespace rack;

namespace {
static constexpr int kNumPartials = 6;

float saturateFold(float x, float fold) {
        if (fold <= 0.f)
                return x;
        float drive = 1.f + 4.f * fold;
        float clipped = std::tanh(drive * x);
        float folded = std::sin((float)M_PI * clipped);
        return rack::math::crossfade(clipped, folded, fold);
}

struct PercEnvelope {
        float env = 0.f;
        float attackCoef = 0.f;
        float decayCoef = 0.f;
        bool inAttack = false;

        void setTimes(float sampleRate, float attackTime, float decayTime) {
                attackTime = std::max(attackTime, 1e-5f);
                decayTime = std::max(decayTime, 1e-4f);
                attackCoef = attackTime <= 1e-4f ? 0.f : std::exp(-1.f / (attackTime * sampleRate));
                decayCoef = std::exp(-1.f / (decayTime * sampleRate));
        }

        void trigger() {
                inAttack = true;
                env = std::max(env, 0.f);
        }

        float process() {
                if (inAttack) {
                        if (attackCoef <= 0.f) {
                                env = 1.f;
                                inAttack = false;
                        } else {
                                env = 1.f - (1.f - env) * attackCoef;
                                if (env > 0.999f) {
                                        env = 1.f;
                                        inAttack = false;
                                }
                        }
                } else {
                        env *= decayCoef;
                        if (env < 1e-6f)
                                env = 0.f;
                }
                return env;
        }
};

struct Partial {
        float phase = 0.f;
        float baseFreq = 0.f;
        float amp = 0.f;
        float env = 0.f;
        float decay = 0.999f;
        float fmPhase = 0.f;
        float jitter = 0.f;
        float ratio = 1.f;
};

struct NoiseBurst {
        float env = 0.f;
        float decay = 0.99f;

        void trigger(float decayCoef) {
                env = 1.f;
                decay = decayCoef;
        }

        float process() {
                env *= decay;
                if (env < 1e-5f)
                        env = 0.f;
                return env;
        }
};

struct ToneShaper {
        Biquad lowShelf;
        Biquad highShelf;
        int lastTone = -1;
        int lastMode = -1;
        float lastHarm = -100.f;
        float lastFold = -100.f;

        float process(float sample, int tone, int mode, float harmonic, float fold, float sampleRate) {
                bool dirty = false;
                if (tone != lastTone || mode != lastMode)
                        dirty = true;
                if (std::fabs(harmonic - lastHarm) > 0.02f)
                        dirty = true;
                if (std::fabs(fold - lastFold) > 0.02f)
                        dirty = true;

                if (dirty) {
                        static constexpr struct {
                                float lowFreq;
                                float lowGain;
                                float highFreq;
                                float highGain;
                        } toneProfiles[3] = {
                                {65.f, 6.f, 4200.f, -2.f},
                                {110.f, 0.f, 7000.f, 2.f},
                                {180.f, -4.f, 10500.f, 6.f}
                        };
                        tone = rack::math::clamp(tone, 0, 2);
                        auto profile = toneProfiles[tone];

                        float harmonicTilt = (harmonic - 0.5f) * 8.f;
                        float foldEnergy = fold * 6.f;
                        float modeLift = (mode == 2) ? 2.5f : (mode == 1 ? 1.2f : 0.4f);

                        float lowGain = profile.lowGain - 0.35f * harmonicTilt - 0.5f * foldEnergy;
                        float highGain = profile.highGain + harmonicTilt + foldEnergy + modeLift;

                        lowShelf.setLowShelf(sampleRate, profile.lowFreq, lowGain);
                        highShelf.setHighShelf(sampleRate, profile.highFreq, highGain);

                        lastTone = tone;
                        lastMode = mode;
                        lastHarm = harmonic;
                        lastFold = fold;
                }

                float out = lowShelf.process(sample);
                out = highShelf.process(out);
                return out;
        }
};

} // namespace

struct BasimilusIteritasAlia : Module {
        enum ParamIds {
                PITCH_PARAM,
                SPREAD_PARAM,
                MORPH_PARAM,
                FOLD_PARAM,
                HARMONIC_PARAM,
                ATTACK_PARAM,
                DECAY_PARAM,
                MODE_PARAM,
                TONE_PARAM,
                HIT_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                PITCH_INPUT,
                ATTACK_INPUT,
                MODE_INPUT,
                TONE_INPUT,
                SPREAD_INPUT,
                MORPH_INPUT,
                DECAY_INPUT,
                HARMONIC_INPUT,
                FOLD_INPUT,
                TRIG_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                ENV_OUTPUT,
                OUT_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                MODE1_LIGHT,
                MODE2_LIGHT,
                MODE3_LIGHT,
                TONE1_LIGHT,
                TONE2_LIGHT,
                TONE3_LIGHT,
                NUM_LIGHTS
        };

        PercEnvelope envelope;
        NoiseBurst noiseBurst;
        std::array<Partial, kNumPartials> partials;
        ToneShaper toneShaper;
        dsp::SchmittTrigger trigTrigger;
        dsp::SchmittTrigger hitTrigger;
        float baseFreqState = 110.f;
        bool initialized = false;
        float envShape = 0.f;

        BasimilusIteritasAlia() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(PITCH_PARAM, -3.f, 3.f, 0.f, "Pitch", " oct");
                configParam(SPREAD_PARAM, 0.f, 1.f, 0.4f, "Spread");
                configParam(MORPH_PARAM, 0.f, 1.f, 0.3f, "Morph");
                configParam(FOLD_PARAM, 0.f, 1.f, 0.2f, "Fold");
                configParam(HARMONIC_PARAM, 0.f, 1.f, 0.5f, "Harmonic");
                configParam(ATTACK_PARAM, 0.f, 1.f, 0.2f, "Attack");
                configParam(DECAY_PARAM, 0.f, 1.f, 0.6f, "Decay");
                configSwitch(MODE_PARAM, 0.f, 2.f, 0.f, "Mode", {"Skin", "Liquid", "Metal"});
                configSwitch(TONE_PARAM, 0.f, 2.f, 0.f, "Tone", {"Bass", "Alto", "Treble"});
                configButton(HIT_PARAM, "Hit");

                configInput(PITCH_INPUT, "Pitch CV");
                configInput(ATTACK_INPUT, "Attack CV");
                configInput(MODE_INPUT, "Mode CV");
                configInput(TONE_INPUT, "Tone CV");
                configInput(SPREAD_INPUT, "Spread CV");
                configInput(MORPH_INPUT, "Morph CV");
                configInput(DECAY_INPUT, "Decay CV");
                configInput(HARMONIC_INPUT, "Harmonic CV");
                configInput(FOLD_INPUT, "Fold CV");
                configInput(TRIG_INPUT, "Trigger");

                configOutput(ENV_OUTPUT, "Envelope");
                configOutput(OUT_OUTPUT, "Audio");

                onReset();
        }

        void onReset() override {
                envelope = PercEnvelope{};
                noiseBurst = NoiseBurst{};
                for (auto& partial : partials) {
                        partial = Partial{};
                        partial.baseFreq = 110.f;
                        partial.env = 0.f;
                }
                baseFreqState = 110.f;
                initialized = false;
        }

        void updateSpectralTargets(float baseFreq, float spread, float harmonic, int mode, float sampleRate) {
                static constexpr float harmonicRatios[kNumPartials] = {1.f, 2.f, 3.f, 4.f, 5.f, 7.f};
                static constexpr float skinRatios[kNumPartials]    = {1.f, 1.5f, 2.f, 2.5f, 3.5f, 5.f};
                static constexpr float liquidRatios[kNumPartials]  = {1.f, 1.25f, 1.75f, 2.45f, 3.15f, 4.6f};
                static constexpr float metalRatios[kNumPartials]   = {1.f, 1.414f, 1.732f, 2.236f, 2.732f, 3.618f};

                const float* targetRatios = nullptr;
                switch (mode) {
                        case 0: targetRatios = skinRatios; break;
                        case 1: targetRatios = liquidRatios; break;
                        default: targetRatios = metalRatios; break;
                }

                float harmonicWeight = 0.55f + 0.75f * harmonic;

                for (int i = 0; i < kNumPartials; ++i) {
                        float ratio = rack::math::crossfade(harmonicRatios[i], targetRatios[i], spread);
                        ratio = std::max(ratio, 0.1f);
                        float targetFreq = baseFreq * ratio;
                        if (!initialized)
                                partials[i].baseFreq = targetFreq;
                        else
                                partials[i].baseFreq += 0.02f * (targetFreq - partials[i].baseFreq);
                        partials[i].ratio = ratio;

                        float targetAmp = std::pow(ratio, -harmonicWeight);
                        targetAmp *= 1.f + 0.25f * (mode == 2 ? (i % 2 == 0 ? 1.f : -0.4f) : 0.f);
                        targetAmp = std::max(targetAmp, 0.0005f);
                        if (!initialized)
                                partials[i].amp = targetAmp;
                        else
                                partials[i].amp += 0.08f * (targetAmp - partials[i].amp);

                        float partialBase = 0.05f + 0.03f * i;
                        float partialTime = partialBase * (1.3f - 0.6f * harmonic) * (mode == 0 ? 1.15f : 1.f);
                        if (mode == 2)
                                partialTime *= 0.75f;
                        float decayCoef = std::exp(-1.f / (std::max(0.006f, partialTime) * sampleRate));
                        partials[i].decay = decayCoef;
                }
                initialized = true;
        }

        void triggerVoice(float attackTime, float decayTime, float spread, float harmonic, float sampleRate) {
                envelope.setTimes(sampleRate, attackTime, decayTime);
                envelope.trigger();
                float noiseDecayTime = 0.006f + 0.02f * (1.f - attackTime);
                float noiseCoef = std::exp(-1.f / (std::max(0.001f, noiseDecayTime) * sampleRate));
                noiseBurst.trigger(noiseCoef);

                for (int i = 0; i < kNumPartials; ++i) {
                        partials[i].env = 1.f;
                        partials[i].phase = random::uniform();
                        partials[i].fmPhase = random::uniform();
                        float jitterAmt = 0.005f + 0.012f * spread;
                        partials[i].jitter = (random::normal() * jitterAmt) * partials[i].baseFreq;
                }

                envShape = 0.f;
        }

        void process(const ProcessArgs& args) override {
                float spread = rack::math::clamp(params[SPREAD_PARAM].getValue() + inputs[SPREAD_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
                float morph = rack::math::clamp(params[MORPH_PARAM].getValue() + inputs[MORPH_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
                float fold = rack::math::clamp(params[FOLD_PARAM].getValue() + inputs[FOLD_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
                float harmonic = rack::math::clamp(params[HARMONIC_PARAM].getValue() + inputs[HARMONIC_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
                float attackNorm = rack::math::clamp(params[ATTACK_PARAM].getValue() + inputs[ATTACK_INPUT].getVoltage() * 0.1f, 0.f, 1.f);
                float decayNorm = rack::math::clamp(params[DECAY_PARAM].getValue() + inputs[DECAY_INPUT].getVoltage() * 0.1f, 0.f, 1.f);

                float attackTime = 0.0004f + 0.04f * attackNorm * attackNorm;
                float decayTime = 0.06f + 2.4f * decayNorm * decayNorm * decayNorm;

                float pitchParam = params[PITCH_PARAM].getValue();
                float pitchCv = inputs[PITCH_INPUT].getVoltage();
                float pitch = pitchParam + pitchCv;
                float baseFreq = dsp::FREQ_C4 * std::pow(2.f, pitch);
                baseFreq = rack::math::clamp(baseFreq, 10.f, 8000.f);

                if (!initialized)
                        baseFreqState = baseFreq;
                baseFreqState += 0.005f * (baseFreq - baseFreqState);

                float modeControl = params[MODE_PARAM].getValue() + inputs[MODE_INPUT].getVoltage() * 0.2f;
                int mode = rack::math::clamp((int)std::round(modeControl), 0, 2);

                float toneControl = params[TONE_PARAM].getValue() + inputs[TONE_INPUT].getVoltage() * 0.2f;
                int tone = rack::math::clamp((int)std::round(toneControl), 0, 2);

                updateSpectralTargets(baseFreqState, spread, harmonic, mode, args.sampleRate);

                bool trigger = false;
                if (trigTrigger.process(inputs[TRIG_INPUT].getVoltage()))
                        trigger = true;
                if (hitTrigger.process(params[HIT_PARAM].getValue()))
                        trigger = true;
                if (trigger)
                        triggerVoice(attackTime, decayTime, spread, harmonic, args.sampleRate);
                else
                        envelope.setTimes(args.sampleRate, attackTime, decayTime);

                float env = envelope.process();
                envShape += 0.05f * (env - envShape);
                float envPow = env * env;

                float pitchBend = 1.f + spread * 0.7f * envPow;
                float body = 0.f;

                for (int i = 0; i < kNumPartials; ++i) {
                        auto& p = partials[i];
                        float freq = p.baseFreq * pitchBend + p.jitter;
                        freq = std::max(freq, 2.f);
                        p.phase += freq * args.sampleTime;
                        p.phase -= std::floor(p.phase);

                        float sine = std::sin(2.f * (float)M_PI * p.phase);
                        float tri = 2.f * std::fabs(2.f * p.phase - 1.f) - 1.f;
                        float saw = 2.f * p.phase - 1.f;

                        float wave = 0.f;
                        if (mode == 0) {
                                float oddBoost = 0.4f * (1.f - harmonic);
                                float tilt = rack::math::crossfade(sine, tri, morph * 0.6f);
                                wave = tilt + oddBoost * (tri - sine * 0.5f);
                        } else if (mode == 1) {
                                p.fmPhase += (p.baseFreq * (0.3f + 0.8f * morph)) * args.sampleTime;
                                p.fmPhase -= std::floor(p.fmPhase);
                                float fm = std::sin(2.f * (float)M_PI * p.fmPhase);
                                wave = std::sin(2.f * (float)M_PI * (p.phase + 0.25f * morph * fm));
                                wave = rack::math::crossfade(wave, saw, 0.25f * morph);
                        } else {
                                p.fmPhase += (p.baseFreq * (0.5f + 1.2f * morph)) * args.sampleTime;
                                p.fmPhase -= std::floor(p.fmPhase);
                                float fm = std::sin(2.f * (float)M_PI * p.fmPhase);
                                float ring = sine * std::sin(2.f * (float)M_PI * partials[(i + 3) % kNumPartials].phase);
                                float metallic = ring + 0.35f * fm + 0.2f * saw;
                                wave = rack::math::crossfade(sine, metallic, 0.6f + 0.4f * morph);
                        }

                        p.env *= p.decay;
                        body += p.amp * p.env * wave;
                }

                float noiseEnv = noiseBurst.process();
                float noise = noiseEnv * (random::normal() * (0.2f + 0.6f * morph) * (mode == 2 ? 1.6f : 1.f));

                float signal = body + noise;
                signal = saturateFold(signal, fold);
                signal *= envPow;

                float shaped = toneShaper.process(signal, tone, mode, harmonic, fold, args.sampleRate);
                shaped = 5.f * std::tanh(shaped * 0.9f);

                outputs[OUT_OUTPUT].setVoltage(shaped);
                outputs[ENV_OUTPUT].setVoltage(env * 10.f);

                lights[MODE1_LIGHT].setSmoothBrightness(mode == 0 ? envPow : 0.f, args.sampleTime);
                lights[MODE2_LIGHT].setSmoothBrightness(mode == 1 ? envPow : 0.f, args.sampleTime);
                lights[MODE3_LIGHT].setSmoothBrightness(mode == 2 ? envPow : 0.f, args.sampleTime);
                lights[TONE1_LIGHT].setSmoothBrightness(tone == 0 ? env : 0.f, args.sampleTime);
                lights[TONE2_LIGHT].setSmoothBrightness(tone == 1 ? env : 0.f, args.sampleTime);
                lights[TONE3_LIGHT].setSmoothBrightness(tone == 2 ? env : 0.f, args.sampleTime);
        }
};

struct BasimilusIteritasAliaWidget : ModuleWidget {
        BasimilusIteritasAliaWidget(BasimilusIteritasAlia* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/BasimilusIteritasAlia.svg")));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0.f)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0.f)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(14.5f, 26.f)), module, BasimilusIteritasAlia::PITCH_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(30.5f, 21.f)), module, BasimilusIteritasAlia::MORPH_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(46.5f, 26.f)), module, BasimilusIteritasAlia::ATTACK_PARAM));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(14.5f, 52.f)), module, BasimilusIteritasAlia::SPREAD_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(30.5f, 52.f)), module, BasimilusIteritasAlia::FOLD_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(46.5f, 52.f)), module, BasimilusIteritasAlia::DECAY_PARAM));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(14.5f, 78.f)), module, BasimilusIteritasAlia::HARMONIC_PARAM));
                addParam(createParamCentered<TL1105>(mm2px(Vec(30.5f, 78.f)), module, BasimilusIteritasAlia::HIT_PARAM));

                addParam(createParamCentered<CKSSThree>(mm2px(Vec(53.8f, 66.f)), module, BasimilusIteritasAlia::MODE_PARAM));
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(53.8f, 88.f)), module, BasimilusIteritasAlia::TONE_PARAM));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(11.f, 103.f)), module, BasimilusIteritasAlia::PITCH_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.5f, 103.f)), module, BasimilusIteritasAlia::ATTACK_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(38.f, 103.f)), module, BasimilusIteritasAlia::MODE_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(51.5f, 103.f)), module, BasimilusIteritasAlia::TONE_INPUT));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(11.f, 115.f)), module, BasimilusIteritasAlia::SPREAD_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.5f, 115.f)), module, BasimilusIteritasAlia::MORPH_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(38.f, 115.f)), module, BasimilusIteritasAlia::DECAY_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(51.5f, 115.f)), module, BasimilusIteritasAlia::HARMONIC_INPUT));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(11.f, 127.f)), module, BasimilusIteritasAlia::FOLD_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.5f, 127.f)), module, BasimilusIteritasAlia::TRIG_INPUT));

                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(38.f, 127.f)), module, BasimilusIteritasAlia::ENV_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(51.5f, 127.f)), module, BasimilusIteritasAlia::OUT_OUTPUT));

                addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(53.5f, 57.5f)), module, BasimilusIteritasAlia::MODE1_LIGHT));
                addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(58.f, 66.f)), module, BasimilusIteritasAlia::MODE2_LIGHT));
                addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(53.5f, 74.5f)), module, BasimilusIteritasAlia::MODE3_LIGHT));

                addChild(createLightCentered<SmallLight<BlueLight>>(mm2px(Vec(53.5f, 83.5f)), module, BasimilusIteritasAlia::TONE1_LIGHT));
                addChild(createLightCentered<SmallLight<YellowLight>>(mm2px(Vec(58.f, 92.f)), module, BasimilusIteritasAlia::TONE2_LIGHT));
                addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(53.5f, 100.5f)), module, BasimilusIteritasAlia::TONE3_LIGHT));
        }
};

Model* modelBasimilusIteritasAlia = createModel<BasimilusIteritasAlia, BasimilusIteritasAliaWidget>("BasimilusIteritasAlia");
