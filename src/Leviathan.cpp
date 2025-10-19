#include "plugin.hpp"
#include <array>
#include <cmath>
#include <algorithm>

namespace {
static constexpr float HUGE_SMOOSH_GAIN = 39810717.f; // 128 dB of drive

struct OnePole {
        float a = 0.f;
        float b = 0.f;
        float z = 0.f;

        void set(float cutoff, float sampleRate) {
                cutoff = rack::math::clamp(cutoff, 1.f, sampleRate * 0.45f);
                float alpha = std::exp(-2.f * M_PI * cutoff / sampleRate);
                a = alpha;
                b = 1.f - alpha;
        }

        float process(float x) {
                z = a * z + b * x;
                return z;
        }

        void reset() {
                z = 0.f;
        }
};

static float applyWavefolder(float x, float amount) {
        if (amount <= 1e-5f)
                return x;

        // More aggressive gain curve - Infinifolder style
        float gain = 1.f + amount * amount * 24.f;
        float folded = x * gain;

        // More iterations for harder crushing
        for (int i = 0; i < 12; ++i) {
                if (folded > 1.f) {
                        folded = 2.f - folded;
                } else if (folded < -1.f) {
                        folded = -2.f - folded;
                }
        }

        // Asymmetric saturation for more character
        float offset = 0.15f * amount;
        folded = std::tanh((folded + offset) * (1.f + amount * 1.5f)) - offset * 0.5f;

        // Less dry blend for more crushing
        float dryBlend = rack::math::clamp(1.f - amount * 1.3f, 0.f, 1.f);
        return rack::math::crossfade(folded, x, dryBlend);
}

struct RectifierStage {
        float sampleRate = 44100.f;
        float dcState = 0.f;
        float alpha = 0.999f;

        void setSampleRate(float sr) {
                sampleRate = std::max(1.f, sr);
                float cutoff = 5.f;
                alpha = std::exp(-2.f * M_PI * cutoff / sampleRate);
        }

        void reset() {
                dcState = 0.f;
        }

        float process(float in, float amount) {
                if (amount <= 1e-5f)
                        return in;

                // Full-wave rectification
                float rect = std::fabs(in);

                // DC blocking
                dcState = rack::math::clamp(alpha * dcState + (1.f - alpha) * rect, -10.f, 10.f);
                float centered = rect - dcState;

                // Much heavier distortion - Pura Ruina style
                // Pre-gain for more aggressive rectification
                float preGain = 1.5f + amount * 4.5f;
                float driven = centered * preGain;

                // Hard clip for more aggressive character
                driven = rack::math::clamp(driven, -3.f, 3.f);

                // Asymmetric waveshaping for octave-up character
                float asymmetry = 0.2f + amount * 0.3f;
                float shaped = std::tanh((driven + asymmetry) * (2.5f + 8.f * amount)) - asymmetry * 0.5f;

                // Add harmonics boost
                shaped *= (1.f + amount * 0.6f);

                // More wet mix at higher amounts
                float wetAmount = amount * (0.8f + amount * 0.4f);
                return rack::math::crossfade(in, shaped, wetAmount);
        }
};

struct NotchFilter {
        float b0 = 1.f, b1 = 0.f, b2 = 0.f;
        float a1 = 0.f, a2 = 0.f;
        float z1 = 0.f, z2 = 0.f;

        void set(float freq, float q, float sampleRate) {
                freq = rack::math::clamp(freq, 20.f, sampleRate * 0.45f);
                q = std::max(0.1f, q);
                float w0 = 2.f * M_PI * freq / sampleRate;
                float cosw = std::cos(w0);
                float alpha = std::sin(w0) / (2.f * q);
                float a0 = 1.f + alpha;
                b0 = 1.f / a0;
                b1 = -2.f * cosw / a0;
                b2 = 1.f / a0;
                a1 = -2.f * cosw / a0;
                a2 = (1.f - alpha) / a0;
        }

        void reset() {
                z1 = z2 = 0.f;
        }

        float process(float in) {
                float out = b0 * in + z1;
                z1 = b1 * in + z2 - a1 * out;
                z2 = b2 * in - a2 * out;
                return out;
        }
};

struct AllpassPhase {
        float a = 0.f;
        float z = 0.f;

        void set(float freq, float sampleRate) {
                freq = rack::math::clamp(freq, 5.f, sampleRate * 0.49f);
                float k = std::tan(M_PI * freq / sampleRate);
                a = (k - 1.f) / (k + 1.f);
        }

        void reset() {
                z = 0.f;
        }

        float process(float in) {
                float y = -a * in + z;
                z = in + a * y;
                return y;
        }
};

struct SubOctaveChorus {
        float sampleRate = 44100.f;
        bool prevPositive = false;
        float subPolarity = 1.f;
        float env = 0.f;
        float detunePhase = 0.f;
        float buzzPhase = 0.f;
        OnePole smoother;

        void setSampleRate(float sr) {
                sampleRate = std::max(1.f, sr);
                smoother.set(100.f, sampleRate);
        }

        void reset() {
                prevPositive = false;
                subPolarity = 1.f;
                env = 0.f;
                detunePhase = 0.f;
                buzzPhase = 0.f;
                smoother.reset();
        }

        float process(float input, float amount) {
                if (amount <= 1e-5f)
                        return input;

                // Simple suboctave: flip polarity on zero crossings
                bool positive = input >= 0.f;
                if (positive != prevPositive) {
                        subPolarity = -subPolarity;
                        prevPositive = positive;
                }

                // Track envelope
                float absIn = std::fabs(input);
                env += 0.002f * (absIn - env);

                // Base suboctave square wave
                float sub = subPolarity * rack::math::clamp(env, 0.f, 1.f);
                sub = smoother.process(sub);

                // Detuning that gets worse as amount increases
                // Slow LFO for pitch warble - starts immediately
                float detuneSpeed = 1.2f + amount * 2.2f; // Faster base speed
                detunePhase += detuneSpeed / sampleRate;
                if (detunePhase >= 1.f)
                        detunePhase -= 1.f;

                float detune = std::sin(2.f * M_PI * detunePhase);

                // Apply detuning - starts early and gets more extreme
                // More linear at start, then accelerates
                float detuneDepth = (0.15f + amount * 0.9f) * amount;
                float detuned = sub * (1.f + detune * detuneDepth);

                // In top quarter, add overtone buzz
                float output = detuned;
                if (amount > 0.75f) {
                        float buzzAmount = (amount - 0.75f) * 4.f; // 0-1 in last quarter

                        // Fast buzz oscillator
                        buzzPhase += 120.f / sampleRate;
                        if (buzzPhase >= 1.f)
                                buzzPhase -= 1.f;

                        // Generate buzz with harmonics
                        float buzz = std::sin(2.f * M_PI * buzzPhase);
                        buzz += 0.5f * std::sin(4.f * M_PI * buzzPhase);
                        buzz += 0.3f * std::sin(6.f * M_PI * buzzPhase);

                        // Mix buzz with envelope
                        output += buzz * env * buzzAmount * 0.6f;
                }

                // Volume increases with amount
                float level = amount;
                return input + output * level;
        }
};

struct MultiBandSaturator {
        OnePole low;
        OnePole lowMid;
        OnePole highMid;
        float sampleRate = 44100.f;

        void setSampleRate(float sr) {
                sampleRate = std::max(1.f, sr);
                low.reset();
                lowMid.reset();
                highMid.reset();
        }

        float process(float in, float emphasis, float centerControl, float smooshBoost) {
                if (sampleRate <= 0.f)
                        return in;

                float centerFreq = 200.f * std::pow(2.f, centerControl * 4.f);
                float width = 0.9f + 1.8f * centerControl;
                float split1 = centerFreq / width;
                float split2 = centerFreq;
                float split3 = centerFreq * width;
                split1 = rack::math::clamp(split1, 40.f, sampleRate * 0.25f);
                split2 = rack::math::clamp(split2, split1 + 10.f, sampleRate * 0.35f);
                split3 = rack::math::clamp(split3, split2 + 10.f, sampleRate * 0.45f);

                low.set(split1, sampleRate);
                lowMid.set(split2, sampleRate);
                highMid.set(split3, sampleRate);

                float lowBand = low.process(in);
                float remaining = in - lowBand;
                float lowMidBand = lowMid.process(remaining);
                remaining -= lowMidBand;
                float highMidBand = highMid.process(remaining);
                float highBand = remaining - highMidBand;

                float segment = rack::math::clamp(emphasis * 3.f, 0.f, 3.f);
                float wLow = 0.f;
                float wLowMid = 0.f;
                float wHighMid = 0.f;
                float wHigh = 0.f;
                if (segment <= 1.f) {
                        wLow = 1.f - segment;
                        wLowMid = segment;
                } else if (segment <= 2.f) {
                        wLowMid = 2.f - segment;
                        wHighMid = segment - 1.f;
                } else {
                        wHighMid = 3.f - segment;
                        wHigh = segment - 2.f;
                }
                float weightSum = wLow + wLowMid + wHighMid + wHigh;
                if (weightSum <= 0.f)
                        weightSum = 1.f;
                wLow /= weightSum;
                wLowMid /= weightSum;
                wHighMid /= weightSum;
                wHigh /= weightSum;

                // Much heavier saturation - Seca Ruina style
                float intensity = rack::math::clamp(0.6f + 0.8f * emphasis + smooshBoost, 0.f, 1.f);
                auto saturateBand = [&](float sample, float weight, float softness) {
                        // More aggressive drive curve
                        float baseDrive = 2.f + (12.f + 28.f * weight) * (0.5f + 1.2f * emphasis + 0.8f * smooshBoost);

                        // Hard clip before tanh for more aggression
                        float preClip = rack::math::clamp(sample * baseDrive * 0.5f, -2.5f, 2.5f);

                        // Asymmetric saturation for character
                        float offset = 0.1f * weight;
                        float shaped = std::tanh((preClip + offset) * 1.8f) - offset * 0.3f;

                        // Post-gain to compensate
                        shaped *= (1.f + 0.5f * weight);

                        return rack::math::crossfade(sample, shaped, rack::math::clamp(intensity * softness, 0.f, 1.f));
                };

                float lowSat = saturateBand(lowBand, wLow, 1.0f);
                float lowMidSat = saturateBand(lowMidBand, wLowMid, 1.1f);
                float highMidSat = saturateBand(highMidBand, wHighMid, 1.15f);
                float highSat = saturateBand(highBand, wHigh, 1.2f);

                float combined = lowSat + lowMidSat + highMidSat + highSat;

                // More wet mix for heavier saturation
                float wetAmount = rack::math::clamp(0.7f + 0.7f * emphasis + 0.4f * smooshBoost, 0.f, 1.f);
                return rack::math::crossfade(in, combined, wetAmount);
        }
};
} // namespace

struct Leviathan : Module {
        enum ParamIds {
                BLEND_PARAM,
                FOLD_PARAM,
                CENTER_PARAM,
                DOOM_PARAM,
                PHASE_PARAM,
                DRIVE_PARAM,
                RECTIFY_PARAM,
                FLOW_PARAM,
                NOTCH_PARAM,
                SMOOSH_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                IN_L_INPUT,
                IN_R_INPUT,
                BLEND_CV_INPUT,
                FOLD_CV_INPUT,
                CENTER_CV_INPUT,
                DOOM_CV_INPUT,
                PHASE_CV_INPUT,
                DRIVE_CV_INPUT,
                RECTIFY_CV_INPUT,
                FLOW_CV_INPUT,
                NOTCH_CV_INPUT,
                SMOOSH_GATE_INPUT,
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
                SMOOSH_LIGHT,
                NUM_LIGHTS
        };

        std::array<RectifierStage, PORT_MAX_CHANNELS> rectifierL{};
        std::array<RectifierStage, PORT_MAX_CHANNELS> rectifierR{};
        std::array<SubOctaveChorus, PORT_MAX_CHANNELS> doomL{};
        std::array<SubOctaveChorus, PORT_MAX_CHANNELS> doomR{};
        std::array<MultiBandSaturator, PORT_MAX_CHANNELS> saturatorL{};
        std::array<MultiBandSaturator, PORT_MAX_CHANNELS> saturatorR{};
        std::array<AllpassPhase, PORT_MAX_CHANNELS> phaseL{};
        std::array<AllpassPhase, PORT_MAX_CHANNELS> phaseR{};
        std::array<NotchFilter, PORT_MAX_CHANNELS> notchL{};
        std::array<NotchFilter, PORT_MAX_CHANNELS> notchR{};

        float bootTimer = 1.2f;
        bool bootActive = true;

        Leviathan() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(BLEND_PARAM, 0.f, 1.f, 0.5f, "Blend");
                configParam(FOLD_PARAM, 0.f, 1.f, 0.25f, "Fold");
                configParam(CENTER_PARAM, 0.f, 1.f, 0.5f, "Center");
                configParam(DOOM_PARAM, 0.f, 1.f, 0.f, "Doom");
                configParam(PHASE_PARAM, 0.f, 1.f, 0.f, "Phase");
                configParam(DRIVE_PARAM, 0.f, 1.f, 0.45f, "Drive");
                configParam(RECTIFY_PARAM, 0.f, 1.f, 0.f, "8vize");
                configSwitch(FLOW_PARAM, 0.f, 2.f, 0.f, "Routing", {"UND", "X", "OVR"});
                configSwitch(NOTCH_PARAM, 0.f, 2.f, 0.f, "Notch", {"Off", "1k", "Track"});
                configButton(SMOOSH_PARAM, "Smoosh");

                configInput(IN_L_INPUT, "Left audio");
                configInput(IN_R_INPUT, "Right audio");
                configInput(BLEND_CV_INPUT, "Blend CV");
                configInput(FOLD_CV_INPUT, "Fold CV");
                configInput(CENTER_CV_INPUT, "Center CV");
                configInput(DOOM_CV_INPUT, "Doom CV");
                configInput(PHASE_CV_INPUT, "Phase CV");
                configInput(DRIVE_CV_INPUT, "Drive CV");
                configInput(RECTIFY_CV_INPUT, "8vize CV");
                configInput(FLOW_CV_INPUT, "Routing CV");
                configInput(NOTCH_CV_INPUT, "Notch CV");
                configInput(SMOOSH_GATE_INPUT, "Smoosh gate");

                configOutput(OUT_L_OUTPUT, "Left audio");
                configOutput(OUT_R_OUTPUT, "Right audio");

                onSampleRateChange();
        }

        void onSampleRateChange() override {
                float sr = APP ? APP->engine->getSampleRate() : 44100.f;
                for (int c = 0; c < PORT_MAX_CHANNELS; ++c) {
                        rectifierL[c].setSampleRate(sr);
                        rectifierR[c].setSampleRate(sr);
                        doomL[c].setSampleRate(sr);
                        doomR[c].setSampleRate(sr);
                        saturatorL[c].setSampleRate(sr);
                        saturatorR[c].setSampleRate(sr);
                        phaseL[c].set(200.f, sr);
                        phaseR[c].set(200.f, sr);
                        notchL[c].set(1000.f, 4.f, sr);
                        notchR[c].set(1000.f, 4.f, sr);
                }
        }

        void onReset() override {
                bootActive = true;
                bootTimer = 1.2f;
                for (int c = 0; c < PORT_MAX_CHANNELS; ++c) {
                        rectifierL[c].reset();
                        rectifierR[c].reset();
                        doomL[c].reset();
                        doomR[c].reset();
                        phaseL[c].reset();
                        phaseR[c].reset();
                        notchL[c].reset();
                        notchR[c].reset();
                }
        }

        float getParamWithCv(int paramId, int inputId, int c) {
                float value = params[paramId].getValue();
                if (inputId >= 0)
                        value += inputs[inputId].getPolyVoltage(c) / 5.f;
                return rack::math::clamp(value, 0.f, 1.f);
        }

        int getSwitchWithCv(int paramId, int inputId, int c) {
                float base = params[paramId].getValue();
                if (inputId >= 0)
                        base += inputs[inputId].getPolyVoltage(c) / 5.f;
                int mode = (int)std::round(base);
                return rack::math::clamp(mode, 0, 2);
        }

        void process(const ProcessArgs& args) override {
                int channels = std::max(inputs[IN_L_INPUT].getChannels(), inputs[IN_R_INPUT].getChannels());
                if (channels == 0)
                        channels = 1;
                outputs[OUT_L_OUTPUT].setChannels(channels);
                outputs[OUT_R_OUTPUT].setChannels(channels);

                float blendParam = params[BLEND_PARAM].getValue();
                bool smoosh = params[SMOOSH_PARAM].getValue() > 0.5f || inputs[SMOOSH_GATE_INPUT].getVoltage() > 2.f;
                float smooshBoost = smoosh ? 0.35f : 0.f;

                bootTimer -= args.sampleTime;
                if (bootTimer <= 0.f)
                        bootActive = false;

                for (int c = 0; c < channels; ++c) {
                        float blend = rack::math::clamp(blendParam + inputs[BLEND_CV_INPUT].getPolyVoltage(c) / 5.f, 0.f, 1.f);
                        float foldAmount = getParamWithCv(FOLD_PARAM, FOLD_CV_INPUT, c);
                        float centerControl = getParamWithCv(CENTER_PARAM, CENTER_CV_INPUT, c);
                        float doomAmount = getParamWithCv(DOOM_PARAM, DOOM_CV_INPUT, c);
                        float phaseAmount = getParamWithCv(PHASE_PARAM, PHASE_CV_INPUT, c);
                        float driveAmount = getParamWithCv(DRIVE_PARAM, DRIVE_CV_INPUT, c);
                        float rectAmount = getParamWithCv(RECTIFY_PARAM, RECTIFY_CV_INPUT, c);
                        int flowMode = getSwitchWithCv(FLOW_PARAM, FLOW_CV_INPUT, c);
                        int notchMode = getSwitchWithCv(NOTCH_PARAM, NOTCH_CV_INPUT, c);

                        float inL = inputs[IN_L_INPUT].getPolyVoltage(c);
                        float inR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getPolyVoltage(c) : inL;
                        float dryL = inL;
                        float dryR = inR;

                        if (smoosh) {
                                inL = rack::math::clamp(inL * HUGE_SMOOSH_GAIN, -15.f, 15.f);
                                inR = rack::math::clamp(inR * HUGE_SMOOSH_GAIN, -15.f, 15.f);
                        }

                        auto applyFlow = [&](float& left, float& right) {
                                auto foldStage = [&](float& lx, float& rx) {
                                        lx = applyWavefolder(lx, foldAmount);
                                        rx = applyWavefolder(rx, foldAmount);
                                };
                                auto doomStage = [&](float& lx, float& rx) {
                                        lx = doomL[c].process(lx, doomAmount);
                                        rx = doomR[c].process(rx, doomAmount);
                                };
                                auto rectStage = [&](float& lx, float& rx) {
                                        lx = rectifierL[c].process(lx, rectAmount);
                                        rx = rectifierR[c].process(rx, rectAmount);
                                };
                                auto satStage = [&](float& lx, float& rx) {
                                        lx = saturatorL[c].process(lx, driveAmount, centerControl, smooshBoost);
                                        rx = saturatorR[c].process(rx, driveAmount, centerControl, smooshBoost);
                                };

                                switch (flowMode) {
                                        case 0: // UND
                                        default:
                                                foldStage(left, right);
                                                doomStage(left, right);
                                                rectStage(left, right);
                                                satStage(left, right);
                                                break;
                                        case 1: // X
                                                foldStage(left, right);
                                                satStage(left, right);
                                                rectStage(left, right);
                                                doomStage(left, right);
                                                break;
                                        case 2: // OVR
                                                foldStage(left, right);
                                                doomStage(left, right);
                                                rectStage(left, right);
                                                satStage(left, right);
                                                foldStage(left, right);
                                                break;
                                }
                        };

                        float left = inL;
                        float right = inR;
                        applyFlow(left, right);

                        float phaseFreq = 40.f + std::pow(phaseAmount, 2.f) * 6000.f;
                        phaseL[c].set(phaseFreq, args.sampleRate);
                        phaseR[c].set(phaseFreq * (1.2f + 0.5f * phaseAmount), args.sampleRate);
                        float phasedL = phaseL[c].process(left);
                        float phasedR = phaseR[c].process(right);
                        left = rack::math::crossfade(left, phasedL, phaseAmount);
                        right = rack::math::crossfade(right, phasedR, phaseAmount);

                        if (notchMode > 0) {
                                float notchFreq = 1000.f;
                                if (notchMode == 2) {
                                        notchFreq = 150.f + centerControl * 4800.f;
                                }
                                notchL[c].set(notchFreq, 4.f, args.sampleRate);
                                notchR[c].set(notchFreq, 4.f, args.sampleRate);
                                left = notchL[c].process(left);
                                right = notchR[c].process(right);
                        }

                        float outL = rack::math::crossfade(dryL, left, blend);
                        float outR = rack::math::crossfade(dryR, right, blend);

                        outputs[OUT_L_OUTPUT].setVoltage(outL, c);
                        outputs[OUT_R_OUTPUT].setVoltage(outR, c);
                }

                lights[SMOOSH_LIGHT].setBrightness(smoosh ? 1.f : 0.f);
                if (bootActive) {
                        float bootProgress = rack::math::clamp(bootTimer / 1.2f, 0.f, 1.f);
                        float fade = 1.f - bootProgress;
                        lights[BOOT_LEFT_LIGHT].setSmoothBrightness(0.8f * fade, args.sampleTime);
                        lights[BOOT_LEFT_CENTER_LIGHT].setSmoothBrightness(0.8f * fade, args.sampleTime);
                        lights[BOOT_RIGHT_CENTER_LIGHT].setSmoothBrightness(0.5f * fade, args.sampleTime);
                        lights[BOOT_RIGHT_LIGHT].setSmoothBrightness(0.9f * fade, args.sampleTime);
                } else {
                        lights[BOOT_LEFT_LIGHT].setBrightness(0.f);
                        lights[BOOT_LEFT_CENTER_LIGHT].setBrightness(0.f);
                        lights[BOOT_RIGHT_CENTER_LIGHT].setBrightness(0.f);
                        lights[BOOT_RIGHT_LIGHT].setBrightness(0.f);
                }
        }
};

struct LeviathanWidget : ModuleWidget {
        LeviathanWidget(Leviathan* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Leviathan.svg")));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(12.0, 23.0)), module, Leviathan::BLEND_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(38.8, 23.0)), module, Leviathan::FOLD_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(12.0, 53.0)), module, Leviathan::CENTER_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(38.8, 53.0)), module, Leviathan::DOOM_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(12.0, 83.0)), module, Leviathan::PHASE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(38.8, 83.0)), module, Leviathan::DRIVE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(12.0, 113.0)), module, Leviathan::RECTIFY_PARAM));

                addParam(createParamCentered<CKSSThree>(mm2px(Vec(38.8, 100.0)), module, Leviathan::FLOW_PARAM));
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(38.8, 110.0)), module, Leviathan::NOTCH_PARAM));
                addParam(createParamCentered<TL1105>(mm2px(Vec(38.8, 118.0)), module, Leviathan::SMOOSH_PARAM));

                // Audio I/O at bottom
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.0, 108.0)), module, Leviathan::IN_L_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(16.0, 108.0)), module, Leviathan::IN_R_INPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(8.0, 118.0)), module, Leviathan::OUT_L_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(16.0, 118.0)), module, Leviathan::OUT_R_OUTPUT));

                // CV inputs next to knobs
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.5, 23.0)), module, Leviathan::BLEND_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(47.0, 23.0)), module, Leviathan::FOLD_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.5, 53.0)), module, Leviathan::CENTER_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(47.0, 53.0)), module, Leviathan::DOOM_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.5, 83.0)), module, Leviathan::PHASE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(47.0, 83.0)), module, Leviathan::DRIVE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.5, 113.0)), module, Leviathan::RECTIFY_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(47.0, 113.0)), module, Leviathan::SMOOSH_GATE_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(25.0, 100.0)), module, Leviathan::FLOW_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(25.0, 110.0)), module, Leviathan::NOTCH_CV_INPUT));

                // Boot lights
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(14.0, 13.0)), module, Leviathan::BOOT_LEFT_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(21.0, 13.0)), module, Leviathan::BOOT_LEFT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(30.0, 13.0)), module, Leviathan::BOOT_RIGHT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(37.0, 13.0)), module, Leviathan::BOOT_RIGHT_LIGHT));
                addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(45.0, 118.0)), module, Leviathan::SMOOSH_LIGHT));
        }
};

Model* modelLeviathan = createModel<Leviathan, LeviathanWidget>("Leviathan");
