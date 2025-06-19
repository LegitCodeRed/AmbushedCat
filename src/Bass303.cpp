#include "plugin.hpp"
#include "dsp/AcidFilter.cpp"
#include "dsp/Saturation.hpp"
#include <cmath>


struct GateEnv {
    float attack = 0.001f;
    float release = 0.2f;
    float value = 0.f;

    float process(bool gate, float dt) {
        float target = gate ? 1.f : 0.f;
        float tau = gate ? attack : release;
        float coeff = dt / tau;
        coeff = rack::math::clamp(coeff, 0.f, 1.f);
        value += (target - value) * coeff;
        return value;
    }
};

struct Bass303 : Module {
    enum ParamId {
        CUTOFF_PARAM,
        RES_PARAM,
        ENV_PARAM,
        DECAY_PARAM,
        ACCENT_PARAM,
        SLIDE_PARAM,
        WAVE_PARAM,
        LEVEL_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        PITCH_INPUT,
        GATE_INPUT,
        ACCENT_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        AUDIO_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    dsp::SchmittTrigger gateTrigger;
    float phase = 0.f;
    AcidFilter filter;
    dspext::Saturator<2> saturator;
    GateEnv env;
    float slidePitch = 0.f;

    Bass303() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(CUTOFF_PARAM, 0.f, 1.f, 0.5f, "Cutoff");
        configParam(RES_PARAM, 0.f, 1.f, 0.2f, "Resonance");
        configParam(ENV_PARAM, 0.f, 1.f, 0.5f, "Env Mod");
        configParam(DECAY_PARAM, 0.f, 1.f, 0.5f, "Decay");
        configParam(ACCENT_PARAM, 0.f, 1.f, 0.f, "Accent Amount");
        configParam(SLIDE_PARAM, 0.f, 1.f, 0.f, "Slide Time");
        configParam(WAVE_PARAM, 0.f, 1.f, 0.f, "Waveform");
        configParam(LEVEL_PARAM, 0.f, 10.f, 0.8f, "Level");
        configInput(PITCH_INPUT, "CV In");
        configInput(GATE_INPUT, "Gate In");
        configInput(ACCENT_INPUT, "Accent In");
        configOutput(AUDIO_OUTPUT, "Audio");
    }

    inline float diodeClip(float x) {
        if (x >= 0.f)
            return std::tanh(x);         // full wave on positive
        else
            return 0.5f * std::tanh(x);  // weaker on negative
    }

    void process(const ProcessArgs& args) override {
        filter.setSampleRate(args.sampleRate);
        float dt = args.sampleTime;

        // --- Inputs ---
        bool gate = inputs[GATE_INPUT].getVoltage() >= 1.f;
        float targetPitch = inputs[PITCH_INPUT].getVoltage();
        float accentGate = (inputs[ACCENT_INPUT].isConnected() && inputs[ACCENT_INPUT].getVoltage() >= 1.f) ? 1.f : 0.f;

        // --- Parameters ---
        float accentAmount = params[ACCENT_PARAM].getValue();
        float accent = accentGate * accentAmount;

        float decayParam = params[DECAY_PARAM].getValue();
        float envDecay = rack::math::rescale(decayParam, 0.f, 1.f, 0.05f, 1.f);
        float punchyDecay = envDecay * (1.f - 0.7f * accent); // accent makes decay snappier
        env.release = std::max(punchyDecay, 0.01f);

        float envVal = env.process(gate, dt);
        float shapedEnv = 1.f - std::exp(-6.f * envVal); // fast, analog-like decay

        // --- Portamento (slide) ---
        float slideTime = rack::math::rescale(params[SLIDE_PARAM].getValue(), 0.f, 1.f, 0.f, 0.5f);
        float coeff = slideTime > 0.f ? rack::math::clamp(dt / slideTime, 0.f, 1.f) : 1.f;
        slidePitch += (targetPitch - slidePitch) * coeff;

        // --- Oscillator ---
        float freq = dsp::FREQ_C4 * std::pow(2.f, slidePitch);
        phase += freq * dt;
        if (phase >= 1.f)
            phase -= 1.f;

        float saw = 2.f * phase - 1.f;
        float square = phase < 0.5f ? 1.f : -1.f;
        float waveMix = params[WAVE_PARAM].getValue();
        float wave = rack::math::crossfade(saw, square, waveMix);

        // --- Filter cutoff modulation ---
        float baseCutoff = rack::math::rescale(params[CUTOFF_PARAM].getValue(), 0.f, 1.f, 80.f, 6000.f);
        float envMod = shapedEnv * params[ENV_PARAM].getValue() * (1.f + 0.5f * accent);
        float cutoff = baseCutoff * (1.f + envMod * 3.f);

        // Optional: pitch glide influences cutoff tone
        float glideMod = rack::math::rescale(slidePitch, 0.f, 10.f, -0.5f, 0.5f);
        cutoff *= std::pow(2.f, glideMod); // glide-to-cutoff tracking

        // --- Resonance modulation ---
        float resKnob = params[RES_PARAM].getValue();
        float resonance = resKnob * (1.f + 0.8f * accent);
        resonance = clamp(resonance, 0.f, 1.f);

        // --- Drive control ---
        float drive = 0.5f + 0.6f * shapedEnv + 0.4f * accent;

        // --- Set filter parameters ---
        filter.setCutoff(cutoff);
        filter.setResonance(resonance);
        filter.setDrive(drive);
        filter.setAccent(accent);
        filter.setEnv(shapedEnv);

        // --- Pre-filter diode-style saturation ---
        float drivenInput = diodeClip(wave * (2.0f + 2.0f * accent));

        // --- Filter processing ---
        float filtered = filter.process(drivenInput);

        // --- Envelope-based amplitude control ---
        float amp = std::tanh(envVal * (1.f + 1.5f * accent));
        float preOut = filtered * amp;

        // --- Post-filter saturation stage ---
        float clipped = saturator.process(preOut, 1.2f, args.sampleRate, dspext::Saturator<2>::MODERATE);

        // --- Output level + soft clip ---
        float level = params[LEVEL_PARAM].getValue() / 10.f; // normalize to 0–1
        float out = std::tanh(clipped * 2.0f) * level;
        outputs[AUDIO_OUTPUT].setVoltage(5.f * out);
    }
};

struct Bass303Widget : ModuleWidget {
    Bass303Widget(Bass303* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Bass303.svg")));

        addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(15.0, 20.0)), module, Bass303::CUTOFF_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 35.0)), module, Bass303::RES_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 50.0)), module, Bass303::ENV_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 65.0)), module, Bass303::DECAY_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 80.0)), module, Bass303::ACCENT_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 95.0)), module, Bass303::SLIDE_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 110.0)), module, Bass303::WAVE_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 120.0)), module, Bass303::LEVEL_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 122.0)), module, Bass303::PITCH_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.0, 122.0)), module, Bass303::GATE_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 128.0)), module, Bass303::ACCENT_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(20.0, 128.0)), module, Bass303::AUDIO_OUTPUT));
    }
};

Model* modelBass303 = createModel<Bass303, Bass303Widget>("Bass303");
