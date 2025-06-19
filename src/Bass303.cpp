#include "plugin.hpp"
#include <cmath>

struct AcidFilter {
    float cutoff = 1000.f;
    float resonance = 0.1f;
    float sampleRate = 44100.f;
    float s1 = 0.f, s2 = 0.f, s3 = 0.f, s4 = 0.f;

    void setSampleRate(float sr) {
        sampleRate = sr;
    }

    void setParams(float fc, float res) {
        cutoff = fc;
        resonance = res;
    }

    float process(float in) {
        float f = cutoff / sampleRate;
        f = rack::math::clamp(f, 0.f, 0.49f);
        float fb = resonance * (1.f - 0.15f * f * f);

        in -= fb * s4;
        in *= 0.35013f * (f * f) * (f * f);
        s1 = in + 0.3f * s1 + 1e-20f;
        s2 = s1 + 0.3f * s2;
        s3 = s2 + 0.3f * s3;
        s4 = s3 + 0.3f * s4;
        return s4;
    }
};

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
        WAVE_PARAM,
        LEVEL_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        PITCH_INPUT,
        GATE_INPUT,
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
    GateEnv env;

    Bass303() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(CUTOFF_PARAM, 0.f, 1.f, 0.5f, "Cutoff");
        configParam(RES_PARAM, 0.f, 1.f, 0.2f, "Resonance");
        configParam(ENV_PARAM, 0.f, 1.f, 0.5f, "Env Amount");
        configParam(WAVE_PARAM, 0.f, 1.f, 0.f, "Waveform");
        configParam(LEVEL_PARAM, 0.f, 1.f, 0.8f, "Level");
        configInput(PITCH_INPUT, "V/Oct");
        configInput(GATE_INPUT, "Gate");
        configOutput(AUDIO_OUTPUT, "Audio");
    }

    void process(const ProcessArgs& args) override {
        filter.setSampleRate(args.sampleRate);
        float dt = args.sampleTime;

        bool gate = inputs[GATE_INPUT].getVoltage() >= 1.f;
        float envVal = env.process(gate, dt);

        float pitch = inputs[PITCH_INPUT].getVoltage();
        float freq = dsp::FREQ_C4 * std::pow(2.f, pitch);
        phase += freq * dt;
        if (phase >= 1.f)
            phase -= 1.f;

        float saw = 2.f * phase - 1.f;
        float square = phase < 0.5f ? 1.f : -1.f;
        float wave = (params[WAVE_PARAM].getValue() < 0.5f) ? saw : square;

        float cutoffKnob = params[CUTOFF_PARAM].getValue();
        float cutoff = rack::math::rescale(cutoffKnob, 0.f, 1.f, 80.f, 6000.f);
        cutoff *= 1.f + envVal * params[ENV_PARAM].getValue() * 2.f;
        float resonance = params[RES_PARAM].getValue();
        filter.setParams(cutoff, resonance);
        float filtered = filter.process(wave);

        float out = filtered * envVal * params[LEVEL_PARAM].getValue();
        outputs[AUDIO_OUTPUT].setVoltage(5.f * out);
    }
};

struct Bass303Widget : ModuleWidget {
    Bass303Widget(Bass303* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/TuringMaschine.svg")));

        addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(15.0, 30.0)), module, Bass303::CUTOFF_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 50.0)), module, Bass303::RES_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 70.0)), module, Bass303::ENV_PARAM));
        addParam(createParamCentered<CKSS>(mm2px(Vec(15.0, 90.0)), module, Bass303::WAVE_PARAM));
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 110.0)), module, Bass303::LEVEL_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 120.0)), module, Bass303::PITCH_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.0, 120.0)), module, Bass303::GATE_INPUT));
        addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(15.0, 130.0)), module, Bass303::AUDIO_OUTPUT));
    }
};

Model* modelBass303 = createModel<Bass303, Bass303Widget>("Bass303");
