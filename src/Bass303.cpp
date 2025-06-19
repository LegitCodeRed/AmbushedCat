#include "plugin.hpp"
#include "dsp/DiodeLadder.cpp"
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
    DiodeLadder filter;
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

    void process(const ProcessArgs& args) override {
        filter.setSampleRate(args.sampleRate);
        float dt = args.sampleTime;

        env.release = rack::math::rescale(params[DECAY_PARAM].getValue(), 0.f, 1.f, 0.05f, 1.f);

        bool gate = inputs[GATE_INPUT].getVoltage() >= 1.f;
        float envVal = env.process(gate, dt);

        float targetPitch = inputs[PITCH_INPUT].getVoltage();
        float slideTime = rack::math::rescale(params[SLIDE_PARAM].getValue(), 0.f, 1.f, 0.f, 0.5f);
        float coeff = slideTime > 0.f ? rack::math::clamp(dt / slideTime, 0.f, 1.f) : 1.f;
        slidePitch += (targetPitch - slidePitch) * coeff;

        float freq = dsp::FREQ_C4 * std::pow(2.f, slidePitch);
        phase += freq * dt;
        if (phase >= 1.f)
            phase -= 1.f;

        float saw = 2.f * phase - 1.f;
        float square = phase < 0.5f ? 1.f : -1.f;
        float mix = params[WAVE_PARAM].getValue();
        float wave = rack::math::crossfade(saw, square, mix);

        float accentGate = inputs[ACCENT_INPUT].isConnected() && inputs[ACCENT_INPUT].getVoltage() >= 1.f ? 1.f : 0.f;
        float accent = accentGate * params[ACCENT_PARAM].getValue();

        float cutoffKnob = params[CUTOFF_PARAM].getValue();
        float cutoff = rack::math::rescale(cutoffKnob, 0.f, 1.f, 80.f, 6000.f);
        cutoff *= 1.f + envVal * params[ENV_PARAM].getValue() * (1.f + accent) * 2.f;
        float resonance = params[RES_PARAM].getValue();
        filter.setCutoff(cutoff);
        filter.setResonance(resonance);
        filter.setDrive(0.5f);
        float filtered = filter.process(wave);

        float out = filtered * envVal * (1.f + accent) * params[LEVEL_PARAM].getValue();
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
