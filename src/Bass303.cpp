#include "plugin.hpp"
#include "dsp/open303/rosic_Open303.h"
#include <cmath>




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

    bool gateState = false;
    rosic::Open303 synth;

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
        
        synth.setSlideTime(60.0);
        synth.setVolume(0.0);
    }
    void process(const ProcessArgs& args) override {
        synth.setSampleRate(args.sampleRate);

        bool gate = inputs[GATE_INPUT].getVoltage() >= 1.f;
        float pitchCv = inputs[PITCH_INPUT].getVoltage();
        bool accentGate = inputs[ACCENT_INPUT].isConnected() && inputs[ACCENT_INPUT].getVoltage() >= 1.f;

        int midiNote = clamp((int)std::round(60.f + pitchCv * 12.f), 0, 127);
        int velocity = accentGate ? 127 : 100;

       // Trigger note events on gate changes
        if (gate && !gateState) {
            synth.noteOn(midiNote, velocity, 0.0);
        }
        else if (!gate && gateState) {
            // Velocity zero acts as note-off for the Open303 engine
            synth.noteOn(midiNote, 0, 0.0);
        }
        gateState = gate;

        float cutoff = rack::math::rescale(params[CUTOFF_PARAM].getValue(), 0.f, 1.f, 80.f, 6000.f);
        float resonance = params[RES_PARAM].getValue() * 100.f;
        float envMod = params[ENV_PARAM].getValue() * 100.f;
        float decay = rack::math::rescale(params[DECAY_PARAM].getValue(), 0.f, 1.f, 30.f, 3000.f);
        float accentAmt = params[ACCENT_PARAM].getValue() * 100.f;
        float slideTime = rack::math::rescale(params[SLIDE_PARAM].getValue(), 0.f, 1.f, 0.f, 200.f);
        float waveform = params[WAVE_PARAM].getValue();
        float levelDb = rack::math::rescale(params[LEVEL_PARAM].getValue(), 0.f, 10.f, -60.f, 0.f);

        synth.setCutoff(cutoff);
        synth.setResonance(resonance);
        synth.setEnvMod(envMod);
        synth.setDecay(decay);
        synth.setAccent(accentAmt);
        synth.setSlideTime(slideTime);
        synth.setWaveform(waveform);
        synth.setVolume(levelDb);

        double sample = synth.getSample();
        outputs[AUDIO_OUTPUT].setVoltage(5.f * (float)sample);
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
