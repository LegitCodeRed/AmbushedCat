#include "plugin.hpp"
#include "dsp/open303/rosic_Open303.h"
#include <cmath>

struct Open303 : Module {
    enum ParamId {
        WAVEFORM_PARAM,
        TUNING_PARAM,
        CUTOFF_PARAM,
        RESONANCE_PARAM,
        ENVMOD_PARAM,
        DECAY_PARAM,
        ACCENT_PARAM,
        VOLUME_PARAM,
        FILTER_TYPE_PARAM,
        AMP_SUSTAIN_PARAM,
        RUN_SEQ_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        PITCH_INPUT,
        GATE_INPUT,
        VELOCITY_INPUT,
        WAVEFORM_CV,
        TUNING_CV,
        CUTOFF_CV,
        RESONANCE_CV,
        ENVMOD_CV,
        DECAY_CV,
        ACCENT_CV,
        VOLUME_CV,
        FILTER_TYPE_CV,
        AMP_SUSTAIN_CV,
        INPUTS_LEN
    };
    enum OutputId {
        AUDIO_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    rosic::Open303 synth;
    dsp::SchmittTrigger gateTrigger[16];
    int countdown[16] = {};
    int noteByChannel[16] = {};
    float priorParams[PARAMS_LEN] = {};
    int every = 0;

    Open303() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configParam(WAVEFORM_PARAM, 0.f, 1.f, 0.f, "Waveform");
        configParam(TUNING_PARAM, 400.f, 480.f, 440.f, "Tuning");
        configParam(CUTOFF_PARAM, 0.f, 2.f, 0.5f, "Cutoff", " Hz", std::pow(2, 10.f), dsp::FREQ_C4 / std::pow(2, 5.f));
        configParam(RESONANCE_PARAM, 0.f, 1.f, 0.707f, "Resonance");
        configParam(ENVMOD_PARAM, 0.f, 1.f, 0.25f, "Env Mod");
        configParam(DECAY_PARAM, 200.f, 2000.f, 400.f, "Decay", " ms");
        configParam(ACCENT_PARAM, 0.f, 1.f, 0.f, "Accent");
        configParam(VOLUME_PARAM, -60.f, 0.f, 0.f, "Volume", " dB");
        configParam(FILTER_TYPE_PARAM, 0.f, (float)rosic::TeeBeeFilter::NUM_MODES, (float)rosic::TeeBeeFilter::TB_303, "Filter Type");
        configParam(AMP_SUSTAIN_PARAM, -60.f, 0.f, -60.f, "Amp Sustain", " dB");
        configParam(RUN_SEQ_PARAM, 0.f, 1.f, 0.f, "Run Seq");

        synth.setSampleRate(APP->engine->getSampleRate());
        for (int i = 0; i < 16; ++i) {
            countdown[i] = -1;
            noteByChannel[i] = -1;
        }
        for (int i = 0; i < PARAMS_LEN; ++i)
            priorParams[i] = -12345768.9f;
    }

    inline bool resetParam(int param, float cvScale, float clampLow, float clampHi, float mul, float &val) {
        int cvid = param - WAVEFORM_PARAM + WAVEFORM_CV;
        float pv = params[param].getValue();
        if (pv != priorParams[param] || (cvid < INPUTS_LEN && inputs[cvid].isConnected())) {
            float cv = 0.f;
            if (cvid < INPUTS_LEN)
                cv = inputs[cvid].getVoltage();
            val = clamp(pv + cv * cvScale, clampLow, clampHi) * mul;
            return true;
        }
        return false;
    }

    void process(const ProcessArgs &args) override {
        if (!outputs[AUDIO_OUTPUT].isConnected())
            return;
        int nChan = inputs[GATE_INPUT].getChannels();
        outputs[AUDIO_OUTPUT].setChannels(1);

        for (int i = 0; i < nChan; ++i) {
            if (gateTrigger[i].process(inputs[GATE_INPUT].getVoltage(i)) && !synth.sequencer.isRunning()) {
                countdown[i] = 8;
            }
            if (countdown[i] > 0) {
                countdown[i]--;
                if (countdown[i] == 0) {
                    noteByChannel[i] = (int)(inputs[PITCH_INPUT].getPolyVoltage(i) * 12.f + 60.f);
                    float vel = 100.f;
                    if (inputs[VELOCITY_INPUT].isConnected())
                        vel = inputs[VELOCITY_INPUT].getPolyVoltage(i) * 12.7f;
                    synth.noteOn(noteByChannel[i], (int)vel, 0);
                }
            }
            if (inputs[GATE_INPUT].getVoltage(i) < 0.5f && noteByChannel[i] >= 0) {
                countdown[i] = -1;
                synth.noteOn(noteByChannel[i], 0, 0);
                noteByChannel[i] = -1;
            }
        }

        if (every == 0) {
            float val;
            if (resetParam(WAVEFORM_PARAM, 10.f, 0.f, 1.f, 1.f, val))
                synth.setWaveform(val);
            if (resetParam(TUNING_PARAM, 3.f, 400.f, 480.f, 1.f, val))
                synth.setTuning(val);
            {
                float cop = params[CUTOFF_PARAM].getValue() * 10.f - 5.f;
                if (cop != priorParams[CUTOFF_PARAM] || inputs[CUTOFF_CV].isConnected()) {
                    float coc = inputs[CUTOFF_CV].getVoltage();
                    float ccc = clamp(cop + coc, 0.f, 10.f);
                    float nco = dsp::FREQ_C4 * std::pow(2.f, ccc);
                    synth.setCutoff(nco);
                }
            }
            if (resetParam(RESONANCE_PARAM, 0.1f, 0.f, 1.f, 100.f, val))
                synth.setResonance(val);
            if (resetParam(ENVMOD_PARAM, 0.1f, 0.f, 1.f, 100.f, val))
                synth.setEnvMod(val);
            if (resetParam(DECAY_PARAM, 2000.f / 5.f, 200.f, 4000.f, 1.f, val))
                synth.setDecay(val);
            if (resetParam(ACCENT_PARAM, 0.1f, 0.f, 1.f, 100.f, val))
                synth.setAccent(val);
            if (resetParam(VOLUME_PARAM, 1.f / 30.f, -60.f, 0.f, 1.f, val))
                synth.setVolume(val);
            if (resetParam(FILTER_TYPE_PARAM, 1.f, 0.f, (float)rosic::TeeBeeFilter::NUM_MODES, 1.f, val))
                synth.filter.setMode((int)val);
            if (resetParam(AMP_SUSTAIN_PARAM, 6.f, -60.f, 0.f, 1.f, val))
                synth.setAmpSustain(val);
            auto sval = params[RUN_SEQ_PARAM].getValue();
            if (sval != priorParams[RUN_SEQ_PARAM]) {
                if (sval) {
                    synth.sequencer.setMode(rosic::AcidSequencer::HOST_SYNC);
                    synth.sequencer.start();
                } else {
                    synth.sequencer.setMode(rosic::AcidSequencer::OFF);
                    synth.sequencer.stop();
                }
            }
            for (int i = 0; i < PARAMS_LEN; ++i)
                priorParams[i] = params[i].getValue();
        }
        every++;
        every %= 16;
        outputs[AUDIO_OUTPUT].setVoltage(synth.getSample() * 10.f);
    }
};

struct Open303Widget : ModuleWidget {
    Open303Widget(Open303* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Open303.svg")));

        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(5.0, 20.0)), module, Open303::WAVEFORM_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.0, 20.0)), module, Open303::TUNING_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(5.0, 35.0)), module, Open303::CUTOFF_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.0, 35.0)), module, Open303::RESONANCE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(5.0, 50.0)), module, Open303::ENVMOD_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.0, 50.0)), module, Open303::DECAY_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(5.0, 65.0)), module, Open303::ACCENT_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.0, 65.0)), module, Open303::VOLUME_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(5.0, 80.0)), module, Open303::FILTER_TYPE_PARAM));
        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.0, 80.0)), module, Open303::AMP_SUSTAIN_PARAM));
        addParam(createParamCentered<CKSS>(mm2px(Vec(5.0, 95.0)), module, Open303::RUN_SEQ_PARAM));

        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 110.0)), module, Open303::PITCH_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.0, 110.0)), module, Open303::GATE_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 120.0)), module, Open303::VELOCITY_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20.0, 120.0)), module, Open303::AUDIO_OUTPUT));
    }
};

Model* modelOpen303 = createModel<Open303, Open303Widget>("Open303");
