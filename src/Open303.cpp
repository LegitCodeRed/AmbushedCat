#include "plugin.hpp"
#include "dsp/open303/rosic_Open303.h"
#include <cmath>

struct Open303 : Module {
   enum ParamIds
    {
        WAVEFORM_KNOB,
        TUNING_KNOB,
        CUTOFF_KNOB,
        RESONANCE_KNOB,
        ENVMOD_KNOB,
        DECAY_KNOB,
        ACCENT_KNOB,
        VOLUME_KNOB,
        FILTER_TYPE_KNOB,
        AMP_SUSTAIN_KNOB,
        PRE_FILTER_HPF_KNOB,
        FEEDBACK_HPF_KNOB,
        POST_FILTER_HPF_KNOB,
        SQUARE_PHASE_SHIFT_KNOB,

        RUN_SEQ_SWITCH,

        NUM_PARAMS
    };

    enum InputId
    {
        NOTE_PITCH,
        NOTE_GATE,
        NOTE_VEL,

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
        PRE_FILTER_HPF_CV,
        FEEDBACK_HPF_CV,
        POST_FILTER_HPF_CV,
        SQUARE_PHASE_SHIFT_CV,

        NUM_INPUTS
    };

    enum OutputId
    {
        SIGNAL,
        NUM_OUTPUTS
    };

    enum LightIds
    {
        NUM_LIGHTS
    };

    rosic::Open303 open303;

    Open303() : Module()
    {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
        // The VST is a good indication of ranges here
        configParam(WAVEFORM_KNOB, 0, 1, 0);
        configParam(TUNING_KNOB, 400, 480, 440);
        configParam(CUTOFF_KNOB, 0.f, 2.f, 0.5f, "Frequency", " Hz", std::pow(2, 10.f),
                    dsp::FREQ_C4 / std::pow(2, 5.f));
        configParam(RESONANCE_KNOB, 0., 1.0, 0.707);
        configParam(ENVMOD_KNOB, 0., 1.0, 0.25);
        configParam(DECAY_KNOB, 200, 2000, 400, "Decay Time", " ms");
        configParam(VOLUME_KNOB, -60.0, 0.0, 0, "Volume", "dB"); // ct_decibel_narrow
        configParam(FILTER_TYPE_KNOB, 0, rosic::TeeBeeFilter::NUM_MODES,
                    rosic::TeeBeeFilter::TB_303);

        configParam(AMP_SUSTAIN_KNOB, -60, 0, -60);
        configParam(PRE_FILTER_HPF_KNOB, 10, 500, 44.5);
        configParam(FEEDBACK_HPF_KNOB, 10, 500, 150);
        configParam(POST_FILTER_HPF_KNOB, 10, 500, 24);
        configParam(SQUARE_PHASE_SHIFT_KNOB, 0, 360, 189);

        configParam(RUN_SEQ_SWITCH, 0, 1, 0, "Run Sequencer");

        open303.setSampleRate(APP->engine->getSampleRate());
        for (int i = 0; i < 16; ++i)
        {
            countdown[i] = -1;
            noteByChannel[i] = -1;
        }

        INFO("Pattern count: %d", open303.sequencer.getNumPatterns());
        auto sq = &(open303.sequencer);
        for (int i = 0; i < 4; ++i)
        {
            sq->setKey(0, i, 0);
            if (i == 0)
                sq->setAccent(0, i, 1);
            sq->setGate(0, i, true);
            if (i != 3)
                sq->setSlide(0, i, true);
        }

        sq->setKey(0, 4, 3);
        sq->setGate(0, 4, true);
        sq->setSlide(0, 4, true);
        sq->setKey(0, 5, 3);
        sq->setGate(0, 5, true);
        sq->setSlide(0, 5, true);

        sq->setGate(0, 6, true);
        sq->setKey(0, 6, 5);
        sq->setSlide(0, 6, true);
        sq->setGate(0, 7, true);
        sq->setKey(0, 7, 5);
        for (int i = 8; i < 16; i += 2)
        {
            sq->setGate(0, i, true);
            sq->setAccent(0, i, i % 4);
            sq->setKey(0, i, 12 - i / 2);
            if (i == 14)
            {
                sq->setGate(0, i + 1, true);
                sq->setAccent(0, i + 1, true);
                sq->setKey(0, i + 1, 12 - i / 2);
                sq->setSlide(0, i + 1, true);
            }
        }

        for (int i = 0; i < NUM_PARAMS; ++i)
            priorParams[i] = -12345768.9f;
    }

    int countdown[16];
    rack::dsp::SchmittTrigger gateTrigger[16];
    int noteByChannel[16];
    int every = 0;

    float priorParams[NUM_PARAMS];

    inline bool resetParam(int param, float cvScale, float clampLow, float clampHi, float mul,
                           float &val)
    {
        int cvid = param - WAVEFORM_KNOB + WAVEFORM_CV;
        float pv = params[param].getValue();
        if (pv != priorParams[param] || inputs[cvid].isConnected())
        {
            float cv = inputs[cvid].getVoltage();
            val = rack::clamp(pv + cv * cvScale, clampLow, clampHi) * mul;
            return true;
        }

        return false;
    }

    void process(const ProcessArgs &args) override
    {
        if (!outputs[SIGNAL].isConnected())
            return;

        int nChan = inputs[NOTE_GATE].getChannels();
        outputs[SIGNAL].setChannels(1);

        for (int i = 0; i < nChan; ++i)
        {
            if (gateTrigger[i].process(inputs[NOTE_GATE].getVoltage(i)) &&
                !open303.sequencer.isRunning())
            {
                countdown[i] = 8;
            }
            if (countdown[i] > 0)
            {
                countdown[i]--;
                if (countdown[i] == 0)
                {
                    noteByChannel[i] = (int)(inputs[NOTE_PITCH].getPolyVoltage(i) * 12.0 + 60.0);
                    float vel;
                    if (inputs[NOTE_VEL].isConnected())
                        vel = inputs[NOTE_VEL].getPolyVoltage(i) * 12.7;
                    else
                        vel = 100.0;
                    open303.noteOn(noteByChannel[i], (int)vel, 0);
                }
            }
            if (inputs[NOTE_GATE].getVoltage(i) < 0.5 && noteByChannel[i] >= 0)
            {
                countdown[i] = -1;
                open303.noteOn(noteByChannel[i], 0, 0);
                noteByChannel[i] = -1;
            }
        }

        if (every == 0) // presuppose i may want to throttle all these checks and resets
        {
            float val;
            if (resetParam(WAVEFORM_KNOB, 10.0, 0.0, 1.0, 1.0, val))
                open303.setWaveform(val);
            if (resetParam(TUNING_KNOB, 3, 400, 480, 1, val))
                open303.setTuning(val);

            // fix this later
            {
                float cop = params[CUTOFF_KNOB].getValue() * 10.0 - 5.f;
                if (cop != priorParams[CUTOFF_KNOB] || inputs[CUTOFF_CV].isConnected())
                {
                    float coc = inputs[CUTOFF_CV].getVoltage();
                    float ccc = rack::clamp(cop + coc, 0.f, 10.f);
                    float nco = dsp::FREQ_C4 * pow(2.f, ccc);
                    open303.setCutoff(nco);
                }
            }

            if (resetParam(RESONANCE_KNOB, 0.1, 0, 1, 100, val))
                open303.setResonance(val);
            if (resetParam(ENVMOD_KNOB, 0.1, 0, 1, 100, val))
                open303.setEnvMod(val);
            if (resetParam(DECAY_KNOB, 2000 / 5, 200, 4000, 1, val))
                open303.setDecay(val);
            if (resetParam(ACCENT_KNOB, 0.1, 0, 1, 100, val))
                open303.setAccent(val);
            if (resetParam(VOLUME_KNOB, 1.0 / 30.0, -60, 0, 1, val))
                open303.setVolume(val);

            // fix this later
            {
                float rep = params[FILTER_TYPE_KNOB].getValue();
                if (rep != priorParams[FILTER_TYPE_KNOB])
                {
                    int fm = (int)rep;
                    open303.filter.setMode(fm);
                }
            }

            if (resetParam(AMP_SUSTAIN_KNOB, 6.0, -60, 0, 1, val))
                open303.setAmpSustain(val);

            auto sval = params[RUN_SEQ_SWITCH].getValue();
            if (sval != priorParams[RUN_SEQ_SWITCH])
            {
                if (sval)
                {
                    open303.sequencer.setMode(rosic::AcidSequencer::HOST_SYNC);
                    open303.sequencer.start();
                }
                else
                {
                    open303.sequencer.setMode(rosic::AcidSequencer::OFF);
                    open303.sequencer.stop();
                }
            }

            for (int i = 0; i < NUM_PARAMS; ++i)
                priorParams[i] = params[i].getValue();
        }
        every++;
        every = every % 16;

        outputs[SIGNAL].setVoltage(open303.getSample() * 10.0);
    }
};

struct Open303Widget : ModuleWidget {
    Open303Widget(Open303* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/Tape.svg")));
        box.size = Vec(RACK_GRID_WIDTH * 29, RACK_GRID_HEIGHT);

        std::vector<std::string> params = {"waveform", "tuning", "cutoff", "res",     "envmod",
                                        "decay",    "accent", "volume", "filttype"};
        float xp = 10;
        float yp = 30;
        int i = 0;
        float pw = 45, ph = 75;
        for (auto s : params)
        {
       
            addParam(rack::createParam<rack::RoundSmallBlackKnob>(
                rack::Vec(xp + pw / 2 - 12, yp + 15), module, Open303::WAVEFORM_KNOB + i));
            addInput(rack::createInput<rack::PJ301MPort>(rack::Vec(xp + pw / 2 - 12, yp + 43), module,
                                                        Open303::WAVEFORM_CV + i));

            xp += pw;
            if (xp > box.size.x - pw - 10)
            {
                xp = 10;
                yp += ph;
            }
            i++;
        }

        addParam(
            rack::createParam<rack::CKSS>(rack::Vec(10, 130), module, Open303::RUN_SEQ_SWITCH));



        rack::Vec inP = Vec(10, RACK_GRID_HEIGHT - 15 - 43);
        std::vector<std::string> lab = {"1v/o", "gate", "vel"};
        for (int i = 0; i < 3; ++i)
        {
            addInput(rack::createInput<rack::PJ301MPort>(inP, module, Open303::NOTE_PITCH + i));
            inP.x += 35;
        }

        addOutput(rack::createOutput<rack::PJ301MPort>(inP, module, Open303::SIGNAL));
        }
    };

Model* modelOpen303 = createModel<Open303, Open303Widget>("Open303");
