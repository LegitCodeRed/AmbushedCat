#include "plugin.hpp"
#include "SitriBus.hpp"

#include <cmath>
#include <string>

using rack::math::clamp;

struct Lilith : rack::engine::Module {
        enum ParamIds {
                STEPS_PARAM,
                TRIGLEN_PARAM,
                CV_PARAMS_BASE,
                MODE_PARAMS_BASE = CV_PARAMS_BASE + 8,
                NUM_PARAMS = MODE_PARAMS_BASE + 8
        };
        enum InputIds {
                CLK_INPUT,
                RESET_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                CV_OUTPUT,
                GATE_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                RUN_LIGHT,
                STEP_LIGHT_BASE,
                GATE_LIGHT_BASE = STEP_LIGHT_BASE + 8,
                NUM_LIGHTS = GATE_LIGHT_BASE + 8
        };

        dsp::SchmittTrigger clockTrigger;
        dsp::SchmittTrigger resetTrigger;
        dsp::PulseGenerator runPulse;

        int currentStep = 0;
        float triggerTimer = 0.f;

        SitriBus::ExpanderToMaster expanderMessage{};

        Lilith() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                auto* stepsQuantity = configParam(STEPS_PARAM, 1.f, 8.f, 8.f, "Number of active steps", " steps");
                stepsQuantity->snapEnabled = true;

                configParam(TRIGLEN_PARAM, 0.001f, 0.1f, 0.01f, "Trigger length", " ms", 1000.f);

                for (int i = 0; i < 8; ++i) {
                        configParam(CV_PARAMS_BASE + i, -10.f, 10.f, 0.f,
                                    "Step " + std::to_string(i + 1) + " CV", " V");
                        auto* modeQuantity = configSwitch(
                            MODE_PARAMS_BASE + i, 0.f, 2.f, 0.f, "Gate mode",
                            {"Expand", "Mute", "Trigger"});
                        modeQuantity->snapEnabled = true;
                }

                configInput(CLK_INPUT, "Clock");
                configInput(RESET_INPUT, "Reset");
                configOutput(CV_OUTPUT, "CV");
                configOutput(GATE_OUTPUT, "Gate");

                expanderMessage.magic = SitriBus::MAGIC;
                expanderMessage.version = 1;
                for (int i = 0; i < 8; ++i) {
                        expanderMessage.gateMode[i] = SitriBus::GateMode::EXPAND;
                        expanderMessage.stepCV[i] = 0.f;
                }
                leftExpander.producerMessage = &expanderMessage;
        }

        void process(const ProcessArgs& args) override {
                int knobSteps = clamp((int)std::round(params[STEPS_PARAM].getValue()), 1, 8);
                float trigLenSec = clamp(params[TRIGLEN_PARAM].getValue(), 0.001f, 0.1f);

                Module* leftModule = leftExpander.module;
                bool attachedToSitri = leftModule && leftModule->model == modelSitri;
                const SitriBus::MasterToExpander* busMessage = nullptr;
                if (attachedToSitri && leftExpander.consumerMessage) {
                        auto* msg = reinterpret_cast<const SitriBus::MasterToExpander*>(leftExpander.consumerMessage);
                        if (msg && msg->magic == SitriBus::MAGIC && msg->version == 1)
                                busMessage = msg;
                }

                bool jackReset = resetTrigger.process(inputs[RESET_INPUT].getVoltage());
                bool clockEdge = false;
                bool enteringStep = false;
                bool resetEdge = false;

                int activeSteps = knobSteps;
                if (attachedToSitri && busMessage)
                        activeSteps = clamp((int)busMessage->numSteps, 1, 8);

                if (attachedToSitri) {
                        if (busMessage) {
                                int targetStep = clamp((int)busMessage->stepIndex, 1, activeSteps) - 1;
                                targetStep = clamp(targetStep, 0, activeSteps - 1);
                                if (targetStep != currentStep)
                                        enteringStep = true;
                                clockEdge = busMessage->clockEdge != 0;
                                resetEdge = busMessage->resetEdge != 0;
                                currentStep = targetStep;
                        }
                } else {
                        bool clkTrig = clockTrigger.process(inputs[CLK_INPUT].getVoltage());
                        if (clkTrig) {
                                clockEdge = true;
                                enteringStep = true;
                                currentStep = (currentStep + 1) % activeSteps;
                        }
                }

                if (jackReset) {
                        resetEdge = true;
                }

                if (resetEdge) {
                        currentStep = 0;
                        enteringStep = true;
                }

                if (currentStep >= activeSteps) {
                        currentStep = activeSteps - 1;
                        enteringStep = true;
                }
                if (currentStep < 0)
                        currentStep = 0;

                if (clockEdge)
                        runPulse.trigger(0.02f);
                if (clockEdge)
                        enteringStep = true;

                int stepIndex = clamp(currentStep, 0, activeSteps - 1);
                int modeValue = clamp((int)std::round(params[MODE_PARAMS_BASE + stepIndex].getValue()), 0, 2);

                if (modeValue == SitriBus::GateMode::TRIGGER) {
                        if (enteringStep)
                                triggerTimer = trigLenSec;
                        if (triggerTimer > 0.f) {
                                triggerTimer -= args.sampleTime;
                                if (triggerTimer < 0.f)
                                        triggerTimer = 0.f;
                        }
                } else {
                        triggerTimer = 0.f;
                }

                bool gateHigh = false;
                switch (modeValue) {
                case SitriBus::GateMode::EXPAND:
                        gateHigh = true;
                        break;
                case SitriBus::GateMode::MUTE:
                        gateHigh = false;
                        break;
                case SitriBus::GateMode::TRIGGER:
                        gateHigh = triggerTimer > 0.f;
                        break;
                }

                float cvOut = params[CV_PARAMS_BASE + stepIndex].getValue();
                outputs[CV_OUTPUT].setVoltage(cvOut);
                outputs[GATE_OUTPUT].setVoltage(gateHigh ? 10.f : 0.f);

                float runBrightness = runPulse.process(args.sampleTime) ? 1.f : 0.f;
                lights[RUN_LIGHT].setBrightness(runBrightness);

                for (int i = 0; i < 8; ++i) {
                        bool active = (i < activeSteps) && (i == stepIndex);
                        lights[STEP_LIGHT_BASE + i].setBrightness(active ? 1.f : 0.f);
                        bool gateLit = active && gateHigh;
                        lights[GATE_LIGHT_BASE + i].setBrightness(gateLit ? 1.f : 0.f);
                }

                expanderMessage.magic = SitriBus::MAGIC;
                expanderMessage.version = 1;
                for (int i = 0; i < 8; ++i) {
                        int mode = clamp((int)std::round(params[MODE_PARAMS_BASE + i].getValue()), 0, 2);
                        expanderMessage.gateMode[i] = static_cast<SitriBus::GateMode>(mode);
                        expanderMessage.stepCV[i] = params[CV_PARAMS_BASE + i].getValue();
                }
                leftExpander.producerMessage = &expanderMessage;
                leftExpander.requestMessageFlip();
        }
};

struct LilithWidget : rack::app::ModuleWidget {
        LilithWidget(Lilith* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Lilith.svg")));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(14.0f, 19.0f)), module, Lilith::STEPS_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(34.5f, 19.0f)), module, Lilith::TRIGLEN_PARAM));
                addChild(createLightCentered<SmallLight<GreenLight>>(mm2px(Vec(45.5f, 19.0f)), module, Lilith::RUN_LIGHT));

                const float rowStart = 38.0f;
                const float rowSpacing = 9.5f;
                for (int i = 0; i < 8; ++i) {
                        float y = rowStart + rowSpacing * i;
                        addChild(createLightCentered<TinyLight<GreenLight>>(mm2px(Vec(7.5f, y)), module,
                                                                          Lilith::STEP_LIGHT_BASE + i));
                        addParam(createParamCentered<CKSSThree>(mm2px(Vec(18.0f, y)), module,
                                                                Lilith::MODE_PARAMS_BASE + i));
                        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(34.5f, y)), module,
                                                                          Lilith::CV_PARAMS_BASE + i));
                        addChild(createLightCentered<TinyLight<YellowLight>>(mm2px(Vec(45.5f, y)), module,
                                                                            Lilith::GATE_LIGHT_BASE + i));
                }

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(14.0f, 118.0f)), module, Lilith::CLK_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(27.0f, 118.0f)), module, Lilith::RESET_INPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(40.0f, 118.0f)), module, Lilith::CV_OUTPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(47.5f, 118.0f)), module, Lilith::GATE_OUTPUT));
        }
};

Model* modelLilith = createModel<Lilith, LilithWidget>("Lilith");

