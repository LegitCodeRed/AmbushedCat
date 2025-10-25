#include "plugin.hpp"
#include "SitriBus.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

using rack::math::clamp;

namespace {

static constexpr int SITRI_MAX_STEPS = 8;

template <int NumSteps>
struct LilithBase : rack::engine::Module {
        static_assert(NumSteps >= 1, "LilithBase requires at least one step");

        enum ParamIds {
                STEPS_PARAM,
                GATE_PARAM,
                CV_PARAMS_BASE,
                MODE_PARAMS_BASE = CV_PARAMS_BASE + NumSteps,
                NUM_PARAMS = MODE_PARAMS_BASE + NumSteps
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
                GATE_LIGHT_BASE = STEP_LIGHT_BASE + NumSteps,
                NUM_LIGHTS = GATE_LIGHT_BASE + NumSteps
        };

        dsp::SchmittTrigger clockTrigger;
        dsp::SchmittTrigger resetTrigger;
        dsp::PulseGenerator runPulse;

        int currentStep = 0;
        int lastReceivedStep = -1;
        float gateTimer = 0.f;
        bool captureMode = true;
        float lastClockTime = 0.f;
        float clockPeriod = 0.5f;

        static constexpr float LED_FLASH_TIME = 0.05f;
        std::array<float, NumSteps> stepLedTimers{};
        std::array<float, NumSteps> gateLedTimers{};

        SitriBus::MasterToExpander inboundMessages[2]{};
        SitriBus::ExpanderToMaster outboundMessages[2]{};

        LilithBase() {
                this->config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                auto* stepsQuantity = this->configParam(STEPS_PARAM, 1.f, (float)NumSteps, (float)NumSteps,
                                                        "Number of active steps", " steps");
                stepsQuantity->snapEnabled = true;

                this->configParam(GATE_PARAM, 0.05f, 1.f, 0.5f, "Gate Length", "%", 0.f, 100.f);

                for (int i = 0; i < NumSteps; ++i) {
                        this->configParam(CV_PARAMS_BASE + i, -10.f, 10.f, 0.f,
                                          "Step " + std::to_string(i + 1) + " CV", " V");
                        auto* modeQuantity = this->configSwitch(
                            MODE_PARAMS_BASE + i, 0.f, 2.f, 0.f, "Gate mode",
                            {"Expand", "Mute", "Trigger"});
                        modeQuantity->snapEnabled = true;
                }

                this->configInput(CLK_INPUT, "Clock");
                this->configInput(RESET_INPUT, "Reset");
                this->configOutput(CV_OUTPUT, "CV");
                this->configOutput(GATE_OUTPUT, "Gate");

                this->leftExpander.producerMessage = &inboundMessages[0];
                this->leftExpander.consumerMessage = &inboundMessages[1];

                for (int i = 0; i < 2; ++i) {
                        inboundMessages[i].magic = SitriBus::MAGIC;
                        inboundMessages[i].version = 1;
                        inboundMessages[i].running = 0;
                        inboundMessages[i].stepIndex = 1;
                        inboundMessages[i].numSteps = 1;
                }

                for (int i = 0; i < 2; ++i) {
                        outboundMessages[i].magic = SitriBus::MAGIC;
                        outboundMessages[i].version = 1;
                        for (int j = 0; j < SITRI_MAX_STEPS; ++j) {
                                outboundMessages[i].gateMode[j] = SitriBus::GateMode::EXPAND;
                                outboundMessages[i].stepCV[j] = 0.f;
                        }
                }
        }

        json_t* dataToJson() override {
                json_t* rootJ = json_object();
                json_object_set_new(rootJ, "captureMode", json_boolean(captureMode));
                return rootJ;
        }

        void dataFromJson(json_t* rootJ) override {
                json_t* captureModeJ = json_object_get(rootJ, "captureMode");
                if (captureModeJ)
                        captureMode = json_boolean_value(captureModeJ);
        }

        void process(const ProcessArgs& args) override {
                int knobSteps = clamp((int)std::round(this->params[STEPS_PARAM].getValue()), 1, NumSteps);
                float gateLength = clamp(this->params[GATE_PARAM].getValue(), 0.05f, 1.f);

                bool attachedToSitri = this->getLeftExpander().module &&
                                       this->getLeftExpander().module->model &&
                                       this->getLeftExpander().module->model->slug == "Sitri";
                const SitriBus::MasterToExpander* busMessage = nullptr;

                if (attachedToSitri) {
                        auto* msg = reinterpret_cast<const SitriBus::MasterToExpander*>(
                            this->getLeftExpander().consumerMessage);

                        if (msg && msg->magic == SitriBus::MAGIC && msg->version == 1) {
                                busMessage = msg;

                                this->params[GATE_PARAM].setValue(msg->gateLength);

                                if (msg->eocPulse && captureMode) {
                                        captureMode = false;
                                        INFO("Lilith: EOC received - switching to PLAYBACK mode");
                                }

                                if (msg->resetEdge) {
                                        captureMode = true;
                                        INFO("Lilith: RESET received - switching to CAPTURE mode");
                                }

                                if (msg->reseedEdge) {
                                        bool sitriRunning = msg->running != 0;
                                        if (sitriRunning) {
                                                captureMode = true;
                                                INFO("Lilith: RESEED received - switching to CAPTURE mode");
                                        } else {
                                                INFO("Lilith: RESEED received (stopped) - RANDOMIZING sequence");
                                                for (int i = 0; i < NumSteps; ++i) {
                                                        float randomPitch = (random::uniform() * 6.f) - 3.f;
                                                        this->params[CV_PARAMS_BASE + i].setValue(randomPitch);

                                                        float rnd = random::uniform();
                                                        SitriBus::GateMode randomMode;
                                                        if (rnd < 0.6f) {
                                                                randomMode = SitriBus::GateMode::TRIGGER;
                                                        } else if (rnd < 0.9f) {
                                                                randomMode = SitriBus::GateMode::MUTE;
                                                        } else {
                                                                randomMode = SitriBus::GateMode::EXPAND;
                                                        }
                                                        this->params[MODE_PARAMS_BASE + i].setValue((float)randomMode);
                                                }
                                        }
                                }

                                static int connDebugCounter = 0;
                                connDebugCounter++;
                                if (connDebugCounter >= 48000) {
                                        connDebugCounter = 0;
                                        INFO("Lilith: Connected - captureMode=%d eoc=%d reset=%d clockEdge=%d",
                                             captureMode, msg->eocPulse, msg->resetEdge, msg->clockEdge);
                                }
                        }
                }

                bool jackReset = this->resetTrigger.process(this->inputs[RESET_INPUT].getVoltage());
                bool clockEdge = false;
                bool enteringStep = false;
                bool resetEdge = false;

                int activeSteps = knobSteps;
                if (attachedToSitri && busMessage) {
                        activeSteps = clamp((int)busMessage->numSteps, 1, NumSteps);
                }

                bool usingSitriClock = false;
                if (attachedToSitri && busMessage) {
                        bool sitriRunning = busMessage->running != 0;
                        if (sitriRunning) {
                                usingSitriClock = true;
                                clockEdge = busMessage->clockEdge != 0;
                                resetEdge = busMessage->resetEdge != 0;

                                if (clockEdge || resetEdge) {
                                        int targetStep = (int)busMessage->stepIndex - 1;
                                        targetStep = clamp(targetStep, 0, activeSteps - 1);

                                        if (targetStep != currentStep || clockEdge) {
                                                enteringStep = true;
                                                currentStep = targetStep;
                                        }
                                }
                        }
                }

                if (!usingSitriClock) {
                        bool clkTrig = this->clockTrigger.process(this->inputs[CLK_INPUT].getVoltage());
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

                if (clockEdge) {
                        this->runPulse.trigger(0.02f);
                        enteringStep = true;

                        float currentTime = args.sampleTime * args.frame;
                        if (lastClockTime > 0.f) {
                                float measuredPeriod = currentTime - lastClockTime;
                                clockPeriod = 0.9f * clockPeriod + 0.1f * measuredPeriod;
                        }
                        lastClockTime = currentTime;
                }

                int stepIndex = clamp(currentStep, 0, activeSteps - 1);

                if (usingSitriClock && busMessage && clockEdge && captureMode) {
                        int stepsAdvanced = busMessage->stepsAdvanced;

                        if (stepsAdvanced > 1) {
                                for (int i = 0; i < std::min(NumSteps, SITRI_MAX_STEPS); ++i) {
                                        const auto& histStep = busMessage->stepHistory[i];
                                        if (histStep.valid) {
                                                this->params[CV_PARAMS_BASE + i].setValue(histStep.pitch);
                                                SitriBus::GateMode gateMode;
                                                if (!histStep.gate) {
                                                        gateMode = SitriBus::GateMode::MUTE;
                                                } else if (histStep.newNote) {
                                                        gateMode = SitriBus::GateMode::TRIGGER;
                                                } else {
                                                        gateMode = SitriBus::GateMode::EXPAND;
                                                }
                                                this->params[MODE_PARAMS_BASE + i].setValue((float)gateMode);
                                        }
                                }
                        } else {
                                this->params[CV_PARAMS_BASE + stepIndex].setValue(busMessage->currentPitch);

                                SitriBus::GateMode gateMode;
                                if (!busMessage->currentGate) {
                                        gateMode = SitriBus::GateMode::MUTE;
                                } else if (busMessage->newNote) {
                                        gateMode = SitriBus::GateMode::TRIGGER;
                                } else {
                                        gateMode = SitriBus::GateMode::EXPAND;
                                }
                                this->params[MODE_PARAMS_BASE + stepIndex].setValue((float)gateMode);
                        }
                }

                int modeValue = clamp((int)std::round(
                                          this->params[MODE_PARAMS_BASE + stepIndex].getValue()),
                                      0, 2);

                if (modeValue == SitriBus::GateMode::TRIGGER) {
                        if (enteringStep) {
                                gateTimer = clockPeriod * gateLength;
                        }
                        if (gateTimer > 0.f) {
                                gateTimer -= args.sampleTime;
                                if (gateTimer < 0.f)
                                        gateTimer = 0.f;
                        }
                } else {
                        gateTimer = 0.f;
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
                        gateHigh = gateTimer > 0.f;
                        break;
                }

                float cvOut = this->params[CV_PARAMS_BASE + stepIndex].getValue();
                this->outputs[CV_OUTPUT].setVoltage(cvOut);
                this->outputs[GATE_OUTPUT].setVoltage(gateHigh ? 10.f : 0.f);

                float runBrightness = 0.f;
                if (usingSitriClock) {
                        runBrightness = 1.f;
                } else {
                        runBrightness = this->runPulse.process(args.sampleTime) ? 1.f : 0.f;
                }
                this->lights[RUN_LIGHT].setBrightness(runBrightness);

                if (enteringStep && stepIndex >= 0 && stepIndex < NumSteps) {
                        stepLedTimers[stepIndex] = LED_FLASH_TIME;
                        if (gateHigh) {
                                gateLedTimers[stepIndex] = LED_FLASH_TIME;
                        }
                }

                for (int i = 0; i < NumSteps; ++i) {
                        if (stepLedTimers[i] > 0.f) {
                                stepLedTimers[i] -= args.sampleTime;
                        }
                        if (gateLedTimers[i] > 0.f) {
                                gateLedTimers[i] -= args.sampleTime;
                        }

                        bool active = (i < activeSteps) && (i == stepIndex);
                        float stepBrightness = (stepLedTimers[i] > 0.f || active) ? 1.f : 0.f;
                        this->lights[STEP_LIGHT_BASE + i].setBrightness(stepBrightness);

                        bool gateLit = (gateLedTimers[i] > 0.f) || (active && gateHigh);
                        this->lights[GATE_LIGHT_BASE + i].setBrightness(gateLit ? 1.f : 0.f);
                }
        }
};

template <int NumSteps, typename ModuleType>
struct LilithWidgetBase : rack::app::ModuleWidget {
        LilithWidgetBase(ModuleType* module, const std::string& panelAsset) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, panelAsset)));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH,
                                                      RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(8.0f, 15.0f)), module,
                                                                  ModuleType::STEPS_PARAM));
                addParam(createParamCentered<Trimpot>(mm2px(Vec(22.0f, 15.0f)), module,
                                                      ModuleType::GATE_PARAM));
                addChild(createLightCentered<TinyLight<GreenLight>>(mm2px(Vec(32.0f, 15.0f)), module,
                                                                    ModuleType::RUN_LIGHT));

                const float rowStart = 26.0f;
                const float rowEnd = 104.0f;
                float rowSpacing = (NumSteps > 1) ? (rowEnd - rowStart) / (NumSteps - 1) : 0.f;
                for (int i = 0; i < NumSteps; ++i) {
                        float y = rowStart + rowSpacing * i;
                        addChild(createLightCentered<TinyLight<GreenLight>>(mm2px(Vec(4.0f, y)), module,
                                                                          ModuleType::STEP_LIGHT_BASE + i));
                        addParam(createParamCentered<CKSSThree>(mm2px(Vec(11.0f, y)), module,
                                                                ModuleType::MODE_PARAMS_BASE + i));
                        addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.0f, y)), module,
                                                                          ModuleType::CV_PARAMS_BASE + i));
                        addChild(createLightCentered<TinyLight<YellowLight>>(mm2px(Vec(30.0f, y)), module,
                                                                            ModuleType::GATE_LIGHT_BASE + i));
                }

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.0f, 116.0f)), module,
                                                         ModuleType::CLK_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(14.0f, 116.0f)), module,
                                                         ModuleType::RESET_INPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(22.0f, 116.0f)), module,
                                                           ModuleType::CV_OUTPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(30.0f, 116.0f)), module,
                                                           ModuleType::GATE_OUTPUT));
        }
};

} // namespace

struct Lilith : LilithBase<8> {
        Lilith() : LilithBase() {}
};

struct LilithAdvance : LilithBase<16> {
        LilithAdvance() : LilithBase() {}
};

struct LilithWidget : LilithWidgetBase<8, Lilith> {
        LilithWidget(Lilith* module)
            : LilithWidgetBase<8, Lilith>(module, "res/Lilith.svg") {}
};

struct LilithAdvanceWidget : LilithWidgetBase<16, LilithAdvance> {
        LilithAdvanceWidget(LilithAdvance* module)
            : LilithWidgetBase<16, LilithAdvance>(module, "res/LilithAdvance.svg") {}
};

Model* modelLilith = createModel<Lilith, LilithWidget>("Lilith");
Model* modelLilithAdvance = createModel<LilithAdvance, LilithAdvanceWidget>("LilithAdvance");
