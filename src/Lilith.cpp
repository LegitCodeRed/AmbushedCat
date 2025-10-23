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

        // Expander message buffers for communication with Sitri
        // Double buffering: VCV Rack flips between these two buffers
        SitriBus::MasterToExpander inboundMessages[2]{};    // Receive from Sitri
        SitriBus::ExpanderToMaster outboundMessages[2]{};   // Send back (for future chain support)


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

                // Register double buffers for VCV Rack expander messaging
                // VCV Rack will flip between buffer [0] and [1] when requestMessageFlip() is called
                leftExpander.producerMessage = &inboundMessages[0];
                leftExpander.consumerMessage = &inboundMessages[1];

                // Initialize both inbound buffers
                for (int i = 0; i < 2; ++i) {
                        inboundMessages[i].magic = SitriBus::MAGIC;
                        inboundMessages[i].version = 1;
                        inboundMessages[i].running = 0;
                        inboundMessages[i].stepIndex = 1;
                        inboundMessages[i].numSteps = 1;
                }

                // For future chain support (Lilith sending data back)
                for (int i = 0; i < 2; ++i) {
                        outboundMessages[i].magic = SitriBus::MAGIC;
                        outboundMessages[i].version = 1;
                        for (int j = 0; j < 8; ++j) {
                                outboundMessages[i].gateMode[j] = SitriBus::GateMode::EXPAND;
                                outboundMessages[i].stepCV[j] = 0.f;
                        }
                }
	}

	void process(const ProcessArgs& args) override {
		int knobSteps = clamp((int)std::round(params[STEPS_PARAM].getValue()), 1, 8);
		float trigLenSec = clamp(params[TRIGLEN_PARAM].getValue(), 0.001f, 0.1f);

                // Debug: Always log status once per second to confirm Lilith is running
                static int globalDebugCounter = 0;
                globalDebugCounter++;
		if (globalDebugCounter >= 48000) {
			globalDebugCounter = 0;
			bool hasLeftModule = getLeftExpander().module != nullptr;
			bool hasModel = hasLeftModule && getLeftExpander().module->model != nullptr;
			std::string slug = hasModel ? getLeftExpander().module->model->slug : "none";
			INFO("Lilith: Running - leftModule=%d, hasModel=%d, slug='%s'",
			     hasLeftModule, hasModel, slug.c_str());
		}

		// Check if we're attached to Sitri on the left (use string comparison like TuringGateExpander)
		bool attachedToSitri = getLeftExpander().module &&
		                       getLeftExpander().module->model &&
		                       getLeftExpander().module->model->slug == "Sitri";
		const SitriBus::MasterToExpander* busMessage = nullptr;

		if (attachedToSitri) {
			// Get the message from Sitri. VCV Rack automatically copies
			// Sitri's rightExpander.producerMessage to our leftExpander.consumerMessage.
			auto* msg = reinterpret_cast<const SitriBus::MasterToExpander*>(getLeftExpander().consumerMessage);

			// Debug: log message address and status once per second
			if (globalDebugCounter == 0) {
				INFO("Lilith: Consumer=%p Producer=%p Module=%p",
				     (void*)getLeftExpander().consumerMessage,
				     (void*)getLeftExpander().producerMessage,
				     (void*)getLeftExpander().module);

				if (!msg) {
					INFO("Lilith: consumerMessage is NULL!");
				} else {
					INFO("Lilith: Inbound magic=0x%08X running=%d stepIndex=%d",
					     msg->magic, msg->running, msg->stepIndex);
				}
			}

			if (msg && msg->magic == SitriBus::MAGIC && msg->version == 1) {
				busMessage = msg;
			}
		}

		bool jackReset = resetTrigger.process(inputs[RESET_INPUT].getVoltage());
		bool clockEdge = false;
		bool enteringStep = false;
		bool resetEdge = false;

		// Lilith is hardcoded to max 8 steps
		// If attached to Sitri, follow Sitri's step count (but cap at 8)
		// If standalone, use the STEPS knob
		int activeSteps = knobSteps;
		if (attachedToSitri && busMessage) {
			activeSteps = clamp((int)busMessage->numSteps, 1, 8);
		}

		// Determine if we should use Sitri's clock or our own
		bool usingSitriClock = false;
		if (attachedToSitri && busMessage) {
			// Only follow Sitri if it's running
			bool sitriRunning = busMessage->running != 0;
			if (sitriRunning) {
				usingSitriClock = true;
				// busMessage->stepIndex is 1-based, convert to 0-based
				int targetStep = (int)busMessage->stepIndex - 1;
				targetStep = clamp(targetStep, 0, activeSteps - 1);
				if (targetStep != currentStep)
					enteringStep = true;
				clockEdge = busMessage->clockEdge != 0;
				resetEdge = busMessage->resetEdge != 0;
				currentStep = targetStep;

				// Debug log (once per second)
				static int lilithDebugCounter = 0;
				lilithDebugCounter++;
				if (lilithDebugCounter >= 48000) {
					lilithDebugCounter = 0;
					INFO("Lilith: Synced - currentStep=%d, targetStep=%d, enteringStep=%d, clockEdge=%d",
					     currentStep, targetStep, enteringStep, clockEdge);
				}
			}
		}

		// Fall back to own clock inputs if not using Sitri's clock
		if (!usingSitriClock) {
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

		// RUN light: Shows connection and operation status
		// - Solid green: Connected to Sitri and receiving valid clock
		// - Blinking: Standalone mode receiving clock
		// - Off: Not receiving clock (check connections!)
		float runBrightness = 0.f;
		if (usingSitriClock) {
			// Successfully using Sitri's clock - solid green
			runBrightness = 1.f;
		} else {
			// Standalone or fallback mode - blink on clock
			runBrightness = runPulse.process(args.sampleTime) ? 1.f : 0.f;
		}
		lights[RUN_LIGHT].setBrightness(runBrightness);

		for (int i = 0; i < 8; ++i) {
			bool active = (i < activeSteps) && (i == stepIndex);
			lights[STEP_LIGHT_BASE + i].setBrightness(active ? 1.f : 0.f);
			bool gateLit = active && gateHigh;
			lights[GATE_LIGHT_BASE + i].setBrightness(gateLit ? 1.f : 0.f);
		}

                // TODO: Future expander chain support - write to outboundMessages and flip
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

		// Compact 8HP layout (40.64mm width)
		// Top controls
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(8.0f, 15.0f)), module, Lilith::STEPS_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(22.0f, 15.0f)), module, Lilith::TRIGLEN_PARAM));
		addChild(createLightCentered<TinyLight<GreenLight>>(mm2px(Vec(32.0f, 15.0f)), module, Lilith::RUN_LIGHT));

		// 8 step rows - very compact vertical spacing
		const float rowStart = 26.0f;
		const float rowSpacing = 10.5f;
		for (int i = 0; i < 8; ++i) {
			float y = rowStart + rowSpacing * i;
			// Step LED (leftmost)
			addChild(createLightCentered<TinyLight<GreenLight>>(mm2px(Vec(4.0f, y)), module,
			                                                  Lilith::STEP_LIGHT_BASE + i));
			// Mode switch (3-position)
			addParam(createParamCentered<CKSSThree>(mm2px(Vec(11.0f, y)), module,
			                                        Lilith::MODE_PARAMS_BASE + i));
			// CV knob
			addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.0f, y)), module,
			                                                  Lilith::CV_PARAMS_BASE + i));
			// Gate LED (rightmost)
			addChild(createLightCentered<TinyLight<YellowLight>>(mm2px(Vec(30.0f, y)), module,
			                                                    Lilith::GATE_LIGHT_BASE + i));
		}

		// Bottom I/O - adjusted for 8HP
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.0f, 116.0f)), module, Lilith::CLK_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(14.0f, 116.0f)), module, Lilith::RESET_INPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(22.0f, 116.0f)), module, Lilith::CV_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(30.0f, 116.0f)), module, Lilith::GATE_OUTPUT));
	}
};

Model* modelLilith = createModel<Lilith, LilithWidget>("Lilith");
