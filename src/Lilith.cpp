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

	// Message buffers for expander communication
	SitriBus::MasterToExpander consumerMessage{};  // Receive from Sitri
	SitriBus::ExpanderToMaster producerMessage{};  // Send to Sitri (for future chain support)

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

		// Set up expander message buffers
		leftExpander.producerMessage = &producerMessage;
		leftExpander.consumerMessage = &consumerMessage;

		// Initialize producer message (for potential future use)
		producerMessage.magic = SitriBus::MAGIC;
		producerMessage.version = 1;
		for (int i = 0; i < 8; ++i) {
			producerMessage.gateMode[i] = SitriBus::GateMode::EXPAND;
			producerMessage.stepCV[i] = 0.f;
		}
	}

	void process(const ProcessArgs& args) override {
		int knobSteps = clamp((int)std::round(params[STEPS_PARAM].getValue()), 1, 8);
		float trigLenSec = clamp(params[TRIGLEN_PARAM].getValue(), 0.001f, 0.1f);

		// Check if we're attached to Sitri on the left
		Module* leftModule = leftExpander.module;
		bool attachedToSitri = leftModule && leftModule->model == modelSitri;
		const SitriBus::MasterToExpander* busMessage = nullptr;

		if (attachedToSitri) {
			// Get the message from Sitri
			if (leftExpander.consumerMessage) {
				auto* msg = reinterpret_cast<const SitriBus::MasterToExpander*>(leftExpander.consumerMessage);
				if (msg && msg->magic == SitriBus::MAGIC && msg->version == 1) {
					busMessage = msg;
				}
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

		if (attachedToSitri) {
			if (busMessage) {
				// Only follow Sitri if it's running
				bool sitriRunning = busMessage->running != 0;
				if (sitriRunning) {
					// busMessage->stepIndex is 1-based, convert to 0-based
					int targetStep = (int)busMessage->stepIndex - 1;
					targetStep = clamp(targetStep, 0, activeSteps - 1);
					if (targetStep != currentStep)
						enteringStep = true;
					clockEdge = busMessage->clockEdge != 0;
					resetEdge = busMessage->resetEdge != 0;
					currentStep = targetStep;
				}
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

		// RUN light: blinks on clock when attached to Sitri, solid when standalone and receiving clock
		float runBrightness = 0.f;
		if (attachedToSitri && busMessage) {
			// Connected to Sitri: show if Sitri is running
			runBrightness = (busMessage->running != 0) ? 1.f : 0.f;
		} else {
			// Standalone mode: blink on clock
			runBrightness = runPulse.process(args.sampleTime) ? 1.f : 0.f;
		}
		lights[RUN_LIGHT].setBrightness(runBrightness);

		for (int i = 0; i < 8; ++i) {
			bool active = (i < activeSteps) && (i == stepIndex);
			lights[STEP_LIGHT_BASE + i].setBrightness(active ? 1.f : 0.f);
			bool gateLit = active && gateHigh;
			lights[GATE_LIGHT_BASE + i].setBrightness(gateLit ? 1.f : 0.f);
		}

		// Update producer message (for future expander chain support)
		producerMessage.magic = SitriBus::MAGIC;
		producerMessage.version = 1;
		for (int i = 0; i < 8; ++i) {
			int mode = clamp((int)std::round(params[MODE_PARAMS_BASE + i].getValue()), 0, 2);
			producerMessage.gateMode[i] = static_cast<SitriBus::GateMode>(mode);
			producerMessage.stepCV[i] = params[CV_PARAMS_BASE + i].getValue();
		}
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
