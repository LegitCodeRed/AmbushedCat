#include "plugin.hpp"
#include "SitriBus.hpp"

#include <array>
#include <cmath>
#include <string>

using rack::math::clamp;

struct Lilith : rack::engine::Module {
	enum ParamIds {
		STEPS_PARAM,
		GATE_PARAM,
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
	int lastReceivedStep = -1;  // Track last step we received from Sitri to detect skips
	float gateTimer = 0.f;
	bool captureMode = true;  // True = capturing sequence, False = playing back snapshot
	float lastClockTime = 0.f;
	float clockPeriod = 0.5f;  // Estimated clock period in seconds

	// LED persistence for visual feedback at high speeds
	static constexpr float LED_FLASH_TIME = 0.05f;  // 50ms flash
	std::array<float, 8> stepLedTimers{};  // Timers for each step LED
	std::array<float, 8> gateLedTimers{};  // Timers for each gate LED

        // Expander message buffers for communication with Sitri
        // Double buffering: VCV Rack flips between these two buffers
        SitriBus::MasterToExpander inboundMessages[2]{};    // Receive from Sitri
        SitriBus::ExpanderToMaster outboundMessages[2]{};   // Send back (for future chain support)


	Lilith() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

		auto* stepsQuantity = configParam(STEPS_PARAM, 1.f, 8.f, 8.f, "Number of active steps", " steps");
		stepsQuantity->snapEnabled = true;

		configParam(GATE_PARAM, 0.05f, 1.f, 0.5f, "Gate Length", "%", 0.f, 100.f);

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

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		// Save capture mode state
		json_object_set_new(rootJ, "captureMode", json_boolean(captureMode));
		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {
		// Restore capture mode state
		json_t* captureModeJ = json_object_get(rootJ, "captureMode");
		if (captureModeJ)
			captureMode = json_boolean_value(captureModeJ);
	}

	void process(const ProcessArgs& args) override {
		int knobSteps = clamp((int)std::round(params[STEPS_PARAM].getValue()), 1, 8);
		float gateLength = clamp(params[GATE_PARAM].getValue(), 0.05f, 1.f);

		// Check if we're attached to Sitri on the left
		bool attachedToSitri = getLeftExpander().module &&
		                       getLeftExpander().module->model &&
		                       getLeftExpander().module->model->slug == "Sitri";
		const SitriBus::MasterToExpander* busMessage = nullptr;

		if (attachedToSitri) {
			// Get the message from Sitri via VCV Rack's message flipping system
			auto* msg = reinterpret_cast<const SitriBus::MasterToExpander*>(getLeftExpander().consumerMessage);

			if (msg && msg->magic == SitriBus::MAGIC && msg->version == 1) {
				busMessage = msg;

				// Sync gate length parameter from Sitri
				params[GATE_PARAM].setValue(msg->gateLength);

				// Check for EOC pulse - this triggers end of capture mode
				if (msg->eocPulse && captureMode) {
					captureMode = false;  // Switch to playback mode after first complete cycle
					INFO("Lilith: EOC received - switching to PLAYBACK mode");
				}

				// Reset to capture mode on reset edge or reseed (allows recapture)
				if (msg->resetEdge) {
					captureMode = true;
					INFO("Lilith: RESET received - switching to CAPTURE mode");
				}

				// Reseed behavior depends on whether Sitri is running
				if (msg->reseedEdge) {
					bool sitriRunning = msg->running != 0;
					if (sitriRunning) {
						// Sitri is running - switch back to capture mode
						captureMode = true;
						INFO("Lilith: RESEED received - switching to CAPTURE mode");
					} else {
						// Sitri is stopped - randomize the current sequence!
						INFO("Lilith: RESEED received (stopped) - RANDOMIZING sequence");
						for (int i = 0; i < 8; ++i) {
							// Random pitch: -3V to +3V (5 octave range)
							float randomPitch = (random::uniform() * 6.f) - 3.f;
							params[CV_PARAMS_BASE + i].setValue(randomPitch);

							// Random gate mode: 60% trigger, 30% mute, 10% expand
							float rnd = random::uniform();
							SitriBus::GateMode randomMode;
							if (rnd < 0.6f) {
								randomMode = SitriBus::GateMode::TRIGGER;
							} else if (rnd < 0.9f) {
								randomMode = SitriBus::GateMode::MUTE;
							} else {
								randomMode = SitriBus::GateMode::EXPAND;
							}
							params[MODE_PARAMS_BASE + i].setValue((float)randomMode);
						}
					}
				}

				// Debug log connection status
				static int connDebugCounter = 0;
				connDebugCounter++;
				if (connDebugCounter >= 48000) {
					connDebugCounter = 0;
					INFO("Lilith: Connected - captureMode=%d eoc=%d reset=%d clockEdge=%d",
					     captureMode, msg->eocPulse, msg->resetEdge, msg->clockEdge);
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

		// Determine if we should use Sitri's clock or our own
		bool usingSitriClock = false;
		if (attachedToSitri && busMessage) {
			// Only follow Sitri if it's running
			bool sitriRunning = busMessage->running != 0;
			if (sitriRunning) {
				usingSitriClock = true;
				clockEdge = busMessage->clockEdge != 0;
				resetEdge = busMessage->resetEdge != 0;

				// Only update step position on clock or reset edges for clean sync
				if (clockEdge || resetEdge) {
					// busMessage->stepIndex is 1-based, convert to 0-based
					int targetStep = (int)busMessage->stepIndex - 1;
					targetStep = clamp(targetStep, 0, activeSteps - 1);

					// Always mark as entering step on clock edge, even if step hasn't changed
					// This ensures we capture step 0 on every loop iteration
					if (targetStep != currentStep || clockEdge) {
						enteringStep = true;
						currentStep = targetStep;
					}
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

		if (clockEdge) {
			runPulse.trigger(0.02f);
			enteringStep = true;

			// Estimate clock period for gate timing
			float currentTime = args.sampleTime * args.frame;
			if (lastClockTime > 0.f) {
				float measuredPeriod = currentTime - lastClockTime;
				// Smooth the estimate
				clockPeriod = 0.9f * clockPeriod + 0.1f * measuredPeriod;
			}
			lastClockTime = currentTime;
		}

		int stepIndex = clamp(currentStep, 0, activeSteps - 1);

		// SNAPSHOT MODE: Capture sequence during first cycle only
		// When synced to Sitri and in capture mode, update step parameters on clock edges
		// After EOC (end of cycle), stop capturing and loop the snapshot
		if (usingSitriClock && busMessage && clockEdge && captureMode) {
			// At high speeds, multiple steps may advance in one frame
			// Check if we should use history buffer (multiple steps) or current data (single step)
			int stepsAdvanced = busMessage->stepsAdvanced;

			if (stepsAdvanced > 1) {
				// Multiple steps advanced - use history buffer to capture ALL of them
				for (int i = 0; i < 8; ++i) {
					const auto& histStep = busMessage->stepHistory[i];
					if (histStep.valid) {
						params[CV_PARAMS_BASE + i].setValue(histStep.pitch);
						SitriBus::GateMode gateMode;
						if (!histStep.gate) {
							gateMode = SitriBus::GateMode::MUTE;
						} else if (histStep.newNote) {
							gateMode = SitriBus::GateMode::TRIGGER;
						} else {
							gateMode = SitriBus::GateMode::EXPAND;
						}
						params[MODE_PARAMS_BASE + i].setValue((float)gateMode);
					}
				}
			} else {
				// Single step - use current data
				params[CV_PARAMS_BASE + stepIndex].setValue(busMessage->currentPitch);

				SitriBus::GateMode gateMode;
				if (!busMessage->currentGate) {
					gateMode = SitriBus::GateMode::MUTE;
				} else if (busMessage->newNote) {
					gateMode = SitriBus::GateMode::TRIGGER;
				} else {
					gateMode = SitriBus::GateMode::EXPAND;
				}
				params[MODE_PARAMS_BASE + stepIndex].setValue((float)gateMode);
			}
		}

		int modeValue = clamp((int)std::round(params[MODE_PARAMS_BASE + stepIndex].getValue()), 0, 2);

		// Gate timing: TRIGGER mode creates a pulse based on gate length parameter
		if (modeValue == SitriBus::GateMode::TRIGGER) {
			if (enteringStep) {
				// Gate duration = clock period * gate length percentage
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

		// Update LED timers - trigger flash when step is active
		if (enteringStep && stepIndex >= 0 && stepIndex < 8) {
			stepLedTimers[stepIndex] = LED_FLASH_TIME;
			if (gateHigh) {
				gateLedTimers[stepIndex] = LED_FLASH_TIME;
			}
		}

		// Decay all LED timers and set brightness
		for (int i = 0; i < 8; ++i) {
			// Decay timers
			if (stepLedTimers[i] > 0.f) {
				stepLedTimers[i] -= args.sampleTime;
			}
			if (gateLedTimers[i] > 0.f) {
				gateLedTimers[i] -= args.sampleTime;
			}

			// Show LED if timer is active OR if this is the current step
			bool active = (i < activeSteps) && (i == stepIndex);
			float stepBrightness = (stepLedTimers[i] > 0.f || active) ? 1.f : 0.f;
			lights[STEP_LIGHT_BASE + i].setBrightness(stepBrightness);

			bool gateLit = (gateLedTimers[i] > 0.f) || (active && gateHigh);
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
		addParam(createParamCentered<Trimpot>(mm2px(Vec(22.0f, 15.0f)), module, Lilith::GATE_PARAM));
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
