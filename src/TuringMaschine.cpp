#include "plugin.hpp"
#include <bitset>

struct BitShiftRegister {
	BitShiftRegister() {
		for (int i = 0; i < 16; ++i) {
			bool bit = random::u32() % 2 == 0;
			bits.set(i, bit);
			seedBits.set(i, bit);
		}
	}



	std::bitset<16> bits;
	std::bitset<16> seedBits;

	int rotateCounters[8] = {};
	const int rotateEvery[8] = {16, 12, 9, 7, 5, 3, 6, 4};
	static const int NUM_OUTPUTS = 8;

	void reset() {
		bits.reset();
	}

	void resetToSeed() {
		bits = seedBits;
	}

	void shift(bool allowMutation, float changeProbability, float bias, int mode) {
		if (mode == 1) {
			// 🔁 Polyrhythmic mode: each output rotates at its own interval
			bool doRotate = false;

			for (int i = 0; i < NUM_OUTPUTS; ++i) {
				rotateCounters[i]++;
				if (rotateCounters[i] >= rotateEvery[i]) {
					rotateCounters[i] = 0;
					doRotate = true;
				}
			}

			if (!doRotate)
				return;  // No rotation needed this tick

			// Rotate shared register once safely
			bool newBit = bits[15];
			bits <<= 1;
			bits.set(0, newBit);

			return;  // Skip mutation in polyrhythmic mode
		}

		// 🎲 Standard Turing machine mode (normal mode)

		bool newBit;
		if (allowMutation && random::uniform() < changeProbability) {
			newBit = (random::uniform() < bias);  // biased mutation
		} else {
			newBit = bits[15];  // preserve MSB
		}

		bits <<= 1;
		bits.set(0, newBit);

		// 👻 Deadlock prevention
		if (allowMutation && bits.none()) {
			bits.set(random::u32() % 16, true);  // Inject a single live bit
		}
	}
	// Converts top N bits into an integer (default: top 8 bits)
	int getTopBitsAsInt(int bitCount) const {
		int value = 0;
		for (int i = 0; i < bitCount; ++i) {
			value <<= 1;
			value |= bits[15 - i];
		}
		return value;
	}
};

struct TuringMaschine : Module {
	enum ParamId {
		CHANGE_PARAM,
		LENGTH_PARAM,
		BIAS_PARAM,
		WRITE_PARAM,
		SEED_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		CLOCK_INPUT,
		RESET_INPUT,
		CHANGE_CV_INPUT,
		LENGTH_CV_INPUT,
		BIAS_CV_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		SEQUENCE_OUTPUT,
		NOISE_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		BLINK_LIGHT,
		BIT_LIGHTS,  // this starts the 16-bit range
		NUM_LIGHTS = BIT_LIGHTS + 16,
		LIGHTS_LEN
	};

	float pitchScale = 5.0f; 
	float blinkTimer = 0.f;

	BitShiftRegister shiftReg;
	dsp::SchmittTrigger clockTrigger;
	dsp::SchmittTrigger resetTrigger;
	dsp::SchmittTrigger seedTrigger;

        int mode = 0; // 0: Normal, 1: Techno
        int pitchMode = 0;
        int writeMode = 0; // 0: Standard, 1: Evolving

	TuringMaschine() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CHANGE_PARAM, 0.f, 1.f, 0.5f, "Change");
		configParam(LENGTH_PARAM, 1.f, 16.f, 8.f, "Length");

		configParam(WRITE_PARAM, 0.f, 1.f, 0.f, "Write");
		configParam(SEED_PARAM, 0.f, 1.f, 0.f, "Generate Seed");

		configInput(CHANGE_CV_INPUT, "Change CV");
		configInput(LENGTH_CV_INPUT, "Length CV");

		paramQuantities[LENGTH_PARAM]->snapEnabled = true;
		configParam(BIAS_PARAM, 0.f, 1.f, 0.5f, "Bias");

		configInput(CLOCK_INPUT, "Clock");
		configInput(RESET_INPUT, "Reset");
		configOutput(SEQUENCE_OUTPUT, "Sequence");
		configOutput(NOISE_OUTPUT, "Noise");
	}

	float getPitchScale() {
		switch (pitchMode) {	
			case 0: return 5.0f;   // 5V range
			case 1: return 3.0f;   // 3V range
			case 2: return 1.0f;   // 1V range
			default: return 5.0f;  // Default to 5V
		}	
	}

	void process(const ProcessArgs& args) override {
		if (resetTrigger.process(inputs[RESET_INPUT].getVoltage())) {
			shiftReg.resetToSeed(); // start from beginning of saved pattern
		}

		if (seedTrigger.process(params[SEED_PARAM].getValue())) {
			// Seed the shift register
			for (int i = 0; i < 16; ++i) {
				bool bit = random::u32() % 2 == 0;
				shiftReg.bits.set(i, bit);
				shiftReg.seedBits.set(i, bit);
			}
		}

		// Detect clock rising edge
		bool allowMutation = params[WRITE_PARAM].getValue() > 0.5f;
		float change = params[CHANGE_PARAM].getValue();
		
		// Handle clock
		bool clockPulse = false;
		if (inputs[CLOCK_INPUT].isConnected()) {
			clockPulse = clockTrigger.process(inputs[CLOCK_INPUT].getVoltage());
		} else {
			// internal clock here if you want
		}

		if (clockPulse) {
			blinkTimer = 0.05f;
			float knobValue = params[BIAS_PARAM].getValue();       // 0.0 to 1.0
			float cvInput   = inputs[BIAS_CV_INPUT].getVoltage();  // -5V to +5V typical
			float cvNormalized = clamp(cvInput / 10.0f + 0.5f, 0.0f, 1.0f);  // Normalize to 0.0–1.0

			float bias = clamp(knobValue + cvNormalized - 0.5f, 0.0f, 1.0f);

			float evolveChange = change;
			bool mutate = allowMutation;
			if (writeMode == 1) {
					// Gradually evolve the pattern
					evolveChange *= 0.1f;
					mutate = true;
			}
			shiftReg.shift(mutate, evolveChange, bias, mode);
			}

		if (inputs[CHANGE_CV_INPUT].isConnected()) {
			change += inputs[CHANGE_CV_INPUT].getVoltage() / 10.f; // Normalize -5V to +5V → -0.5 to +0.5
		}
		change = clamp(change, 0.f, 1.f);


		float lengthParam = params[LENGTH_PARAM].getValue(); // 1 to 16 from knob
		float lengthCV = inputs[LENGTH_CV_INPUT].getVoltage(); // -5V to +5V typically
		float modulatedLength = lengthParam + lengthCV * 1.6f;
		modulatedLength = clamp(modulatedLength, 1.f, 16.f);


		int bitCount = static_cast<int>(modulatedLength);
		int value = shiftReg.getTopBitsAsInt(bitCount);
		int maxValue = (1 << bitCount) - 1;
		
		float voltage = (value / (float)maxValue) * getPitchScale(); 

		outputs[SEQUENCE_OUTPUT].setVoltage(voltage);

		bool noiseBit = random::u32() % 2 == 0;
		outputs[NOISE_OUTPUT].setVoltage(noiseBit ? 10.f : 0.f);

		
                Module* exp = this;
                while (exp->rightExpander.module) {
                        exp = exp->rightExpander.module;

                        if (exp->model && exp->model->slug == "TuringVoltsExpander") {
                                if (!exp->leftExpander.producerMessage) {
                                        exp->leftExpander.producerMessage = malloc(sizeof(TuringVoltsExpanderMessage));
                                }

                                if (exp->leftExpander.producerMessage) {
                                        auto* msg = static_cast<TuringVoltsExpanderMessage*>(exp->leftExpander.producerMessage);
                                        msg->bits = shiftReg.getTopBitsAsInt(5);
                                        exp->leftExpander.requestMessageFlip();
                                }
                                // continue searching to support multiple Volts expanders
                        }
                }

		Module* rightModule = getRightExpander().module;
		if (rightModule && rightModule->getModel()->slug == "TuringGateExpander") {
			float* value = (float*)rightModule->getLeftExpander().producerMessage;
			if (value) {
				*value = (float)shiftReg.getTopBitsAsInt(8); // 8-bit value as float
				rightModule->getLeftExpander().requestMessageFlip();
			}
		}


		if (blinkTimer > 0.f) {
			blinkTimer -= args.sampleTime;
			lights[BLINK_LIGHT].setBrightness(1.f);
		} else {
			lights[BLINK_LIGHT].setBrightness(0.f);
		}

		// Update lights based on the current state of the shift register
		for (int i = 0; i < 16; ++i) {
			if (i < bitCount) {
				// Active bits (top N)
				lights[BIT_LIGHTS + i].setBrightness(shiftReg.bits[15 - i] ? 1.f : 0.f);
			} else {
				// Bits not used — off
				lights[BIT_LIGHTS + i].setBrightness(0.f);
			}
		}
	}

	json_t* dataToJson() override {
		json_t* root = json_object();

		// Save shiftReg.bits
		uint32_t bitsLow = (uint32_t)(shiftReg.bits.to_ullong() & 0xFFFFFFFF);
		uint32_t bitsHigh = (uint32_t)(shiftReg.bits.to_ullong() >> 32);
		json_object_set_new(root, "bitsLow", json_integer(bitsLow));
		json_object_set_new(root, "bitsHigh", json_integer(bitsHigh));

		// Save seedBits the same way
		uint32_t seedLow = (uint32_t)(shiftReg.seedBits.to_ullong() & 0xFFFFFFFF);
		uint32_t seedHigh = (uint32_t)(shiftReg.seedBits.to_ullong() >> 32);
		json_object_set_new(root, "seedLow", json_integer(seedLow));
		json_object_set_new(root, "seedHigh", json_integer(seedHigh));

		// Save pitch scale
                json_object_set_new(root, "pitchMode", json_integer(pitchMode));
                json_object_set_new(root, "mode", json_integer(mode));
                json_object_set_new(root, "writeMode", json_integer(writeMode));

		return root;
	}

	void dataFromJson(json_t* root) override {
		// Restore shiftReg.bits
		uint32_t bitsLow = 0;
		uint32_t bitsHigh = 0;
		json_t* jBitsLow = json_object_get(root, "bitsLow");
		json_t* jBitsHigh = json_object_get(root, "bitsHigh");
		if (jBitsLow && jBitsHigh) {
			bitsLow = json_integer_value(jBitsLow);
			bitsHigh = json_integer_value(jBitsHigh);
			uint64_t fullBits = ((uint64_t)bitsHigh << 32) | bitsLow;
			shiftReg.bits = std::bitset<16>(fullBits);
		}

		// Restore shiftReg.seedBits
		uint32_t seedLow = 0;
		uint32_t seedHigh = 0;
		json_t* jSeedLow = json_object_get(root, "seedLow");
		json_t* jSeedHigh = json_object_get(root, "seedHigh");
		if (jSeedLow && jSeedHigh) {
			seedLow = json_integer_value(jSeedLow);
			seedHigh = json_integer_value(jSeedHigh);
			uint64_t fullSeed = ((uint64_t)seedHigh << 32) | seedLow;
			shiftReg.seedBits = std::bitset<16>(fullSeed);
		}

		// Restore pitch scale
		json_t* jsonPitchMode = json_object_get(root, "pitchMode");
		if (jsonPitchMode) {
			pitchMode = json_integer_value(jsonPitchMode);
		}

                json_t* modeJ = json_object_get(root, "mode");
                if (modeJ) {
                        mode = json_integer_value(modeJ);
                }

                json_t* writeJ = json_object_get(root, "writeMode");
                if (writeJ) {
                        writeMode = json_integer_value(writeJ);
                }
        }
};

struct TuringMaschineWidget : ModuleWidget {
	TuringMaschineWidget(TuringMaschine* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/TuringMaschine.svg")));

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 50.063)), module, TuringMaschine::LENGTH_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 60.063)), module, TuringMaschine::CHANGE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 70.063)), module, TuringMaschine::BIAS_PARAM));

		addParam(createParamCentered<TL1105>(mm2px(Vec(15.24, 30.81)), module, TuringMaschine::SEED_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(15.24, 40.81)), module, TuringMaschine::WRITE_PARAM));

		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 90.478)), module, TuringMaschine::CLOCK_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 100.478)), module, TuringMaschine::RESET_INPUT));

		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(25.0, 46.0)), module, TuringMaschine::CHANGE_CV_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(25.0, 61.0)), module, TuringMaschine::LENGTH_CV_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(25.0, 75.0)), module, TuringMaschine::BIAS_CV_INPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 108.713)), module, TuringMaschine::SEQUENCE_OUTPUT));
		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(25.24, 108.713)), module, TuringMaschine::NOISE_OUTPUT));

		for (int i = 0; i < 16; ++i) {
			float x = mm2px(Vec(4.0f, 20.0f)).x;  // adjust X as needed
			float y = mm2px(Vec(0.0f, 10.0f + i * 4.0f)).y;  // vertical stack
			addChild(createLight<SmallLight<GreenLight>>(Vec(x, y), module, TuringMaschine::BIT_LIGHTS + i));
		}
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(15.24, 25.81)), module, TuringMaschine::BLINK_LIGHT));
	}

	void appendContextMenu(Menu* menu) override {
		TuringMaschine* module = getModule<TuringMaschine>();
		menu->addChild(createIndexPtrSubmenuItem("Mode",
			{"Normal", "Poly Ryhtmic"},
			&module->mode
		));

                menu->addChild(createIndexPtrSubmenuItem("Pitch Output Range",
                        {"5V", "3V", "1V"},
                        &module->pitchMode
                ));

                menu->addChild(createIndexPtrSubmenuItem("Write Mode",
                        {"Standard", "Evolving"},
                        &module->writeMode
                ));
        }
};

Model* modelTuringMaschine = createModel<TuringMaschine, TuringMaschineWidget>("TuringMaschine");