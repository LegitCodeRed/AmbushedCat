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

	void reset() {
		bits.reset();
	}

	void resetToSeed() {
		bits = seedBits;
	}

	void shift(bool allowMutation, float changeProbability) {
		bool newBit;

		if (allowMutation && random::uniform() < changeProbability) {
			newBit = (random::u32() % 2 == 0);  // random bit
		} else {
			newBit = bits[15];  // copy MSB
		}

		bits <<= 1;
		bits.set(0, newBit);

		// Optional: inject entropy only if mutation is on
		if (allowMutation && bits.none()) {
			bits.set(0, 1);
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
		SCALE_PARAM,
		WRITE_PARAM,
		SEED_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		CLOCK_INPUT,
		RESET_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		SEQUENCE_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		BLINK_LIGHT,
		BIT_LIGHTS,  // this starts the 16-bit range
		NUM_LIGHTS = BIT_LIGHTS + 16,
		LIGHTS_LEN
	};

	BitShiftRegister shiftReg;
	dsp::SchmittTrigger clockTrigger;
	dsp::SchmittTrigger resetTrigger;
	dsp::SchmittTrigger seedTrigger;

	TuringMaschine() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CHANGE_PARAM, 0.f, 1.f, 0.f, "Change");
		configParam(LENGTH_PARAM, 1.f, 16.f, 16.f, "Length");
		paramQuantities[LENGTH_PARAM]->snapEnabled = true;
		configParam(SCALE_PARAM, 0.f, 1.f, 0.f, "Scale");

		configInput(CLOCK_INPUT, "Clock");
		configInput(RESET_INPUT, "Reset");
		configOutput(SEQUENCE_OUTPUT, "Sequence");
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
			shiftReg.shift(allowMutation, change);
		}

		int bitCount = static_cast<int>(params[LENGTH_PARAM].getValue());
		int value = shiftReg.getTopBitsAsInt(bitCount);
		int maxValue = (1 << bitCount) - 1;

		float voltage = (value / (float)maxValue) * 5.f;

		outputs[SEQUENCE_OUTPUT].setVoltage(voltage);

		if (blinkTimer > 0.f) {
			blinkTimer -= args.sampleTime;
			lights[BLINK_LIGHT].setBrightness(1.f);
		} else {
			lights[BLINK_LIGHT].setBrightness(0.f);
		}


		// Update lights based on the current state of the shift register
		for (int i = 0; i < 16; ++i) {
			int bitCount = static_cast<int>(params[LENGTH_PARAM].getValue());
			if (i < bitCount) {
				// Active bits (top N)
				lights[BIT_LIGHTS + i].setBrightness(shiftReg.bits[15 - i] ? 1.f : 0.f);
			} else {
				// Bits not used — dimmed or off
				lights[BIT_LIGHTS + i].setBrightness(0.f);
			}
		}
	}
	float blinkTimer = 0.f;
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
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 70.063)), module, TuringMaschine::SCALE_PARAM));

		addParam(createParamCentered<TL1105>(mm2px(Vec(15.24, 30.81)), module, TuringMaschine::SEED_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(15.24, 40.81)), module, TuringMaschine::WRITE_PARAM));

		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 90.478)), module, TuringMaschine::CLOCK_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 100.478)), module, TuringMaschine::RESET_INPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 108.713)), module, TuringMaschine::SEQUENCE_OUTPUT));

		for (int i = 0; i < 16; ++i) {
			float x = mm2px(Vec(4.0f, 20.0f)).x;  // adjust X as needed
			float y = mm2px(Vec(0.0f, 10.0f + i * 4.0f)).y;  // vertical stack
			addChild(createLight<SmallLight<GreenLight>>(Vec(x, y), module, TuringMaschine::BIT_LIGHTS + i));
		}
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(15.24, 25.81)), module, TuringMaschine::BLINK_LIGHT));
	}
};

Model* modelTuringMaschine = createModel<TuringMaschine, TuringMaschineWidget>("TuringMaschine");