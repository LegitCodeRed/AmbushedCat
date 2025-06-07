#include "plugin.hpp"
#include <bitset>

struct BitShiftRegister {
	std::bitset<16> bits;

	void reset() {
		bits.reset();
	}

	void shift(bool mutate, float changeProbability) {
		bool newBit;
		if (mutate && random::uniform() < changeProbability) {
			newBit = (random::u32() % 2 == 0);
		} else {
			newBit = bits[15];  // Copy MSB
		}

		bits <<= 1;
		bits.set(0, newBit);
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
		RATE_PARAM,
		WRITE_PARAM,
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
		LIGHTS_LEN
	};

	BitShiftRegister shiftReg;
	dsp::SchmittTrigger clockTrigger;

	TuringMaschine() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CHANGE_PARAM, 0.f, 1.f, 0.f, "Change");
		configParam(LENGTH_PARAM, 0.f, 1.f, 0.f, "Length");
		configParam(RATE_PARAM, 0.f, 1.f, 0.f, "Rate");

		configInput(CLOCK_INPUT, "Clock");
		configInput(RESET_INPUT, "Reset");
		configOutput(SEQUENCE_OUTPUT, "Sequence");
	}

	void process(const ProcessArgs& args) override {
		// Detect clock rising edge
		bool allowMutation = params[WRITE_PARAM].getValue() > 0.5f;
		float change = params[CHANGE_PARAM].getValue();
		
		if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage())) {
			shiftReg.shift(allowMutation, change);
		}

		float lenNorm = params[LENGTH_PARAM].getValue(); // 0.0 to 1.0
		int bitCount = static_cast<int>(std::round(lenNorm * 15.f)) + 1; // 1 to 16

		int value = shiftReg.getTopBitsAsInt(bitCount);
		int maxValue = (1 << bitCount) - 1; // e.g. 4 bits → max = 15

		float voltage = (value / (float)maxValue) * 5.f;
		outputs[SEQUENCE_OUTPUT].setVoltage(voltage);

		// Blink light at 1Hz
		blinkPhase += args.sampleTime;
		if (blinkPhase >= 1.f)
			blinkPhase -= 1.f;
		lights[BLINK_LIGHT].setBrightness(blinkPhase < 0.5f ? 1.f : 0.f);
	}

	float phase = 0.f;
	float blinkPhase = 0.f;
};

struct TuringMaschineWidget : ModuleWidget {
	TuringMaschineWidget(TuringMaschine* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/TuringMaschine.svg")));

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 46.063)), module, TuringMaschine::LENGTH_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 61.063)), module, TuringMaschine::CHANGE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 76.063)), module, TuringMaschine::RATE_PARAM));

		addParam(createParamCentered<TL1105>(mm2px(Vec(15.24, 35.81)), module, TuringMaschine::WRITE_PARAM));

		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 90.478)), module, TuringMaschine::CLOCK_INPUT));
		addInput(createInputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 100.478)), module, TuringMaschine::RESET_INPUT));

		addOutput(createOutputCentered<ThemedPJ301MPort>(mm2px(Vec(15.24, 108.713)), module, TuringMaschine::SEQUENCE_OUTPUT));

		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(15.24, 25.81)), module, TuringMaschine::BLINK_LIGHT));
	}
};

Model* modelTuringMaschine = createModel<TuringMaschine, TuringMaschineWidget>("TuringMaschine");