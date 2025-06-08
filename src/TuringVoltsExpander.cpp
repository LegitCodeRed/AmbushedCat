#include "plugin.hpp"


struct TuringVoltsExpander : Module {
	enum ParamIds {
		BIT_0_PARAM,
		BIT_1_PARAM,
		BIT_2_PARAM,
		BIT_3_PARAM,
		BIT_4_PARAM,
		NUM_PARAMS
	};
	enum OutputIds {
		VOLTS_OUTPUT,
		NUM_OUTPUTS
	};

	// Message struct shared with main module
	TuringVoltsExpanderMessage message;

	TuringVoltsExpander() {
		config(NUM_PARAMS, 0, NUM_OUTPUTS, 0);
		for (int i = 0; i < 5; i++) {
			configParam(i, 0.f, 10.f, 0.f, "Bit " + std::to_string(i) + " voltage");
		}
	}

	void process(const ProcessArgs& args) override {
		uint8_t bits = 0;
		
		Module* mod = this;
		while (mod->leftExpander.module) {
			mod = mod->leftExpander.module;

			if (mod->model && mod->model->slug == "TuringMaschine") {
				if (getLeftExpander().consumerMessage) {
					auto* msg = static_cast<TuringVoltsExpanderMessage*>(getLeftExpander().consumerMessage);
					bits = msg->bits;
				}
				break;
			}
		}

		
		// Sum voltages where bits are active
		float outputVoltage = 0.f;
		for (int i = 0; i < 5; i++) {
			if (bits & (1 << i)) {
				outputVoltage += params[i].getValue() * 0.2f;  // emulate resistor scaling
			}
		}
		outputVoltage = clamp(outputVoltage, 0.f, 10.f);
		outputs[VOLTS_OUTPUT].setVoltage(outputVoltage);
	}
};

struct TuringVoltsExpanderWidget : ModuleWidget {
	TuringVoltsExpanderWidget(TuringVoltsExpander* module) {
		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/TuringVoltsExpander.svg")));

		for (int i = 0; i < 5; i++) {
			addParam(createParamCentered<RoundSmallBlackKnob>(
				mm2px(Vec(15.0, 20.0 + i * 15.0)), module, i));
		}

		addOutput(createOutputCentered<PJ301MPort>(
			mm2px(Vec(15.0, 100.0)), module, TuringVoltsExpander::VOLTS_OUTPUT));
	}
};


Model* modelTuringVoltsExpander = createModel<TuringVoltsExpander, TuringVoltsExpanderWidget>("TuringVoltsExpander");