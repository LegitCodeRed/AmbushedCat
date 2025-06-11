#include "plugin.hpp"
#include <cmath>

struct TuringGateExpander : Module {
       enum ParamId {
               SWING_PARAM,
               RATE_PARAM,
               PARAMS_LEN
       };
	enum InputId {
		INPUTS_LEN
	};
	enum OutputId {
		GATE_OUTPUT_COMBO_1, // 1 + 2
		GATE_OUTPUT_COMBO_2, // 2 + 4
		GATE_OUTPUT_COMBO_3, // 4 + 7
		GATE_OUTPUT_COMBO_4, // 1 + 2 + 4 + 7
		GATE_OUTPUTS,
		NUM_OUTPUTS = GATE_OUTPUTS + 12,
		OUTPUTS_LEN
	};
	enum LightId {
		GATE_LIGHTS, // start index for 8 bit LEDs
		COMBO_LIGHT_1 = GATE_LIGHTS + 8,
		COMBO_LIGHT_2,
		COMBO_LIGHT_3,
		COMBO_LIGHT_4,
		NUM_LIGHTS,
		LIGHTS_LEN
	};

	float value[2] = {};

       TuringGateExpander() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(SWING_PARAM, -1.f, 1.f, 0.f, "Swing");
		configParam(RATE_PARAM, 0.5f, 8.f, 2.f, "Rate", "x", 0.f, 0.5f);
		getLeftExpander().producerMessage = &value[0];
		getLeftExpander().consumerMessage = &value[1];
		paramQuantities[RATE_PARAM]->snapEnabled = true;
       }

	
	void ClearOutputs(){
		// If no valid expander data, clear everything
		for (int i = 0; i < 8; ++i) {
			outputs[GATE_OUTPUTS + i].setVoltage(0.f);
			lights[GATE_LIGHTS + i].setBrightness(0.f);
		}

		outputs[GATE_OUTPUT_COMBO_1].setVoltage(0.f);
		outputs[GATE_OUTPUT_COMBO_2].setVoltage(0.f);
		outputs[GATE_OUTPUT_COMBO_3].setVoltage(0.f);
		outputs[GATE_OUTPUT_COMBO_4].setVoltage(0.f);

		lights[COMBO_LIGHT_1].setBrightness(0.f);
		lights[COMBO_LIGHT_2].setBrightness(0.f);
		lights[COMBO_LIGHT_3].setBrightness(0.f);
		lights[COMBO_LIGHT_4].setBrightness(0.f);
	}

       void process(const ProcessArgs& args) override {
               float swing = params[SWING_PARAM].getValue();
               float rateParam = params[RATE_PARAM].getValue();
               float rate = std::round(rateParam * 2.f) / 2.f; // Quantize to 0.5 steps

               if (getLeftExpander().module && getLeftExpander().module->model && getLeftExpander().module->model->slug == "TuringMaschine") {
                       float* value = (float*) getLeftExpander().consumerMessage;
                       if (value) {
				uint8_t bits = (uint8_t)value[0];
				for (int i = 0; i < 8; i++) {
					bool bit = (bits >> i) & 0x1;
					outputs[GATE_OUTPUTS + i].setVoltage(bit ? 10.f : 0.f);
					lights[GATE_LIGHTS + i].setBrightness(bit ? 1.f : 0.f);
				}

				bool g1 = (bits >> 1) & 0x1;
				bool g2 = (bits >> 2) & 0x1;
				bool g4 = (bits >> 4) & 0x1;
				bool g7 = (bits >> 7) & 0x1;

				// Define new gate outputs as combinations
				bool outA = g1 || g2;               // 1 + 2
				bool outB = g2 || g4;               // 2 + 4
				bool outC = g4 || g7;               // 4 + 7
				bool outD = g1 || g2 || g4 || g7;   // 1 + 2 + 4 + 7

				outputs[GATE_OUTPUT_COMBO_1].setVoltage(outA ? 10.f : 0.f);
				outputs[GATE_OUTPUT_COMBO_2].setVoltage(outB ? 10.f : 0.f);
				outputs[GATE_OUTPUT_COMBO_3].setVoltage(outC ? 10.f : 0.f);
				outputs[GATE_OUTPUT_COMBO_4].setVoltage(outD ? 10.f : 0.f);

				lights[COMBO_LIGHT_1].setBrightness(outA ? 1.f : 0.f);
				lights[COMBO_LIGHT_2].setBrightness(outB ? 1.f : 0.f);
				lights[COMBO_LIGHT_3].setBrightness(outC ? 1.f : 0.f);
				lights[COMBO_LIGHT_4].setBrightness(outD ? 1.f : 0.f);
			}
			return;
		}
		ClearOutputs();
	}
};


struct TuringGateExpanderWidget : ModuleWidget {
	TuringGateExpanderWidget(TuringGateExpander* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/TuringGateExpander.svg")));

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(5.0, 10.0)), module, TuringGateExpander::SWING_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(15.0, 10.0)), module, TuringGateExpander::RATE_PARAM));

                for (int i = 0; i < 8; i++) {
            addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(5.0, 20.0 + i * 12.5)), module, TuringGateExpander::GATE_OUTPUTS + i));
			addChild(createLightCentered<SmallLight<RedLight>>(
				mm2px(Vec(2.5, 20.0 + i * 12.5 - 6)),
				module,
				TuringGateExpander::GATE_LIGHTS + i
			));
        }

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0,  20.0 + 1 * 12.5 - 5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_1));
		addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 1 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_1));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0, 20.0 + 2 * 12.5 -5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_2));
		addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 2 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_2));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0, 20.0 + 6 * 12.5 - 5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_3));
		addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 6 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_3));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0, 20.0 + 7 * 12.5 - 5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_4));
		addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 7 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_4));
	}
};


Model* modelTuringGateExpander = createModel<TuringGateExpander, TuringGateExpanderWidget>("TuringGateExpander");