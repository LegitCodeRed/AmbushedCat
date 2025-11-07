#include "plugin.hpp"
#include "BuerBus.hpp"

#include <algorithm>
#include <string>

using rack::math::clamp;

namespace {

struct Buer : rack::engine::Module {
        enum ParamIds {
                CV_SCALE_PARAMS_BASE,
                MODE_SCALE_PARAMS_BASE = CV_SCALE_PARAMS_BASE + 16,
                NUM_PARAMS = MODE_SCALE_PARAMS_BASE + 16
        };
        enum InputIds {
                CV_INPUTS_BASE,
                MODE_INPUTS_BASE = CV_INPUTS_BASE + 16,
                NUM_INPUTS = MODE_INPUTS_BASE + 16
        };
        enum OutputIds {
                NUM_OUTPUTS
        };
        enum LightIds {
                NUM_LIGHTS
        };

        BuerBus::ToLilith outboundMessages[2]{};
        BuerBus::FromLilith inboundMessages[2]{};

        Buer() {
                this->config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                for (int i = 0; i < 16; ++i) {
                        this->configParam(CV_SCALE_PARAMS_BASE + i, -1.f, 1.f, 1.f,
                                          "CV modulation scale", "", 0.f, 1.f);
                        this->configParam(MODE_SCALE_PARAMS_BASE + i, -1.f, 1.f, 1.f,
                                          "Gate mode modulation scale", "", 0.f, 1.f);

                        this->configInput(CV_INPUTS_BASE + i,
                                          "Step " + std::to_string(i + 1) + " CV modulation");
                        this->configInput(MODE_INPUTS_BASE + i,
                                          "Step " + std::to_string(i + 1) + " gate mode modulation");
                }

                this->leftExpander.producerMessage = &outboundMessages[0];
                this->leftExpander.consumerMessage = &inboundMessages[0];

                for (int i = 0; i < 2; ++i) {
                        outboundMessages[i].magic = BuerBus::MAGIC;
                        outboundMessages[i].version = 1;
                        outboundMessages[i].connected = 0;

                        inboundMessages[i].magic = BuerBus::MAGIC;
                        inboundMessages[i].version = 1;
                        inboundMessages[i].numSteps = 0;
                        inboundMessages[i].activeSteps = 0;
                }
        }

        void process(const ProcessArgs& args) override {
                // Write to Lilith expander using VCV Rack's message flipping system
                Module* leftModule = getLeftExpander().module;
                bool attachedToLilith = leftModule && leftModule->model &&
                                         (leftModule->model->slug == "Lilith" ||
                                          leftModule->model->slug == "LilithAdvance");

                // Read from Lilith's consumer message
                const auto* inbound = attachedToLilith ?
                    static_cast<const BuerBus::FromLilith*>(leftModule->rightExpander.consumerMessage) : nullptr;

                int activeSteps = 16;
                if (inbound && inbound->magic == BuerBus::MAGIC && inbound->version == 1) {
                        if (inbound->activeSteps > 0 && inbound->activeSteps <= 16)
                                activeSteps = inbound->activeSteps;
                } else if (attachedToLilith) {
                        activeSteps = (leftModule->model->slug == "Lilith") ? 8 : 16;
                }

                if (attachedToLilith) {
                        // Write to Lilith's producer buffer (VCV will flip it to consumer on Lilith's side)
                        auto* outbound = static_cast<BuerBus::ToLilith*>(leftModule->rightExpander.producerMessage);
                        if (outbound) {
                                outbound->magic = BuerBus::MAGIC;
                                outbound->version = 1;
                                outbound->connected = 1;

                                for (int i = 0; i < 16; ++i) {
                                        float cvScale = this->params[CV_SCALE_PARAMS_BASE + i].getValue();
                                        float cvValue = 0.f;
                                        if (i < activeSteps && this->inputs[CV_INPUTS_BASE + i].isConnected()) {
                                                cvValue = this->inputs[CV_INPUTS_BASE + i].getVoltage() * cvScale;
                                        }
                                        outbound->cvMod[i] = cvValue;

                                        float modeScale = this->params[MODE_SCALE_PARAMS_BASE + i].getValue();
                                        float modeValue = 0.f;
                                        if (i < activeSteps && this->inputs[MODE_INPUTS_BASE + i].isConnected()) {
                                                float normalized = this->inputs[MODE_INPUTS_BASE + i].getVoltage() / 5.f;
                                                modeValue = clamp(normalized * modeScale, -2.f, 2.f);
                                        }
                                        outbound->modeMod[i] = modeValue;
                                }

                                // Request VCV Rack to flip the buffers
                                leftModule->rightExpander.requestMessageFlip();
                        }
                }
        }
};


struct BackgroundImage : Widget {
	std::string imagePath = asset::plugin(pluginInstance, "res/TextureDemonMainV2.png");
	widget::SvgWidget* svgWidget;

	BackgroundImage() {
		// Create & load SVG child safely
		svgWidget = new widget::SvgWidget();
		addChild(svgWidget);
		try {
			auto svg = APP->window->loadSvg(asset::plugin(pluginInstance, "res/Buer.svg"));
			if (svg) {
				svgWidget->setSvg(svg);
			} else {
				WARN("SVG returned null: res/Buer.svg");
			}
		} catch (const std::exception& e) {
			WARN("Exception loading SVG res/Buer.svg: %s", e.what());
			// Leave svgWidget with no SVG; still safe to run.
		}
        }

	void draw(const DrawArgs& args) override {
		// Draw background image first
                std::shared_ptr<Image> image = APP->window->loadImage(imagePath);
                if (image && box.size.x > 0.f && box.size.y > 0.f) {
			int w = box.size.x;
			int h = box.size.y;

			NVGpaint paint = nvgImagePattern(args.vg, 0, 0, w, h, 0.0f, image->handle, 1.0f);
			nvgBeginPath(args.vg);
			nvgRect(args.vg, 0, 0, w, h);
			nvgFillPaint(args.vg, paint);
			nvgFill(args.vg);
		}
		// SVG will be drawn automatically by the child SvgWidget
		Widget::draw(args);
	}
};

struct BuerWidget : rack::app::ModuleWidget {
        explicit BuerWidget(Buer* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Buer.svg")));

                auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size;
		addChild(bg);

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH,
                                                      RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                const float columnXs[4] = {14.0f, 33.0f, 52.0f, 71.0f};

                for (int row = 0; row < 4; ++row) {
                        float knobY = 6.0f + 14.0f * (row * 1.1);
                        float jackY = 14.0f + 14.0f * (row * 1.1);
                        for (int col = 0; col < 4; ++col) {
                                int index = row * 4 + col;
                                addParam(createParamCentered<Trimpot>(mm2px(Vec(columnXs[col], knobY)), module,
                                                                      Buer::CV_SCALE_PARAMS_BASE + index));
                                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(columnXs[col], jackY)), module,
                                                                         Buer::CV_INPUTS_BASE + index));
                        }
                }

                for (int row = 0; row < 4; ++row) {
                        float knobY = 68.0f + 14.0f * (row * 1.1);
                        float jackY = 76.0f + 14.0f * (row * 1.1);
                        for (int col = 0; col < 4; ++col) {
                                int index = row * 4 + col;
                                addParam(createParamCentered<Trimpot>(mm2px(Vec(columnXs[col], knobY)), module,
                                                                      Buer::MODE_SCALE_PARAMS_BASE + index));
                                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(columnXs[col], jackY)), module,
                                                                         Buer::MODE_INPUTS_BASE + index));
                        }
                }
        }
};

} // namespace

Model* modelBuer = createModel<Buer, BuerWidget>("Buer");

