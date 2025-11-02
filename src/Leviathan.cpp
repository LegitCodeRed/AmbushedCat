#include "plugin.hpp"
#include "effects/distortion.h"
#include "framework/value.h"
#include "utilities/smooth_value.h"
#include <cmath>
#include <algorithm>
#include <array>

namespace {
} // namespace

struct Leviathan : Module {
        enum ParamIds {
                BLEND_PARAM,
                FOLD_PARAM,
                DIST_TYPE_PARAM,  // Vital distortion type selector
                CENTER_PARAM,
                DOOM_PARAM,
                PHASE_PARAM,
                DRIVE_PARAM,
                RECTIFY_PARAM,
                FLOW_PARAM,
                NOTCH_PARAM,
                SMOOSH_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                IN_L_INPUT,
                IN_R_INPUT,
                BLEND_CV_INPUT,
                FOLD_CV_INPUT,
                DIST_TYPE_CV_INPUT,  // Vital distortion type CV
                CENTER_CV_INPUT,
                DOOM_CV_INPUT,
                PHASE_CV_INPUT,
                DRIVE_CV_INPUT,
                RECTIFY_CV_INPUT,
                FLOW_CV_INPUT,
                NOTCH_CV_INPUT,
                SMOOSH_GATE_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                OUT_L_OUTPUT,
                OUT_R_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                BOOT_LEFT_LIGHT,
                BOOT_LEFT_CENTER_LIGHT,
                BOOT_RIGHT_CENTER_LIGHT,
                BOOT_RIGHT_LIGHT,
                SMOOSH_LIGHT,
                NUM_LIGHTS
        };

        std::unique_ptr<vital::Distortion> distL, distR;
        std::unique_ptr<vital::Output> sig_inL, sig_inR;
        std::array<vital::Value*, 2> valsL = {};
        std::array<vital::Value*, 2> valsR = {};

        float bootTimer = 1.2f;
        bool bootActive = true;

        Leviathan() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(BLEND_PARAM, 0.f, 1.f, 0.5f, "Blend");
                configParam(FOLD_PARAM, 0.f, 1.f, 0.25f, "Fold");
                configSwitch(DIST_TYPE_PARAM, 0.f, 5.f, 2.f, "Distortion Type",
                        {"Soft Clip", "Hard Clip", "Linear Fold", "Sin Fold", "Bit Crush", "Down Sample"});
                configParam(CENTER_PARAM, 0.f, 1.f, 0.5f, "Center");
                configParam(DOOM_PARAM, 0.f, 1.f, 0.f, "Doom");
                configParam(PHASE_PARAM, 0.f, 1.f, 0.f, "Phase");
                configParam(DRIVE_PARAM, 0.f, 1.f, 0.45f, "Drive");
                configParam(RECTIFY_PARAM, 0.f, 1.f, 0.f, "8vize");
                configSwitch(FLOW_PARAM, 0.f, 2.f, 0.f, "Routing", {"UND", "X", "OVR"});
                configSwitch(NOTCH_PARAM, 0.f, 2.f, 0.f, "Notch", {"Off", "1k", "Track"});
                configButton(SMOOSH_PARAM, "Smoosh");

                configInput(IN_L_INPUT, "Left audio");
                configInput(IN_R_INPUT, "Right audio");
                configInput(BLEND_CV_INPUT, "Blend CV");
                configInput(FOLD_CV_INPUT, "Fold CV");
                configInput(DIST_TYPE_CV_INPUT, "Distortion Type CV");
                configInput(CENTER_CV_INPUT, "Center CV");
                configInput(DOOM_CV_INPUT, "Doom CV");
                configInput(PHASE_CV_INPUT, "Phase CV");
                configInput(DRIVE_CV_INPUT, "Drive CV");
                configInput(RECTIFY_CV_INPUT, "8vize CV");
                configInput(FLOW_CV_INPUT, "Routing CV");
                configInput(NOTCH_CV_INPUT, "Notch CV");
                configInput(SMOOSH_GATE_INPUT, "Smoosh gate");

                configOutput(OUT_L_OUTPUT, "Left audio");
                configOutput(OUT_R_OUTPUT, "Right audio");

        // --- Vital DSP Integration ---
        distL = std::make_unique<vital::Distortion>();
        distR = std::make_unique<vital::Distortion>();

        sig_inL = std::make_unique<vital::Output>(vital::kMaxBufferSize);
        sig_inR = std::make_unique<vital::Output>(vital::kMaxBufferSize);

        distL->plug(sig_inL.get(), vital::Distortion::kAudio);
        distR->plug(sig_inR.get(), vital::Distortion::kAudio);

		for (size_t i = 0; i < valsL.size(); i++) {
			valsL[i] = new vital::SmoothValue(0);
            valsR[i] = new vital::SmoothValue(0);
		}

        distL->plug(valsL[0], vital::Distortion::kType);
        distL->plug(valsL[1], vital::Distortion::kDrive);
        distR->plug(valsR[0], vital::Distortion::kType);
        distR->plug(valsR[1], vital::Distortion::kDrive);

        onSampleRateChange();
        distL->reset(vital::poly_mask(-1));
        distR->reset(vital::poly_mask(-1));
        }

        void onSampleRateChange() override {
                float sr = APP ? APP->engine->getSampleRate() : 44100.f;
                if (distL) distL->setSampleRate(sr);
                if (distR) distR->setSampleRate(sr);
        }

        void onReset() override {
                bootActive = true;
                bootTimer = 1.2f;
                if (distL) distL->reset(vital::poly_mask(-1));
                if (distR) distR->reset(vital::poly_mask(-1));
        }

        float getParamWithCv(int paramId, int inputId, int c) {
                float value = params[paramId].getValue();
                if (inputId >= 0)
                        value += inputs[inputId].getPolyVoltage(c) / 5.f;
                return rack::math::clamp(value, 0.f, 1.f);
        }

        int getSwitchWithCv(int paramId, int inputId, int c) {
                float base = params[paramId].getValue();
                if (inputId >= 0)
                        base += inputs[inputId].getPolyVoltage(c) / 5.f;
                int mode = (int)std::round(base);
                return rack::math::clamp(mode, 0, 2);
        }

        int getDistTypeWithCv(int c) {
                float base = params[DIST_TYPE_PARAM].getValue();
                if (inputs[DIST_TYPE_CV_INPUT].isConnected())
                        base += inputs[DIST_TYPE_CV_INPUT].getPolyVoltage(c) * 1.2f; // 0-10V -> 0-12 (covers 0-5 with headroom)
                int distType = (int)std::round(base);
                return rack::math::clamp(distType, 0, 5);
        }

        void process(const ProcessArgs& args) override {
            // Update parameters for Vital processors
            float foldAmount = getParamWithCv(FOLD_PARAM, FOLD_CV_INPUT, 0);
            int distType = getDistTypeWithCv(0);

            // Drive is -30 to 30 dB range
            float driveDb = -30.f + foldAmount * 60.f;

            valsL[0]->set(distType);
            valsL[1]->set(driveDb);
            valsR[0]->set(distType);
            valsR[1]->set(driveDb);

            // Process smoothers
            for (auto& val : valsL) { val->process(1); }
            for (auto& val : valsR) { val->process(1); }

            // Get audio input
            float inL = inputs[IN_L_INPUT].getVoltage();
            float inR = inputs[IN_R_INPUT].isConnected() ? inputs[IN_R_INPUT].getVoltage() : inL;
            float dryL = inL;
            float dryR = inR;

            // --- Process Left Channel ---
            // Write to Vital input buffer (normalizing to +/- 1.0 range, Vital works internally with this range)
            ((vital::mono_float*)sig_inL->buffer)[0] = inL / 5.0f;
            ((vital::mono_float*)sig_inL->buffer)[1] = 0; // Second voice is unused for mono

            // Process with Vital
            distL->process(1);

            // Read from Vital output buffer
            float wetL = ((const vital::mono_float*)distL->output(vital::Distortion::kAudioOut)->buffer)[0] * 5.0f;

            // --- Process Right Channel ---
            ((vital::mono_float*)sig_inR->buffer)[0] = inR / 5.0f;
            ((vital::mono_float*)sig_inR->buffer)[1] = 0;
            distR->process(1);
            float wetR = ((const vital::mono_float*)distR->output(vital::Distortion::kAudioOut)->buffer)[0] * 5.0f;

            // --- Final Mixing ---
            float blend = rack::math::clamp(params[BLEND_PARAM].getValue() + inputs[BLEND_CV_INPUT].getPolyVoltage(0) / 5.f, 0.f, 1.f);
            float outL = rack::math::crossfade(dryL, wetL, blend);
            float outR = rack::math::crossfade(dryR, wetR, blend);

            outputs[OUT_L_OUTPUT].setVoltage(outL);
            outputs[OUT_R_OUTPUT].setVoltage(outR);

            // --- Lights ---
            bool smoosh = params[SMOOSH_PARAM].getValue() > 0.5f || inputs[SMOOSH_GATE_INPUT].getVoltage() > 2.f;
            lights[SMOOSH_LIGHT].setBrightness(smoosh ? 1.f : 0.f);
            bootTimer -= args.sampleTime;
            if (bootTimer <= 0.f) {
                bootActive = false;
                lights[BOOT_LEFT_LIGHT].setBrightness(0.f);
                lights[BOOT_LEFT_CENTER_LIGHT].setBrightness(0.f);
                lights[BOOT_RIGHT_CENTER_LIGHT].setBrightness(0.f);
                lights[BOOT_RIGHT_LIGHT].setBrightness(0.f);
            } else {
                 float bootProgress = rack::math::clamp(bootTimer / 1.2f, 0.f, 1.f);
                 float fade = 1.f - bootProgress;
                 lights[BOOT_LEFT_LIGHT].setSmoothBrightness(0.8f * fade, args.sampleTime);
                 lights[BOOT_LEFT_CENTER_LIGHT].setSmoothBrightness(0.8f * fade, args.sampleTime);
                 lights[BOOT_RIGHT_CENTER_LIGHT].setSmoothBrightness(0.5f * fade, args.sampleTime);
                 lights[BOOT_RIGHT_LIGHT].setSmoothBrightness(0.9f * fade, args.sampleTime);
            }
        }
};

struct BackgroundImage : Widget {
	std::string imagePath = asset::plugin(pluginInstance, "res/TextureDemonMain.png");
	widget::SvgWidget* svgWidget;

	BackgroundImage() {
		// Create & load SVG child safely
		svgWidget = new widget::SvgWidget();
		addChild(svgWidget);
		try {
			auto svg = APP->window->loadSvg(asset::plugin(pluginInstance, "res/Leviathan.svg"));
			if (svg) {
				svgWidget->setSvg(svg);
			} else {
				WARN("SVG returned null: res/Leviathan.svg");
			}
		} catch (const std::exception& e) {
			WARN("Exception loading SVG res/Leviathan.svg: %s", e.what());
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

struct LeviathanWidget : ModuleWidget {
        LeviathanWidget(Leviathan* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Leviathan.svg")));

                auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size;
		addChild(bg);

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                // Boot lights at header
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(14.0, 13.0)), module, Leviathan::BOOT_LEFT_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(21.0, 13.0)), module, Leviathan::BOOT_LEFT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(30.0, 13.0)), module, Leviathan::BOOT_RIGHT_CENTER_LIGHT));
                addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(37.0, 13.0)), module, Leviathan::BOOT_RIGHT_LIGHT));

                // === MAIN KNOB SECTION ===
                // Row 1: BLEND / FOLD (y=23mm)
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(13.0, 23.0)), module, Leviathan::BLEND_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(37.8, 23.0)), module, Leviathan::FOLD_PARAM));

                // Row 2: CENTER / DOOM (y=36mm)
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(13.0, 36.0)), module, Leviathan::CENTER_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(37.8, 36.0)), module, Leviathan::DOOM_PARAM));

                // Row 3: PHASE / DRIVE (y=49mm)
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(13.0, 49.0)), module, Leviathan::PHASE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(37.8, 49.0)), module, Leviathan::DRIVE_PARAM));

                // Row 4: 8VIZE (y=62mm)
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(13.0, 62.0)), module, Leviathan::RECTIFY_PARAM));

                // === SWITCHES & BUTTON SECTION ===
                // Distortion Type selector (6-position switch near FOLD)
                addParam(createParamCentered<CKSSThreeHorizontal>(mm2px(Vec(25.4, 58.0)), module, Leviathan::DIST_TYPE_PARAM));

                // FLOW / NOTCH switches (y=67mm and 75mm)
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(37.8, 63.0)), module, Leviathan::FLOW_PARAM));
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(37.8, 77.0)), module, Leviathan::NOTCH_PARAM));

                // SMOOSH button (y=83mm)
                addParam(createParamCentered<TL1105>(mm2px(Vec(25.4, 76.0)), module, Leviathan::SMOOSH_PARAM));
                addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(25.4, 82.0)), module, Leviathan::SMOOSH_LIGHT));

                // === CV INPUT SECTION (Bottom area, grouped) ===
                // Row 1 CVs (y=96mm)
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5, 92.0)), module, Leviathan::BLEND_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.5, 92.0)), module, Leviathan::FOLD_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(25.4, 92.0)), module, Leviathan::DIST_TYPE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(33.3, 92.0)), module, Leviathan::CENTER_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3, 92.0)), module, Leviathan::DOOM_CV_INPUT));

                // Row 2 CVs (y=105mm)
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5, 101.0)), module, Leviathan::PHASE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.5, 101.0)), module, Leviathan::DRIVE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(33.3, 101.0)), module, Leviathan::RECTIFY_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3, 101.0)), module, Leviathan::FLOW_CV_INPUT));

                // Row 3 CVs (y=114mm)
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(33.3, 110.0)), module, Leviathan::NOTCH_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(42.3, 110.0)), module, Leviathan::SMOOSH_GATE_INPUT));

                // === AUDIO I/O SECTION ===
                // Audio inputs (y=114mm, right side)
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(8.5, 119.0)), module, Leviathan::IN_L_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.5, 119.0)), module, Leviathan::IN_R_INPUT));

                // Audio outputs (y=122mm, right side)
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(33.3, 119.0)), module, Leviathan::OUT_L_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(42.3, 119.0)), module, Leviathan::OUT_R_OUTPUT));
        }
};

Model* modelLeviathan = createModel<Leviathan, LeviathanWidget>("Leviathan");
