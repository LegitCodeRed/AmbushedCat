#include "plugin.hpp"
#include <osdialog.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include "NeuralAmpModelerCore/NAM/dsp.h"
#include "NeuralAmpModelerCore/NAM/get_dsp.h"

namespace {
inline float dbToGain(float db) {
        return std::pow(10.0f, db / 20.0f);
}

// Soft clipping/saturation function
inline float softClip(float x) {
        // Smooth tanh-based saturation
        if (x > 1.5f)
                return 1.0f;
        if (x < -1.5f)
                return -1.0f;
        return std::tanh(x);
}

struct ToneShaper {
        float lowState = 0.f;
        float highState = 0.f;
        float cachedBassGain = 1.f;
        float cachedTrebleGain = 1.f;
        float lastToneAmount = 0.f;
        float lowAlpha = 0.f;
        float highAlpha = 0.f;
        float lastSampleRate = 0.f;

        void reset() {
                lowState = 0.f;
                highState = 0.f;
        }

        void updateCoefficients(float sampleRate) {
                // Pre-calculate filter coefficients (only when sample rate changes)
                const float lowCutoff = 200.f;
                const float highCutoff = 2000.f;
                lowAlpha = std::exp(-2.f * M_PI * lowCutoff / sampleRate);
                highAlpha = std::exp(-2.f * M_PI * highCutoff / sampleRate);
                lastSampleRate = sampleRate;
        }

        inline float process(float input, float toneAmount, float sampleRate) {
                // Update coefficients only if sample rate changed
                if (std::abs(sampleRate - lastSampleRate) > 0.1f) {
                        updateCoefficients(sampleRate);
                }

                // Update gains only if tone changed significantly (> 1%)
                if (std::abs(toneAmount - lastToneAmount) > 0.01f) {
                        // Fast approximation: x^(k) ≈ 1 + k*ln(x) for small k
                        cachedBassGain = dbToGain(-toneAmount * 6.f);
                        cachedTrebleGain = dbToGain(toneAmount * 8.f);
                        lastToneAmount = toneAmount;
                }

                // Single-pole filters (very efficient)
                float oneMinusLowAlpha = 1.f - lowAlpha;
                lowState = oneMinusLowAlpha * input + lowAlpha * lowState;

                float oneMinusHighAlpha = 1.f - highAlpha;
                highState = oneMinusHighAlpha * input + highAlpha * highState;

                float highs = input - highState;
                float mids = highState - lowState;

                return lowState * cachedBassGain + mids + highs * cachedTrebleGain;
        }
};
} // namespace

struct NergalAmp : Module {
        enum ParamIds {
                DRIVE_PARAM,
                TONE_PARAM,
                INPUT_PARAM,
                OUTPUT_PARAM,
                MIX_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                SIGNAL_INPUT,
                DRIVE_CV_INPUT,
                TONE_CV_INPUT,
                INPUT_CV_INPUT,
                OUTPUT_CV_INPUT,
                MIX_CV_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                SIGNAL_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                LOADED_LIGHT,
                NUM_LIGHTS
        };

        // Audio thread variables (protected by modelMutex)
        std::unique_ptr<nam::DSP> model;
        std::string modelPath;
        double modelSampleRate = 48000.0;
        double resamplePhase = 0.0;
        NAM_SAMPLE lastModelOutput = 0.0;
        float previousModelInput = 0.0f;
        bool firstFrame = true;
        ToneShaper tone;

        // Thread safety
        std::mutex modelMutex;

        // Deferred loading (set from UI thread, processed on audio thread)
        std::string pendingModelPath;
        bool loadPending = false;

        // Settings
        bool enableClipper = true;

        // UI state
        std::string lastDirectory;

        NergalAmp() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(INPUT_PARAM, -24.f, 24.f, 0.f, "Input Trim", " dB");
                configParam(DRIVE_PARAM, 0.f, 2.f, 1.f, "Drive/Gain", "x");
                configParam(TONE_PARAM, -1.f, 1.f, 0.f, "Tone");
                configParam(MIX_PARAM, 0.f, 1.f, 1.f, "Dry/Wet Mix", "%", 0.f, 100.f);
                configParam(OUTPUT_PARAM, -24.f, 24.f, 0.f, "Output Level", " dB");

                configInput(SIGNAL_INPUT, "Audio");
                configInput(INPUT_CV_INPUT, "Input Trim CV");
                configInput(DRIVE_CV_INPUT, "Drive CV");
                configInput(TONE_CV_INPUT, "Tone CV");
                configInput(MIX_CV_INPUT, "Mix CV");
                configInput(OUTPUT_CV_INPUT, "Output Level CV");

                configOutput(SIGNAL_OUTPUT, "Audio");

                onSampleRateChange();
        }

        void onSampleRateChange() override {
                Module::onSampleRateChange();
                std::lock_guard<std::mutex> lock(modelMutex);
                resamplePhase = 0.0;
                firstFrame = true;
                tone.reset();
                if (model) {
                        const double effectiveRate = modelSampleRate > 0.0 ? modelSampleRate : APP->engine->getSampleRate();
                        model->Reset(effectiveRate, 64);
                }
        }

        void process(const ProcessArgs& args) override {
                // Check for pending model load (happens on audio thread to avoid blocking)
                if (loadPending) {
                        loadPending = false;
                        loadModelInternal(pendingModelPath);
                }

                // Get input signal and normalize from ±10V to ±1 range
                const float inVolts = inputs[SIGNAL_INPUT].getVoltage();
                const float drySignal = inVolts * 0.1f;

                // Get parameter values with CV modulation (optimized checks)
                float inputGain = dbToGain(math::clamp(
                        params[INPUT_PARAM].getValue() +
                        (inputs[INPUT_CV_INPUT].isConnected() ? inputs[INPUT_CV_INPUT].getVoltage() * 2.4f : 0.f),
                        -24.f, 24.f));

                float driveAmount = math::clamp(
                        params[DRIVE_PARAM].getValue() +
                        (inputs[DRIVE_CV_INPUT].isConnected() ? inputs[DRIVE_CV_INPUT].getVoltage() * 0.2f : 0.f),
                        0.f, 2.f);

                float toneAmount = math::clamp(
                        params[TONE_PARAM].getValue() +
                        (inputs[TONE_CV_INPUT].isConnected() ? inputs[TONE_CV_INPUT].getVoltage() * 0.1f : 0.f),
                        -1.f, 1.f);

                float mixAmount = math::clamp(
                        params[MIX_PARAM].getValue() +
                        (inputs[MIX_CV_INPUT].isConnected() ? inputs[MIX_CV_INPUT].getVoltage() * 0.1f : 0.f),
                        0.f, 1.f);

                float outputGain = dbToGain(math::clamp(
                        params[OUTPUT_PARAM].getValue() +
                        (inputs[OUTPUT_CV_INPUT].isConnected() ? inputs[OUTPUT_CV_INPUT].getVoltage() * 2.4f : 0.f),
                        -24.f, 24.f));

                // Stage 1: Apply input trim
                const float trimmed = drySignal * inputGain;

                // Stage 2: Process through NAM model (with minimal mutex time)
                float modelOutput = trimmed;
                bool hasModel = false;

                // Try lock first - if we can't get it, skip model processing this frame (reduces blocking)
                if (modelMutex.try_lock()) {
                        hasModel = (model != nullptr);

                        if (model) {
                                // Cache frequently used values
                                const float hostRate = args.sampleRate;
                                const float targetRate = modelSampleRate > 0.0 ? modelSampleRate : hostRate;
                                const float ratio = targetRate / hostRate;

                                float phase = resamplePhase;
                                float total = phase + ratio;
                                int steps = (int)total;
                                resamplePhase = total - steps;

                                if (firstFrame) {
                                        previousModelInput = trimmed;
                                        firstFrame = false;
                                }

                                // Linear interpolation resampling (faster than crossfade)
                                if (steps > 0) {
                                        const float invRatio = 1.f / ratio;
                                        const float delta = trimmed - previousModelInput;

                                        for (int s = 0; s < steps; ++s) {
                                                float t = ((float)(s + 1) - phase) * invRatio;
                                                t = simd::clamp(t, 0.f, 1.f);
                                                float interp = previousModelInput + delta * t;

                                                NAM_SAMPLE inputFrame = (NAM_SAMPLE)interp;
                                                NAM_SAMPLE outputFrame = 0.0;
                                                model->process(&inputFrame, &outputFrame, 1);
                                                lastModelOutput = outputFrame;
                                        }
                                }

                                previousModelInput = trimmed;
                                modelOutput = (float)lastModelOutput;
                        }

                        modelMutex.unlock();
                }

                // Stage 3: Apply drive/saturation (conditional execution)
                float driven = modelOutput * driveAmount;
                if (enableClipper && driveAmount > 1.0f) {
                        driven = softClip(driven);
                }

                // Stage 4: Tone shaping (optimized with caching)
                float shaped = tone.process(driven, toneAmount, args.sampleRate);

                // Stage 5: Dry/Wet mix (optimized)
                float mixed = drySignal + mixAmount * (shaped - drySignal);

                // Stage 6: Output level and convert back to ±10V
                outputs[SIGNAL_OUTPUT].setVoltage(mixed * outputGain * 10.f);

                lights[LOADED_LIGHT].setBrightness(hasModel ? 1.f : 0.f);
        }

        void clearModel() {
                std::lock_guard<std::mutex> lock(modelMutex);
                model.reset();
                modelPath.clear();
                modelSampleRate = 48000.0;
                lastModelOutput = 0.0;
                resamplePhase = 0.0;
                firstFrame = true;
                tone.reset();
        }

        // Internal loading - called from audio thread with path already validated
        void loadModelInternal(const std::string& path) {
                if (path.empty()) {
                        clearModel();
                        return;
                }

                try {
                        std::filesystem::path fsPath(path);

                        // Load the model (this is the slow part)
                        auto loaded = nam::get_dsp(fsPath);

                        if (!loaded) {
                                WARN("NergalAmp: Failed to load model %s - get_dsp returned null", path.c_str());
                                clearModel();
                                return;
                        }

                        double sampleRate = loaded->GetExpectedSampleRate();
                        if (!(sampleRate > 0.0)) {
                                sampleRate = APP->engine->getSampleRate();
                                if (!(sampleRate > 0.0)) {
                                        sampleRate = 48000.0;
                                }
                        }
                        loaded->Reset(sampleRate, 64);

                        // Quick atomic swap with mutex
                        {
                                std::lock_guard<std::mutex> lock(modelMutex);
                                model = std::move(loaded);
                                modelPath = path;
                                modelSampleRate = sampleRate;
                                lastModelOutput = 0.0;
                                resamplePhase = 0.0;
                                firstFrame = true;
                                tone.reset();
                        }

                        // Update UI state (outside mutex)
                        if (fsPath.has_parent_path()) {
                                lastDirectory = fsPath.parent_path().string();
                        }

                        INFO("NergalAmp: Successfully loaded model %s at %.0f Hz", path.c_str(), sampleRate);
                }
                catch (const std::exception& e) {
                        WARN("NergalAmp failed to load model %s: %s", path.c_str(), e.what());
                        clearModel();
                }
        }

        // Public API - called from UI thread, defers to audio thread
        void loadModel(const std::string& path) {
                pendingModelPath = path;
                loadPending = true;
        }

        json_t* dataToJson() override {
                json_t* root = json_object();

                // Copy strings with mutex protection
                std::string path, dir;
                {
                        std::lock_guard<std::mutex> lock(modelMutex);
                        path = modelPath;
                }
                dir = lastDirectory; // lastDirectory doesn't need mutex (UI only)

                if (!path.empty()) {
                        json_object_set_new(root, "modelPath", json_string(path.c_str()));
                }
                if (!dir.empty()) {
                        json_object_set_new(root, "lastDirectory", json_string(dir.c_str()));
                }
                json_object_set_new(root, "enableClipper", json_boolean(enableClipper));
                return root;
        }

        void dataFromJson(json_t* root) override {
                json_t* pathJ = json_object_get(root, "modelPath");
                if (pathJ && json_is_string(pathJ)) {
                        // loadModel is already deferred, safe to call from any thread
                        loadModel(json_string_value(pathJ));
                }
                json_t* dirJ = json_object_get(root, "lastDirectory");
                if (dirJ && json_is_string(dirJ)) {
                        lastDirectory = json_string_value(dirJ);
                }
                json_t* clipperJ = json_object_get(root, "enableClipper");
                if (clipperJ) {
                        enableClipper = json_boolean_value(clipperJ);
                }
        }
};

struct NergalAmpWidget : ModuleWidget {
        NergalAmpWidget(NergalAmp* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/NergalAmp.svg")));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                // Model loaded indicator
                addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(15.24, 15.0)), module, NergalAmp::LOADED_LIGHT));

                // Parameters - left column (knobs)
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 30.0)), module, NergalAmp::INPUT_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 50.0)), module, NergalAmp::DRIVE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 70.0)), module, NergalAmp::TONE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 90.0)), module, NergalAmp::MIX_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 110.0)), module, NergalAmp::OUTPUT_PARAM));

                // CV Inputs - right column
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 30.0)), module, NergalAmp::INPUT_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 50.0)), module, NergalAmp::DRIVE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 70.0)), module, NergalAmp::TONE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 90.0)), module, NergalAmp::MIX_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 110.0)), module, NergalAmp::OUTPUT_CV_INPUT));

                // Audio I/O at bottom
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.62, 120.0)), module, NergalAmp::SIGNAL_INPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(22.86, 120.0)), module, NergalAmp::SIGNAL_OUTPUT));
        }

        void appendContextMenu(Menu* menu) override {
                ModuleWidget::appendContextMenu(menu);

                menu->addChild(new MenuSeparator());

                auto* module = getModule<NergalAmp>();

                std::string labelText = "Model: (none)";
                if (module) {
                        // Thread-safe copy of model path
                        std::string path;
                        {
                                std::lock_guard<std::mutex> lock(module->modelMutex);
                                path = module->modelPath;
                        }
                        if (!path.empty()) {
                                labelText = "Model: " + std::filesystem::path(path).filename().string();
                        }
                }
                MenuLabel* label = createMenuLabel(labelText);
                menu->addChild(label);

                struct LoadItem : MenuItem {
                        NergalAmp* module = nullptr;
                        void onAction(const event::Action& e) override {
                                if (!module)
                                        return;
                                osdialog_filters* filters = osdialog_filters_parse("NAM file:n:nam");
                                const char* initialPath = module->lastDirectory.empty() ? nullptr : module->lastDirectory.c_str();
                                char* path = osdialog_file(OSDIALOG_OPEN, initialPath, nullptr, filters);
                                osdialog_filters_free(filters);
                                if (path) {
                                        std::string chosen(path);
                                        module->loadModel(chosen);
                                        free(path);
                                }
                        }
                };

                auto* loadItem = new LoadItem();
                loadItem->text = "Load NAM model";
                loadItem->module = module;
                loadItem->disabled = (module == nullptr);
                menu->addChild(loadItem);

                struct ClearItem : MenuItem {
                        NergalAmp* module = nullptr;
                        void onAction(const event::Action& e) override {
                                if (module)
                                        module->clearModel();
                        }
                };
                auto* clearItem = new ClearItem();
                clearItem->text = "Clear model";
                clearItem->module = module;

                // Thread-safe check if model exists
                bool hasModel = false;
                if (module) {
                        std::lock_guard<std::mutex> lock(module->modelMutex);
                        hasModel = (module->model != nullptr);
                }
                clearItem->disabled = !hasModel;
                menu->addChild(clearItem);

                // Clipper toggle
                menu->addChild(new MenuSeparator());

                struct ClipperItem : MenuItem {
                        NergalAmp* module = nullptr;
                        void onAction(const event::Action& e) override {
                                if (module) {
                                        module->enableClipper = !module->enableClipper;
                                }
                        }
                };
                auto* clipperItem = new ClipperItem();
                clipperItem->text = "Enable soft clipper";
                clipperItem->module = module;
                clipperItem->rightText = CHECKMARK(module && module->enableClipper);
                clipperItem->disabled = (module == nullptr);
                menu->addChild(clipperItem);
        }
};

Model* modelNergalAmp = createModel<NergalAmp, NergalAmpWidget>("NergalAmp");
