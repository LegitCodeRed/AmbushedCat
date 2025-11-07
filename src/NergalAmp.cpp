#include "plugin.hpp"
#include <osdialog.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

// Use Windows native threading to avoid pthread dependency
#ifdef ARCH_WIN
#include <windows.h>
#else
#include <condition_variable>
#include <mutex>
#include <thread>
#endif

#include "NeuralAmpModelerCore/NAM/dsp.h"
#include "NeuralAmpModelerCore/NAM/get_dsp.h"

// Lock-free ring buffer for audio samples
template<typename T, size_t SIZE>
struct RingBuffer {
        std::array<T, SIZE> buffer;
        std::atomic<size_t> writePos{0};
        std::atomic<size_t> readPos{0};

        inline bool push(T value) {
                size_t currentWrite = writePos.load(std::memory_order_relaxed);
                size_t nextWrite = (currentWrite + 1) % SIZE;
                if (nextWrite == readPos.load(std::memory_order_acquire)) {
                        return false; // Buffer full
                }
                buffer[currentWrite] = value;
                writePos.store(nextWrite, std::memory_order_release);
                return true;
        }

        inline bool pop(T& value) {
                size_t currentRead = readPos.load(std::memory_order_relaxed);
                if (currentRead == writePos.load(std::memory_order_acquire)) {
                        return false; // Buffer empty
                }
                value = buffer[currentRead];
                readPos.store((currentRead + 1) % SIZE, std::memory_order_release);
                return true;
        }

        inline size_t size() const {
                size_t w = writePos.load(std::memory_order_acquire);
                size_t r = readPos.load(std::memory_order_acquire);
                return (w >= r) ? (w - r) : (SIZE - r + w);
        }

        inline void clear() {
                readPos.store(0, std::memory_order_release);
                writePos.store(0, std::memory_order_release);
        }
};

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
                SIGNAL_INPUT_L,
                SIGNAL_INPUT_R,
                DRIVE_CV_INPUT,
                TONE_CV_INPUT,
                INPUT_CV_INPUT,
                OUTPUT_CV_INPUT,
                MIX_CV_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                SIGNAL_OUTPUT_L,
                SIGNAL_OUTPUT_R,
                NUM_OUTPUTS
        };
        enum LightIds {
                LOADED_LIGHT,
                NUM_LIGHTS
        };

        // Worker thread for NAM processing
#ifdef ARCH_WIN
        HANDLE workerThread = NULL;
        HANDLE workerEvent = NULL;  // Event for waking worker
        CRITICAL_SECTION modelMutex;  // Lightweight mutex for model access
#else
        std::thread workerThread;
        std::mutex workerMutex;
        std::condition_variable workerCV;
#endif
        std::atomic<bool> workerRunning{false};

        // Lock-free ring buffers for stereo (8192 samples = ~170ms @ 48kHz)
        RingBuffer<float, 8192> inputBufferL;
        RingBuffer<float, 8192> outputBufferL;
        RingBuffer<float, 8192> inputBufferR;
        RingBuffer<float, 8192> outputBufferR;

        // Model state (protected by mutex on non-Windows)
        std::unique_ptr<nam::DSP> model;
        std::string modelPath;
        double modelSampleRate = 48000.0;
        std::atomic<bool> modelReady{false};

        // Audio thread state (no mutex needed) - stereo
        ToneShaper toneL;
        ToneShaper toneR;
        float previousModelInputL = 0.0f;
        float previousModelInputR = 0.0f;
        bool firstFrame = true;

        // Deferred loading
        std::string pendingModelPath;
        std::atomic<bool> loadPending{false};

        // Settings
        std::atomic<bool> enableClipper{true};

        // UI state
        std::string lastDirectory;

        NergalAmp() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(INPUT_PARAM, -24.f, 24.f, 0.f, "Input Trim", " dB");
                configParam(DRIVE_PARAM, 0.f, 2.f, 1.f, "Drive/Gain", "x");
                configParam(TONE_PARAM, -1.f, 1.f, 0.f, "Tone");
                configParam(MIX_PARAM, 0.f, 1.f, 1.f, "Dry/Wet Mix", "%", 0.f, 100.f);
                configParam(OUTPUT_PARAM, -24.f, 24.f, 0.f, "Output Level", " dB");

                configInput(SIGNAL_INPUT_L, "Audio L");
                configInput(SIGNAL_INPUT_R, "Audio R");
                configInput(INPUT_CV_INPUT, "Input Trim CV");
                configInput(DRIVE_CV_INPUT, "Drive CV");
                configInput(TONE_CV_INPUT, "Tone CV");
                configInput(MIX_CV_INPUT, "Mix CV");
                configInput(OUTPUT_CV_INPUT, "Output Level CV");

                configOutput(SIGNAL_OUTPUT_L, "Audio L");
                configOutput(SIGNAL_OUTPUT_R, "Audio R");

                // Start worker thread
                workerRunning = true;
#ifdef ARCH_WIN
                InitializeCriticalSection(&modelMutex);
                workerEvent = CreateEvent(NULL, FALSE, FALSE, NULL);  // Auto-reset event
                workerThread = CreateThread(NULL, 0, workerThreadFuncWin, this, 0, NULL);
#else
                workerThread = std::thread(&NergalAmp::workerThreadFunc, this);
#endif

                onSampleRateChange();
        }

        ~NergalAmp() {
                // Stop worker thread
                workerRunning = false;
#ifdef ARCH_WIN
                if (workerEvent) {
                        SetEvent(workerEvent);  // Wake worker
                }
                if (workerThread) {
                        WaitForSingleObject(workerThread, INFINITE);
                        CloseHandle(workerThread);
                }
                if (workerEvent) {
                        CloseHandle(workerEvent);
                }
                DeleteCriticalSection(&modelMutex);
#else
                workerCV.notify_one();
                if (workerThread.joinable()) {
                        workerThread.join();
                }
#endif
        }

#ifdef ARCH_WIN
        // Windows thread entry point (must be static)
        static DWORD WINAPI workerThreadFuncWin(LPVOID param) {
                NergalAmp* module = static_cast<NergalAmp*>(param);
                module->workerThreadFunc();
                return 0;
        }
#endif

        // Worker thread function - processes NAM model in background (stereo)
        void workerThreadFunc() {
                while (workerRunning) {
                        float inputL, inputR;
                        bool hasWorkL = inputBufferL.pop(inputL);
                        bool hasWorkR = inputBufferR.pop(inputR);

                        if ((hasWorkL || hasWorkR) && modelReady.load(std::memory_order_acquire)) {
                                // Process L channel through NAM model
                                if (hasWorkL) {
                                        NAM_SAMPLE inputFrame = (NAM_SAMPLE)inputL;
                                        NAM_SAMPLE outputFrame = 0.0;

                                        // Lock only for actual model processing
#ifdef ARCH_WIN
                                        EnterCriticalSection(&modelMutex);
                                        if (model) {
                                                model->process(&inputFrame, &outputFrame, 1);
                                        }
                                        LeaveCriticalSection(&modelMutex);
#else
                                        {
                                                std::lock_guard<std::mutex> lock(workerMutex);
                                                if (model) {
                                                        model->process(&inputFrame, &outputFrame, 1);
                                                }
                                        }
#endif

                                        // Push result to output buffer (lock-free)
                                        while (!outputBufferL.push((float)outputFrame)) {
                                                // Output buffer full, yield CPU
#ifdef ARCH_WIN
                                                Sleep(0);
#else
                                                std::this_thread::yield();
#endif
                                        }
                                }

                                // Process R channel through NAM model
                                if (hasWorkR) {
                                        NAM_SAMPLE inputFrame = (NAM_SAMPLE)inputR;
                                        NAM_SAMPLE outputFrame = 0.0;

                                        // Lock only for actual model processing
#ifdef ARCH_WIN
                                        EnterCriticalSection(&modelMutex);
                                        if (model) {
                                                model->process(&inputFrame, &outputFrame, 1);
                                        }
                                        LeaveCriticalSection(&modelMutex);
#else
                                        {
                                                std::lock_guard<std::mutex> lock(workerMutex);
                                                if (model) {
                                                        model->process(&inputFrame, &outputFrame, 1);
                                                }
                                        }
#endif

                                        // Push result to output buffer (lock-free)
                                        while (!outputBufferR.push((float)outputFrame)) {
                                                // Output buffer full, yield CPU
#ifdef ARCH_WIN
                                                Sleep(0);
#else
                                                std::this_thread::yield();
#endif
                                        }
                                }
                        }
                        else {
                                // No work, wait for notification or timeout
#ifdef ARCH_WIN
                                WaitForSingleObject(workerEvent, 1);  // 1ms timeout
#else
                                std::unique_lock<std::mutex> lock(workerMutex);
                                workerCV.wait_for(lock, std::chrono::microseconds(100));
#endif
                        }
                }
        }

        void onSampleRateChange() override {
                Module::onSampleRateChange();
#ifdef ARCH_WIN
                EnterCriticalSection(&modelMutex);
#else
                std::lock_guard<std::mutex> lock(workerMutex);
#endif
                firstFrame = true;
                toneL.reset();
                toneR.reset();
                inputBufferL.clear();
                outputBufferL.clear();
                inputBufferR.clear();
                outputBufferR.clear();
                if (model) {
                        const double effectiveRate = modelSampleRate > 0.0 ? modelSampleRate : APP->engine->getSampleRate();
                        model->Reset(effectiveRate, 64);
                }
#ifdef ARCH_WIN
                LeaveCriticalSection(&modelMutex);
#endif
        }

        void process(const ProcessArgs& args) override {
                // Check for pending model load
                if (loadPending.load(std::memory_order_acquire)) {
                        loadPending.store(false, std::memory_order_release);
                        loadModelInternal(pendingModelPath);
                }

                // Determine stereo/mono mode
                bool leftConnected = inputs[SIGNAL_INPUT_L].isConnected();
                bool rightConnected = inputs[SIGNAL_INPUT_R].isConnected();

                // Get input signals and normalize from ±10V to ±1 range
                float inVoltsL = leftConnected ? inputs[SIGNAL_INPUT_L].getVoltage() :
                                (rightConnected ? inputs[SIGNAL_INPUT_R].getVoltage() : 0.f);
                float inVoltsR = rightConnected ? inputs[SIGNAL_INPUT_R].getVoltage() :
                                (leftConnected ? inputs[SIGNAL_INPUT_L].getVoltage() : 0.f);

                const float drySignalL = inVoltsL * 0.1f;
                const float drySignalR = inVoltsR * 0.1f;

                // Get parameter values with CV modulation
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
                const float trimmedL = drySignalL * inputGain;
                const float trimmedR = drySignalR * inputGain;

                // Stage 2: Worker thread NAM processing (lock-free stereo)
                float modelOutputL = trimmedL;
                float modelOutputR = trimmedR;
                bool hasModel = modelReady.load(std::memory_order_acquire);

                if (hasModel) {
                        // Push both channels to worker thread (non-blocking)
                        bool pushedL = inputBufferL.push(trimmedL);
                        bool pushedR = inputBufferR.push(trimmedR);

                        if (pushedL || pushedR) {
#ifdef ARCH_WIN
                                SetEvent(workerEvent);  // Wake worker
#else
                                workerCV.notify_one();
#endif
                        }

                        // Try to get processed outputs (non-blocking)
                        float workerOutputL, workerOutputR;
                        if (outputBufferL.pop(workerOutputL)) {
                                modelOutputL = workerOutputL;
                        }
                        if (outputBufferR.pop(workerOutputR)) {
                                modelOutputR = workerOutputR;
                        }
                        // else: use previous output (smooth latency handling)
                }

                // Stage 3: Apply drive/saturation
                float drivenL = modelOutputL * driveAmount;
                float drivenR = modelOutputR * driveAmount;
                if (enableClipper.load(std::memory_order_relaxed) && driveAmount > 1.0f) {
                        drivenL = softClip(drivenL);
                        drivenR = softClip(drivenR);
                }

                // Stage 4: Tone shaping (optimized with caching)
                float shapedL = toneL.process(drivenL, toneAmount, args.sampleRate);
                float shapedR = toneR.process(drivenR, toneAmount, args.sampleRate);

                // Stage 5: Dry/Wet mix
                float mixedL = drySignalL + mixAmount * (shapedL - drySignalL);
                float mixedR = drySignalR + mixAmount * (shapedR - drySignalR);

                // Stage 6: Output level and convert back to ±10V
                outputs[SIGNAL_OUTPUT_L].setVoltage(mixedL * outputGain * 10.f);
                outputs[SIGNAL_OUTPUT_R].setVoltage(mixedR * outputGain * 10.f);

                lights[LOADED_LIGHT].setBrightness(hasModel ? 1.f : 0.f);
        }

        void clearModel() {
                modelReady.store(false, std::memory_order_release);
                inputBufferL.clear();
                outputBufferL.clear();
                inputBufferR.clear();
                outputBufferR.clear();

#ifdef ARCH_WIN
                EnterCriticalSection(&modelMutex);
#else
                std::lock_guard<std::mutex> lock(workerMutex);
#endif
                model.reset();
                modelPath.clear();
                modelSampleRate = 48000.0;
                firstFrame = true;
                toneL.reset();
                toneR.reset();
#ifdef ARCH_WIN
                LeaveCriticalSection(&modelMutex);
#endif
        }

        // Internal loading - called from audio thread with path already validated
        void loadModelInternal(const std::string& path) {
                if (path.empty()) {
                        clearModel();
                        return;
                }

                try {
                        std::filesystem::path fsPath(path);

                        // Load the model (this is the slow part - not blocking audio thread)
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

                        // Clear buffers before swapping model
                        inputBufferL.clear();
                        outputBufferL.clear();
                        inputBufferR.clear();
                        outputBufferR.clear();

                        // Quick atomic swap with worker mutex
#ifdef ARCH_WIN
                        EnterCriticalSection(&modelMutex);
#else
                        {
                                std::lock_guard<std::mutex> lock(workerMutex);
#endif
                                model = std::move(loaded);
                                modelPath = path;
                                modelSampleRate = sampleRate;
                                firstFrame = true;
                                toneL.reset();
                                toneR.reset();
#ifdef ARCH_WIN
                        LeaveCriticalSection(&modelMutex);
#else
                        }
#endif

                        // Activate model atomically
                        modelReady.store(true, std::memory_order_release);
#ifdef ARCH_WIN
                        SetEvent(workerEvent);  // Wake worker thread
#else
                        workerCV.notify_one();
#endif

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
                loadPending.store(true, std::memory_order_release);
        }

        json_t* dataToJson() override {
                json_t* root = json_object();

                // Copy strings with mutex protection
                std::string path, dir;
#ifdef ARCH_WIN
                EnterCriticalSection(&modelMutex);
                path = modelPath;
                LeaveCriticalSection(&modelMutex);
#else
                {
                        std::lock_guard<std::mutex> lock(workerMutex);
                        path = modelPath;
                }
#endif
                dir = lastDirectory; // lastDirectory doesn't need mutex (UI only)

                if (!path.empty()) {
                        json_object_set_new(root, "modelPath", json_string(path.c_str()));
                }
                if (!dir.empty()) {
                        json_object_set_new(root, "lastDirectory", json_string(dir.c_str()));
                }
                json_object_set_new(root, "enableClipper", json_boolean(enableClipper.load()));
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
                        enableClipper.store(json_boolean_value(clipperJ));
                }
        }
};

struct BackgroundImage : Widget {
	std::string imagePath = asset::plugin(pluginInstance, "res/TextureDemonMainV2.png");
	widget::SvgWidget* svgWidget;

	BackgroundImage() {
		// Create SVG widget as a child
		svgWidget = new widget::SvgWidget();
		svgWidget->setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/NergalAmp.svg")));
		addChild(svgWidget);
	}

	void draw(const DrawArgs& args) override {
		// Draw background image first
		std::shared_ptr<Image> image = APP->window->loadImage(imagePath);
		if (image) {
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

struct NergalAmpWidget : ModuleWidget {
        NergalAmpWidget(NergalAmp* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/NergalAmp.svg")));

                auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size;
		addChild(bg);

                // Helpers
                constexpr float HP = 5.08f;                // 1 HP in mm
                constexpr float PANEL_HP = 8.0f;           // 8 HP module
                constexpr float W = PANEL_HP * HP;         // 40.64 mm

                // A tidy grid
                constexpr float SIDE_MARGIN = 8.0f;       // left/right margin in mm
                constexpr float KNOB_X = SIDE_MARGIN;      // left column (knobs)
                constexpr float CV_X   = W - SIDE_MARGIN - 8.0f;  // right column (CV)
                constexpr float TOP_LED_Y = 18.375f+2.5f;         // LED near top
                constexpr float FIRST_ROW_Y = 30.0f;       // first control row
                constexpr float ROW_STEP = 15.0f;          // vertical spacing

                // Screws (unchanged, use Rack constants)
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                // Model loaded indicator — centered at top
                addChild(createLightCentered<MediumLight<GreenLight>>(
                mm2px(Vec(HP*3, TOP_LED_Y)), module, NergalAmp::LOADED_LIGHT));

                // Parameters (left column — large knobs)
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(KNOB_X, FIRST_ROW_Y + 0 * ROW_STEP)), module, NergalAmp::INPUT_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(KNOB_X, FIRST_ROW_Y + 1 * ROW_STEP)), module, NergalAmp::DRIVE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(KNOB_X, FIRST_ROW_Y + 2 * ROW_STEP)), module, NergalAmp::TONE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(KNOB_X, FIRST_ROW_Y + 3 * ROW_STEP)), module, NergalAmp::MIX_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(KNOB_X, FIRST_ROW_Y + 4 * ROW_STEP)), module, NergalAmp::OUTPUT_PARAM));

                // CV Inputs (right column — aligned by row)
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(CV_X, FIRST_ROW_Y + 0 * ROW_STEP)), module, NergalAmp::INPUT_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(CV_X, FIRST_ROW_Y + 1 * ROW_STEP)), module, NergalAmp::DRIVE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(CV_X, FIRST_ROW_Y + 2 * ROW_STEP)), module, NergalAmp::TONE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(CV_X, FIRST_ROW_Y + 3 * ROW_STEP)), module, NergalAmp::MIX_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(CV_X, FIRST_ROW_Y + 4 * ROW_STEP)), module, NergalAmp::OUTPUT_CV_INPUT));

                // Audio I/O — stereo at bottom (2 inputs left, 2 outputs right)
                constexpr float AUDIO_Y_TOP = 107.0f;      // top row of audio I/O
                constexpr float AUDIO_Y_BOTTOM = 116.0f;   // bottom row of audio I/O
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(SIDE_MARGIN, AUDIO_Y_TOP)), module, NergalAmp::SIGNAL_INPUT_L));     // L input
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(SIDE_MARGIN, AUDIO_Y_BOTTOM)), module, NergalAmp::SIGNAL_INPUT_R));  // R input
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(W - SIDE_MARGIN - 8.0f, AUDIO_Y_TOP)), module, NergalAmp::SIGNAL_OUTPUT_L));    // L output
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(W - SIDE_MARGIN - 8.0f, AUDIO_Y_BOTTOM)), module, NergalAmp::SIGNAL_OUTPUT_R)); // R output

        }

        void appendContextMenu(Menu* menu) override {
                ModuleWidget::appendContextMenu(menu);

                menu->addChild(new MenuSeparator());

                auto* module = getModule<NergalAmp>();

                std::string labelText = "Model: (none)";
                if (module) {
                        // Thread-safe copy of model path
                        std::string path;
#ifdef ARCH_WIN
                        EnterCriticalSection(&module->modelMutex);
                        path = module->modelPath;
                        LeaveCriticalSection(&module->modelMutex);
#else
                        {
                                std::lock_guard<std::mutex> lock(module->workerMutex);
                                path = module->modelPath;
                        }
#endif
                        if (!path.empty()) {
                                // Extract filename without std::filesystem for macOS <10.15 compatibility
                                size_t lastSlash = path.find_last_of("/\\");
                                std::string filename = (lastSlash != std::string::npos) ? path.substr(lastSlash + 1) : path;
                                labelText = "Model: " + filename;
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
                bool hasModel = module ? module->modelReady.load(std::memory_order_acquire) : false;
                clearItem->disabled = !hasModel;
                menu->addChild(clearItem);

                // Clipper toggle
                menu->addChild(new MenuSeparator());

                struct ClipperItem : MenuItem {
                        NergalAmp* module = nullptr;
                        void onAction(const event::Action& e) override {
                                if (module) {
                                        bool current = module->enableClipper.load(std::memory_order_relaxed);
                                        module->enableClipper.store(!current, std::memory_order_relaxed);
                                }
                        }
                };
                auto* clipperItem = new ClipperItem();
                clipperItem->text = "Enable soft clipper";
                clipperItem->module = module;
                clipperItem->rightText = CHECKMARK(module && module->enableClipper.load(std::memory_order_relaxed));
                clipperItem->disabled = (module == nullptr);
                menu->addChild(clipperItem);
        }
};

Model* modelNergalAmp = createModel<NergalAmp, NergalAmpWidget>("NergalAmp");
