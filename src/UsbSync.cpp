#include "plugin.hpp"

#include <ableton/Link.hpp>
#include <RtMidi.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {
constexpr int CLOCKS_PER_QUARTER = 24;  // MIDI Clock is 24 PPQN
constexpr double MAX_PLL_CORRECTION_MS = 0.2;  // ±0.2ms max correction per tick
constexpr int JITTER_HISTORY_SIZE = 96;  // Track last 96 ticks (4 beats at 24 PPQN)

// MIDI Real-Time messages
constexpr uint8_t MIDI_CLOCK = 0xF8;
constexpr uint8_t MIDI_START = 0xFA;
constexpr uint8_t MIDI_CONTINUE = 0xFB;
constexpr uint8_t MIDI_STOP = 0xFC;
constexpr uint8_t MIDI_SPP = 0xF2;

// Jitter statistics tracking
struct JitterStats {
    double avgMs = 0.0;
    double p95Ms = 0.0;
    std::deque<double> history;

    void addSample(double jitterMs) {
        history.push_back(jitterMs);
        if (history.size() > JITTER_HISTORY_SIZE) {
            history.pop_front();
        }

        if (history.empty()) return;

        // Calculate average
        double sum = 0.0;
        for (double j : history) {
            sum += std::abs(j);
        }
        avgMs = sum / history.size();

        // Calculate 95th percentile
        std::vector<double> sorted(history.begin(), history.end());
        for (auto& v : sorted) v = std::abs(v);
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(sorted.size() * 0.95);
        if (idx >= sorted.size()) idx = sorted.size() - 1;
        p95Ms = sorted[idx];
    }

    void clear() {
        history.clear();
        avgMs = 0.0;
        p95Ms = 0.0;
    }
};

}  // namespace

struct UsbSync : Module {
    enum ParamId {
        LINK_ENABLE_PARAM,
        FOLLOW_TRANSPORT_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        INPUTS_LEN
    };
    enum OutputId {
        OUTPUTS_LEN
    };
    enum LightId {
        LINK_ENABLED_LIGHT,
        LINK_CONNECTED_LIGHT,
        TRANSPORT_LIGHT,
        LIGHTS_LEN
    };

    // Link instance
    std::unique_ptr<ableton::Link> link;

    // RtMidi output
    std::unique_ptr<RtMidiOut> midiOut;
    int selectedMidiPort = -1;
    std::vector<std::string> midiPortNames;
    std::mutex midiMutex;

    // Clock thread
    std::unique_ptr<std::thread> clockThread;
    std::atomic<bool> clockThreadRunning{false};
    std::atomic<bool> clockThreadShouldStop{false};

    // Configuration (atomic for thread-safe access)
    std::atomic<bool> linkEnabled{false};
    std::atomic<bool> followTransport{true};
    std::atomic<double> quantum{4.0};  // 4 beats per bar
    std::atomic<double> offsetMs{0.0};  // User-adjustable offset

    // Status (atomic for thread-safe reads from UI)
    std::atomic<double> currentBpm{120.0};
    std::atomic<size_t> numPeers{0};
    std::atomic<bool> isPlaying{false};
    std::atomic<int64_t> tickCount{0};

    // Jitter tracking (protected by mutex)
    std::mutex jitterMutex;
    JitterStats jitterStats;

    UsbSync() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configButton(LINK_ENABLE_PARAM, "Enable Link");
        configButton(FOLLOW_TRANSPORT_PARAM, "Follow Transport Start/Stop");

        // Initialize Link with default 120 BPM
        link = std::make_unique<ableton::Link>(120.0);
        link->enable(false);  // Start disabled

        // Set up Link callbacks
        link->setNumPeersCallback([this](std::size_t peers) {
            numPeers.store(peers);
        });

        link->setTempoCallback([this](double bpm) {
            currentBpm.store(bpm);
        });

        link->setStartStopCallback([this](bool playing) {
            isPlaying.store(playing);
        });

        // Initialize RtMidi
        try {
            midiOut = std::make_unique<RtMidiOut>();
            scanMidiPorts();
        } catch (RtMidiError& error) {
            // Log error but don't crash
        }
    }

    ~UsbSync() {
        stopClockThread();
        if (link) {
            link->enable(false);
        }
        if (midiOut && midiOut->isPortOpen()) {
            midiOut->closePort();
        }
    }

    void onReset() override {
        stopClockThread();
        linkEnabled.store(false);
        followTransport.store(true);
        if (link) {
            link->enable(false);
        }
        std::lock_guard<std::mutex> lock(jitterMutex);
        jitterStats.clear();
    }

    json_t* dataToJson() override {
        json_t* rootJ = Module::dataToJson();
        json_object_set_new(rootJ, "linkEnabled", json_boolean(linkEnabled.load()));
        json_object_set_new(rootJ, "followTransport", json_boolean(followTransport.load()));
        json_object_set_new(rootJ, "quantum", json_real(quantum.load()));
        json_object_set_new(rootJ, "offsetMs", json_real(offsetMs.load()));
        json_object_set_new(rootJ, "selectedMidiPort", json_integer(selectedMidiPort));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        Module::dataFromJson(rootJ);

        json_t* linkEnabledJ = json_object_get(rootJ, "linkEnabled");
        if (linkEnabledJ) {
            bool enabled = json_boolean_value(linkEnabledJ);
            linkEnabled.store(enabled);
            if (link) link->enable(enabled);
            if (enabled) startClockThread();
        }

        json_t* followTransportJ = json_object_get(rootJ, "followTransport");
        if (followTransportJ) {
            followTransport.store(json_boolean_value(followTransportJ));
            if (link) link->enableStartStopSync(followTransport.load());
        }

        json_t* quantumJ = json_object_get(rootJ, "quantum");
        if (quantumJ) {
            quantum.store(json_real_value(quantumJ));
        }

        json_t* offsetMsJ = json_object_get(rootJ, "offsetMs");
        if (offsetMsJ) {
            offsetMs.store(json_real_value(offsetMsJ));
        }

        json_t* portJ = json_object_get(rootJ, "selectedMidiPort");
        if (portJ) {
            int port = json_integer_value(portJ);
            selectMidiPort(port);
        }
    }

    void scanMidiPorts() {
        std::lock_guard<std::mutex> lock(midiMutex);
        midiPortNames.clear();

        if (!midiOut) return;

        unsigned int portCount = midiOut->getPortCount();
        for (unsigned int i = 0; i < portCount; i++) {
            try {
                std::string name = midiOut->getPortName(i);
                midiPortNames.push_back(name);
            } catch (RtMidiError& error) {
                midiPortNames.push_back("(Error reading port)");
            }
        }
    }

    void selectMidiPort(int port) {
        std::lock_guard<std::mutex> lock(midiMutex);

        if (!midiOut) return;

        // Close current port
        if (midiOut->isPortOpen()) {
            midiOut->closePort();
        }

        selectedMidiPort = port;

        // Open new port
        if (port >= 0 && port < (int)midiOut->getPortCount()) {
            try {
                midiOut->openPort(port, "VCV Rack - Link to MIDI Clock");
            } catch (RtMidiError& error) {
                selectedMidiPort = -1;
            }
        }
    }

    void sendMidiMessage(uint8_t status) {
        std::lock_guard<std::mutex> lock(midiMutex);
        if (!midiOut || !midiOut->isPortOpen()) return;

        try {
            std::vector<unsigned char> message = {status};
            midiOut->sendMessage(&message);
        } catch (RtMidiError& error) {
            // Ignore errors during send
        }
    }

    void sendMidiSPP(int position) {
        std::lock_guard<std::mutex> lock(midiMutex);
        if (!midiOut || !midiOut->isPortOpen()) return;

        try {
            // SPP position is in "MIDI beats" (1 MIDI beat = 6 MIDI clocks)
            uint8_t lsb = position & 0x7F;
            uint8_t msb = (position >> 7) & 0x7F;
            std::vector<unsigned char> message = {MIDI_SPP, lsb, msb};
            midiOut->sendMessage(&message);
        } catch (RtMidiError& error) {
            // Ignore errors during send
        }
    }

    void startClockThread() {
        if (clockThreadRunning.load()) return;

        clockThreadShouldStop.store(false);
        clockThreadRunning.store(true);

        clockThread = std::make_unique<std::thread>([this]() {
            this->clockThreadFunc();
        });
    }

    void stopClockThread() {
        if (!clockThreadRunning.load()) return;

        clockThreadShouldStop.store(true);
        if (clockThread && clockThread->joinable()) {
            clockThread->join();
        }
        clockThreadRunning.store(false);
    }

    void clockThreadFunc() {
        // Set thread priority (platform-specific)
#ifdef _WIN32
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#elif defined(__APPLE__)
        // macOS: set realtime priority
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#elif defined(__linux__)
        // Linux: set realtime priority
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#endif

        using Clock = std::chrono::steady_clock;
        using TimePoint = std::chrono::time_point<Clock>;
        using Micros = std::chrono::microseconds;

        bool wasPlaying = false;
        int64_t ticksSinceStart = 0;

        // PLL state for phase correction
        double pllError = 0.0;

        while (!clockThreadShouldStop.load()) {
            if (!link || !linkEnabled.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Capture Link session state
            auto sessionState = link->captureAppSessionState();
            double tempo = sessionState.tempo();
            bool playing = followTransport.load() ? sessionState.isPlaying() : true;

            currentBpm.store(tempo);
            isPlaying.store(playing);

            // Handle transport changes
            if (playing && !wasPlaying) {
                // Just started playing
                sendMidiMessage(MIDI_START);
                ticksSinceStart = 0;
                pllError = 0.0;

                std::lock_guard<std::mutex> lock(jitterMutex);
                jitterStats.clear();
            } else if (!playing && wasPlaying) {
                // Just stopped playing
                sendMidiMessage(MIDI_STOP);
            }
            wasPlaying = playing;

            if (!playing) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Calculate tick interval in microseconds
            // tempo is in BPM, we need 24 ticks per beat
            double tickIntervalUs = (60.0 * 1000000.0) / (tempo * CLOCKS_PER_QUARTER);

            // Get current Link time
            auto linkTime = link->clock().micros();

            // Get current phase within the quantum
            double phase = sessionState.phaseAtTime(linkTime, quantum.load());
            double beat = sessionState.beatAtTime(linkTime, quantum.load());

            // Calculate what tick we should be at based on Link phase
            double idealTick = beat * CLOCKS_PER_QUARTER;

            // Calculate next tick time
            double nextTick = std::floor(idealTick) + 1.0;
            double beatsUntilNextTick = (nextTick / CLOCKS_PER_QUARTER) - beat;
            double usUntilNextTick = beatsUntilNextTick * (60.0 * 1000000.0) / tempo;

            // Apply user offset
            usUntilNextTick += offsetMs.load() * 1000.0;

            // Apply PLL correction (clamp to max correction)
            double correctionUs = std::clamp(pllError,
                                            -MAX_PLL_CORRECTION_MS * 1000.0,
                                            MAX_PLL_CORRECTION_MS * 1000.0);
            usUntilNextTick += correctionUs;

            if (usUntilNextTick > 0) {
                // Sleep until next tick
                auto wakeTime = Clock::now() + Micros(static_cast<int64_t>(usUntilNextTick));
                std::this_thread::sleep_until(wakeTime);

                // Send MIDI clock
                sendMidiMessage(MIDI_CLOCK);
                tickCount.fetch_add(1);
                ticksSinceStart++;

                // Update PLL error for next iteration
                auto actualTime = link->clock().micros();
                auto sessionState2 = link->captureAppSessionState();
                double actualBeat = sessionState2.beatAtTime(actualTime, quantum.load());
                double expectedTick = nextTick;
                double actualTick = actualBeat * CLOCKS_PER_QUARTER;
                double tickError = actualTick - expectedTick;
                double timeErrorUs = tickError * tickIntervalUs;

                pllError = -timeErrorUs * 0.1;  // Proportional gain of 0.1

                // Track jitter
                std::lock_guard<std::mutex> lock(jitterMutex);
                jitterStats.addSample(timeErrorUs / 1000.0);  // Convert to ms
            } else {
                // We're behind schedule, send immediately
                sendMidiMessage(MIDI_CLOCK);
                tickCount.fetch_add(1);
                ticksSinceStart++;
            }
        }
    }

    void process(const ProcessArgs& args) override {
        // Handle Link enable button
        if (params[LINK_ENABLE_PARAM].getValue() > 0.5f) {
            bool enabled = !linkEnabled.load();
            linkEnabled.store(enabled);
            if (link) {
                link->enable(enabled);
            }

            if (enabled) {
                startClockThread();
            } else {
                stopClockThread();
            }

            params[LINK_ENABLE_PARAM].setValue(0.f);
        }

        // Handle transport follow button
        if (params[FOLLOW_TRANSPORT_PARAM].getValue() > 0.5f) {
            bool follow = !followTransport.load();
            followTransport.store(follow);
            if (link) {
                link->enableStartStopSync(follow);
            }
            params[FOLLOW_TRANSPORT_PARAM].setValue(0.f);
        }

        // Update lights
        lights[LINK_ENABLED_LIGHT].setBrightness(linkEnabled.load() ? 1.f : 0.f);
        lights[LINK_CONNECTED_LIGHT].setBrightness(numPeers.load() > 0 ? 1.f : 0.f);
        lights[TRANSPORT_LIGHT].setBrightness(isPlaying.load() ? 1.f : 0.f);
    }

    // Accessors for UI
    std::vector<std::string> getMidiPortNames() {
        std::lock_guard<std::mutex> lock(midiMutex);
        return midiPortNames;
    }

    int getSelectedMidiPort() const {
        return selectedMidiPort;
    }

    JitterStats getJitterStats() {
        std::lock_guard<std::mutex> lock(jitterMutex);
        return jitterStats;
    }
};

// UI Widgets

struct MidiPortDisplay : LedDisplay {
    UsbSync* module = nullptr;

    void draw(const DrawArgs& args) override {
        nvgScissor(args.vg, RECT_ARGS(args.clipBox));

        // Background
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 3.0);
        nvgFillColor(args.vg, nvgRGB(0, 0, 0));
        nvgFill(args.vg);

        // Border
        nvgStrokeWidth(args.vg, 1.0);
        nvgStrokeColor(args.vg, nvgRGB(80, 80, 80));
        nvgStroke(args.vg);

        // Text
        std::shared_ptr<window::Font> font = APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        if (font && font->handle >= 0) {
            nvgFillColor(args.vg, nvgRGB(255, 255, 255));
            nvgFontFaceId(args.vg, font->handle);
            nvgTextLetterSpacing(args.vg, 0.0);
            nvgFontSize(args.vg, 11);
            nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

            std::string text = "(No MIDI device)";
            if (module) {
                int port = module->getSelectedMidiPort();
                if (port >= 0) {
                    auto names = module->getMidiPortNames();
                    if (port < (int)names.size()) {
                        text = names[port];
                    }
                }
            }
            nvgText(args.vg, box.size.x / 2, box.size.y / 2, text.c_str(), NULL);
        }

        nvgResetScissor(args.vg);
    }

    void onButton(const event::Button& e) override {
        if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            if (!module) return;

            ui::Menu* menu = createMenu();
            menu->addChild(createMenuLabel("MIDI Output Port"));

            module->scanMidiPorts();
            auto portNames = module->getMidiPortNames();

            for (size_t i = 0; i < portNames.size(); i++) {
                struct PortItem : MenuItem {
                    UsbSync* module;
                    int port;
                    void onAction(const event::Action& e) override {
                        module->selectMidiPort(port);
                    }
                };

                PortItem* item = new PortItem;
                item->text = portNames[i];
                item->module = module;
                item->port = i;
                item->rightText = CHECKMARK(module->getSelectedMidiPort() == (int)i);
                menu->addChild(item);
            }

            e.consume(this);
        }
    }
};

struct LinkStatusDisplay : TransparentWidget {
    UsbSync* module = nullptr;
    std::shared_ptr<window::Font> font;

    LinkStatusDisplay() {
        box.size = Vec(140.f, 50.f);
    }

    void draw(const DrawArgs& args) override {
        nvgSave(args.vg);
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0.f, 0.f, box.size.x, box.size.y, 4.f);
        nvgFillColor(args.vg, nvgRGBA(10, 10, 10, 200));
        nvgFill(args.vg);
        nvgStrokeColor(args.vg, nvgRGBA(60, 60, 60, 255));
        nvgStrokeWidth(args.vg, 1.0f);
        nvgStroke(args.vg);
        nvgRestore(args.vg);

        if (!module) return;

        if (!font) {
            font = APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        }
        if (!font) return;

        nvgSave(args.vg);
        nvgFontSize(args.vg, 12.f);
        nvgFontFaceId(args.vg, font->handle);
        nvgFillColor(args.vg, nvgRGBA(230, 230, 230, 0xff));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);

        // Line 1: BPM and peers
        double bpm = module->currentBpm.load();
        size_t peers = module->numPeers.load();
        std::string line1 = string::f("%.1f BPM | %zu peer%s", bpm, peers, peers == 1 ? "" : "s");
        nvgText(args.vg, box.size.x * 0.5f, 5.f, line1.c_str(), nullptr);

        // Line 2: Ticks
        int64_t ticks = module->tickCount.load();
        std::string line2 = string::f("Ticks: %lld", (long long)ticks);
        nvgText(args.vg, box.size.x * 0.5f, 20.f, line2.c_str(), nullptr);

        // Line 3: Jitter stats
        auto jitter = module->getJitterStats();
        std::string line3 = string::f("Jitter: %.3f / %.3f ms", jitter.avgMs, jitter.p95Ms);
        nvgText(args.vg, box.size.x * 0.5f, 35.f, line3.c_str(), nullptr);

        nvgRestore(args.vg);
    }
};

struct UsbSyncWidget : ModuleWidget {
    MidiPortDisplay* midiPortDisplay = nullptr;
    LinkStatusDisplay* linkStatusDisplay = nullptr;

    UsbSyncWidget(UsbSync* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/UsbSync.svg")));

        const float panelWidth = 40.64f;

        // MIDI port selector
        midiPortDisplay = createWidget<MidiPortDisplay>(mm2px(Vec(2.5f, 12.f)));
        midiPortDisplay->box.size = mm2px(Vec(panelWidth - 5.f, 7.f));
        midiPortDisplay->module = module;
        addChild(midiPortDisplay);

        // Link status display
        linkStatusDisplay = createWidget<LinkStatusDisplay>(mm2px(Vec(2.5f, 25.f)));
        linkStatusDisplay->box.size = mm2px(Vec(panelWidth - 5.f, 20.f));
        linkStatusDisplay->module = module;
        addChild(linkStatusDisplay);

        // Status lights
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(10.16f, 55.f)), module, UsbSync::LINK_ENABLED_LIGHT));
        addChild(createLightCentered<MediumLight<BlueLight>>(mm2px(Vec(panelWidth / 2.f, 55.f)), module, UsbSync::LINK_CONNECTED_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(panelWidth - 10.16f, 55.f)), module, UsbSync::TRANSPORT_LIGHT));

        // Control buttons
        addParam(createLightParamCentered<VCVLightButton<MediumSimpleLight<GreenLight>>>(
            mm2px(Vec(panelWidth / 2.f - 7.f, 70.f)),
            module,
            UsbSync::LINK_ENABLE_PARAM,
            UsbSync::LINK_ENABLED_LIGHT));

        addParam(createLightParamCentered<VCVLightButton<MediumSimpleLight<YellowLight>>>(
            mm2px(Vec(panelWidth / 2.f + 7.f, 70.f)),
            module,
            UsbSync::FOLLOW_TRANSPORT_PARAM,
            UsbSync::TRANSPORT_LIGHT));
    }

    void appendContextMenu(Menu* menu) override {
        UsbSync* usbSync = dynamic_cast<UsbSync*>(module);
        if (!usbSync) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Link Settings"));

        // Quantum selector
        struct QuantumItem : MenuItem {
            UsbSync* module;
            double quantum;
            void onAction(const event::Action& e) override {
                module->quantum.store(quantum);
            }
        };

        menu->addChild(createMenuLabel("Quantum (beats per bar)"));
        for (double q : {1.0, 2.0, 4.0, 8.0, 16.0}) {
            QuantumItem* item = new QuantumItem;
            item->text = string::f("%.0f beats", q);
            item->module = usbSync;
            item->quantum = q;
            item->rightText = CHECKMARK(std::abs(usbSync->quantum.load() - q) < 0.01);
            menu->addChild(item);
        }

        // Offset slider
        menu->addChild(createMenuLabel("Offset"));
        struct OffsetSlider : ui::Slider {
            UsbSync* module;
            void onAction(const event::Action& e) override {
                if (module) {
                    module->offsetMs.store(quantity->getValue());
                }
            }
        };

        struct OffsetQuantity : Quantity {
            UsbSync* module;
            OffsetQuantity(UsbSync* m) : module(m) {}
            void setValue(float value) override {
                if (module) module->offsetMs.store(value);
            }
            float getValue() override {
                return module ? module->offsetMs.load() : 0.f;
            }
            float getMinValue() override { return -10.f; }
            float getMaxValue() override { return 10.f; }
            float getDefaultValue() override { return 0.f; }
            std::string getLabel() override { return "Offset"; }
            std::string getUnit() override { return " ms"; }
            int getDisplayPrecision() override { return 2; }
        };

        OffsetSlider* slider = new OffsetSlider;
        slider->module = usbSync;
        slider->quantity = new OffsetQuantity(usbSync);
        slider->box.size.x = 200.f;
        menu->addChild(slider);
    }

    void step() override {
        ModuleWidget::step();
        if (midiPortDisplay) {
            midiPortDisplay->module = static_cast<UsbSync*>(module);
        }
        if (linkStatusDisplay) {
            linkStatusDisplay->module = static_cast<UsbSync*>(module);
        }
    }
};

Model* modelUsbSync = createModel<UsbSync, UsbSyncWidget>("UsbSync");
