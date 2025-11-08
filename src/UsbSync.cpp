#include "plugin.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>

namespace {
constexpr float CLOCK_VOLTAGE = 10.f;
constexpr float GATE_VOLTAGE = 10.f;
constexpr float RESET_VOLTAGE = 10.f;
constexpr float RESET_PULSE_MS = 1.f;
constexpr int CLOCKS_PER_QUARTER = 24;
constexpr int CLOCKS_PER_BAR = CLOCKS_PER_QUARTER * 4;  // Assume 4/4
constexpr int CLOCKS_PER_SPP_UNIT = 6;                  // 24 PPQN / 4

struct PulseEvent {
    enum class Type {
        Clock,
        Reset,
        Run,
    };

    Type type = Type::Clock;
    int64_t frame = 0;
    bool runState = false;
};
}  // namespace

struct UsbSync : Module {
    enum ParamId {
        PARAMS_LEN
    };
    enum OutputId {
        CLK_OUTPUT,
        RUN_OUTPUT,
        RESET_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LOCK_LIGHT,
        RUN_LIGHT,
        LIGHTS_LEN
    };

    midi::InputQueue midiInput;

    bool running = false;
    bool locked = false;
    int lockCounter = 0;
    int64_t lastClockFrame = -1;
    double periodSamples = 0.0;
    bool havePeriod = false;
    double bpm = 0.0;
    int clockPulseRemain = 0;
    int resetPulseRemain = 0;
    int resetPulseLength = 1;
    bool runGate = false;

    std::deque<PulseEvent> eventQueue;

    UsbSync() {
        config(PARAMS_LEN, 0, OUTPUTS_LEN, LIGHTS_LEN);
        configOutput(CLK_OUTPUT, "Clock (24 PPQN)");
        configOutput(RUN_OUTPUT, "Run gate");
        configOutput(RESET_OUTPUT, "Reset trigger");
    }

    void onSampleRateChange() override {
        updateResetPulseLength(APP->engine->getSampleRate());
    }

    void onReset() override {
        midiInput.reset();
        running = false;
        runGate = false;
        locked = false;
        lockCounter = 0;
        lastClockFrame = -1;
        havePeriod = false;
        periodSamples = 0.0;
        bpm = 0.0;
        clockPulseRemain = 0;
        resetPulseRemain = 0;
        eventQueue.clear();
    }

    json_t* dataToJson() override {
        json_t* rootJ = Module::dataToJson();
        json_object_set_new(rootJ, "midiInput", midiInput.toJson());
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        Module::dataFromJson(rootJ);
        json_t* midiJ = json_object_get(rootJ, "midiInput");
        if (midiJ) {
            midiInput.fromJson(midiJ);
        }
    }

    double sampleRate() const {
        return APP ? APP->engine->getSampleRate() : 44100.0;
    }

    void updateResetPulseLength(double sr) {
        resetPulseLength = std::max(1, (int)std::lround(sr * (RESET_PULSE_MS / 1000.0)));
    }

    void scheduleEvent(const PulseEvent& event) {
        auto it = std::upper_bound(
            eventQueue.begin(), eventQueue.end(), event,
            [](const PulseEvent& a, const PulseEvent& b) { return a.frame < b.frame; });
        eventQueue.insert(it, event);
    }

    void scheduleClockPulse(int64_t frame) {
        PulseEvent ev;
        ev.type = PulseEvent::Type::Clock;
        ev.frame = frame;
        scheduleEvent(ev);
    }

    void scheduleResetPulse(int64_t frame) {
        PulseEvent ev;
        ev.type = PulseEvent::Type::Reset;
        ev.frame = frame;
        scheduleEvent(ev);
    }

    void scheduleRunChange(int64_t frame, bool state) {
        PulseEvent ev;
        ev.type = PulseEvent::Type::Run;
        ev.frame = frame;
        ev.runState = state;
        scheduleEvent(ev);
    }

    void handleStart(int64_t frame) {
        running = true;
        scheduleRunChange(frame, true);
        scheduleResetPulse(frame);
        lockCounter = 0;
        locked = false;
        lastClockFrame = -1;
        havePeriod = false;
        eventQueue.clear();
        clockPulseRemain = 0;
        resetPulseRemain = 0;
    }

    void handleContinue(int64_t frame) {
        running = true;
        scheduleRunChange(frame, true);
    }

    void handleStop(int64_t frame) {
        running = false;
        scheduleRunChange(frame, false);
        lockCounter = 0;
        locked = false;
    }

    void handleClock(int64_t frame) {
        if (!running) {
            return;
        }

        // Calculate BPM from raw clock timing (no smoothing)
        if (lastClockFrame >= 0) {
            int64_t delta = frame - lastClockFrame;
            if (delta > 0) {
                periodSamples = (double)delta;
                havePeriod = true;
                bpm = computeBpmFromSamples(delta);
            }
        }

        lastClockFrame = frame;
        if (havePeriod && periodSamples > 0.0) {
            lockCounter = std::min(lockCounter + 1, CLOCKS_PER_BAR * 4);
            if (lockCounter >= 12) {
                locked = true;
            }
        }
        scheduleClockPulse(frame);
    }

    double computeBpmFromSamples(double samples) const {
        double sr = sampleRate();
        if (samples <= 0.0 || sr <= 0.0) {
            return 0.0;
        }
        double tickHz = sr / samples;
        return tickHz * 60.0 / CLOCKS_PER_QUARTER;
    }

    // SPP (Song Position Pointer) is ignored for hardware sync

    void process(const ProcessArgs& args) override {
        if (resetPulseLength <= 0) {
            updateResetPulseLength(args.sampleRate);
        }

        midi::Message msg;
        while (midiInput.tryPop(&msg, args.frame)) {
            uint8_t status = msg.getStatus();
            switch (status) {
                case 0x8:  // timing clock (0xF8)
                    handleClock(msg.frame);
                    break;
                case 0xa:  // start (0xFA)
                    handleStart(msg.frame);
                    break;
                case 0xb:  // continue (0xFB)
                    handleContinue(msg.frame);
                    break;
                case 0xc:  // stop (0xFC)
                    handleStop(msg.frame);
                    break;
                default:
                    break;
            }
        }

        // Unlock if no clock received for too long (timeout detection)
        if (running && havePeriod && lastClockFrame >= 0) {
            double threshold = periodSamples * 2.0;
            if (threshold <= 0.0) {
                threshold = sampleRate() * 0.1;
            }
            double framesSinceClock = (double)args.frame - (double)lastClockFrame;
            if (framesSinceClock > threshold) {
                locked = false;
                lockCounter = 0;
            }
        }

        while (!eventQueue.empty() && eventQueue.front().frame <= args.frame) {
            PulseEvent ev = eventQueue.front();
            eventQueue.pop_front();
            switch (ev.type) {
                case PulseEvent::Type::Clock:
                    clockPulseRemain = 1;
                    break;
                case PulseEvent::Type::Reset:
                    resetPulseRemain = resetPulseLength;
                    break;
                case PulseEvent::Type::Run:
                    runGate = ev.runState;
                    break;
            }
        }

        if (clockPulseRemain > 0) {
            outputs[CLK_OUTPUT].setVoltage(CLOCK_VOLTAGE);
            --clockPulseRemain;
        } else {
            outputs[CLK_OUTPUT].setVoltage(0.f);
        }

        if (resetPulseRemain > 0) {
            outputs[RESET_OUTPUT].setVoltage(RESET_VOLTAGE);
            --resetPulseRemain;
        } else {
            outputs[RESET_OUTPUT].setVoltage(0.f);
        }

        outputs[RUN_OUTPUT].setVoltage(runGate ? GATE_VOLTAGE : 0.f);

        lights[RUN_LIGHT].setBrightness(runGate ? 1.f : 0.f);
        lights[LOCK_LIGHT].setBrightness(locked ? 1.f : 0.f);
    }
};

struct MidiInputChoice : TransparentWidget {
    UsbSync* module = nullptr;
    std::string label;
    std::shared_ptr<window::Font> font;

    struct DriverItem : ui::MenuItem {
        UsbSync* module = nullptr;
        int driverId = -1;
        void onAction(const event::Action& e) override {
            if (!module) {
                return;
            }
            module->midiInput.setDriverId(driverId);
            module->midiInput.setDeviceId(-1);
            module->midiInput.reset();
        }
        void step() override {
            if (!module) {
                return;
            }
            rightText = CHECKMARK(module->midiInput.driverId == driverId);
        }
    };

    struct DeviceItem : ui::MenuItem {
        UsbSync* module = nullptr;
        int deviceId = -1;
        void onAction(const event::Action& e) override {
            if (!module) {
                return;
            }
            module->midiInput.setDeviceId(deviceId);
            module->midiInput.reset();
        }
        void step() override {
            if (!module) {
                return;
            }
            rightText = CHECKMARK(module->midiInput.deviceId == deviceId);
        }
    };

    MidiInputChoice() {
        box.size = Vec(150.f, 24.f);
    }

    void setModule(UsbSync* m) {
        module = m;
    }

    void updateLabel() {
        if (!module) {
            label = "(No module)";
            return;
        }
        std::string driver;
        if (module->midiInput.driverId >= 0) {
            midi::Driver* drv = midi::getDriver(module->midiInput.driverId);
            if (drv) {
                driver = drv->getName();
            }
        }
        std::string device =
            module->midiInput.deviceId >= 0 ? module->midiInput.getDeviceName(module->midiInput.deviceId)
                                            : "(No device)";
        label = driver.empty() ? device : driver + " — " + device;
    }

    void step() override {
        TransparentWidget::step();
        updateLabel();
    }

    void draw(const DrawArgs& args) override {
        nvgSave(args.vg);
        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0.f, 0.f, box.size.x, box.size.y, 3.f);
        nvgFillColor(args.vg, nvgRGBA(25, 25, 33, 255));
        nvgFill(args.vg);
        nvgStrokeColor(args.vg, nvgRGBA(90, 90, 110, 255));
        nvgStrokeWidth(args.vg, 0.8f);
        nvgStroke(args.vg);
        nvgRestore(args.vg);

        if (!font) {
            font = APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        }

        nvgSave(args.vg);
        if (font) {
            nvgFontFaceId(args.vg, font->handle);
        }
        nvgFontSize(args.vg, 12.f);
        nvgFillColor(args.vg, nvgRGBA(230, 230, 230, 0xff));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.5f, label.c_str(), nullptr);
        nvgRestore(args.vg);
    }

    void onButton(const event::Button& e) override {
        if (!module) {
            return;
        }
        if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            ui::Menu* menu = createMenu();

            menu->addChild(createMenuLabel("MIDI driver"));
            for (int driverId : midi::getDriverIds()) {
                DriverItem* item = new DriverItem;
                item->module = module;
                item->driverId = driverId;
                item->text = midi::getDriver(driverId)->getName();
                menu->addChild(item);
            }

            menu->addChild(new ui::MenuSeparator);
            menu->addChild(createMenuLabel("MIDI device"));
            {
                DeviceItem* item = new DeviceItem;
                item->module = module;
                item->deviceId = -1;
                item->text = "(No device)";
                menu->addChild(item);
            }
            for (int deviceId : module->midiInput.getDeviceIds()) {
                DeviceItem* item = new DeviceItem;
                item->module = module;
                item->deviceId = deviceId;
                item->text = module->midiInput.getDeviceName(deviceId);
                menu->addChild(item);
            }

            e.consume(this);
        }
    }
};

struct BpmDisplay : TransparentWidget {
    UsbSync* module = nullptr;
    std::shared_ptr<window::Font> font;

    BpmDisplay() {
        box.size = Vec(140.f, 30.f);
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

        if (!module) {
            return;
        }

        if (!font) {
            font = APP->window->loadFont(asset::system("res/fonts/ShareTechMono-Regular.ttf"));
        }
        if (!font) {
            return;
        }

        nvgSave(args.vg);
        nvgFontSize(args.vg, 20.f);
        nvgFontFaceId(args.vg, font->handle);
        nvgFillColor(args.vg, nvgRGBA(230, 230, 230, 0xff));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

        std::string bpmText = "--";
        if (module->bpm > 0.1) {
            bpmText = string::f("%0.1f BPM", module->bpm);
        }

        nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.5f, bpmText.c_str(), nullptr);
        nvgRestore(args.vg);
    }
};

struct UsbSyncWidget : ModuleWidget {
    MidiInputChoice* midiChoice = nullptr;
    BpmDisplay* bpmDisplay = nullptr;

    UsbSyncWidget(UsbSync* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/UsbSync.svg")));

        const float panelWidth = 40.64f;

        // MIDI input selector - positioned higher up on the panel
        midiChoice = createWidget<MidiInputChoice>(mm2px(Vec(2.5f, 12.f)));
        midiChoice->box.size = mm2px(Vec(panelWidth - 5.f, 9.f));
        midiChoice->setModule(module);
        addChild(midiChoice);

        // BPM display - positioned in the middle area with proper spacing
        bpmDisplay = createWidget<BpmDisplay>(mm2px(Vec(2.5f, 58.f)));
        bpmDisplay->box.size = mm2px(Vec(panelWidth - 5.f, 12.f));
        bpmDisplay->module = module;
        addChild(bpmDisplay);

        // Status lights - positioned above outputs
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(10.16f, 90.f)), module, UsbSync::LOCK_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(panelWidth - 10.16f, 90.f)), module,
                                                               UsbSync::RUN_LIGHT));

        // Outputs - positioned near the bottom
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.16f, 107.f)), module, UsbSync::CLK_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(panelWidth / 2.f, 107.f)), module, UsbSync::RUN_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(panelWidth - 10.16f, 107.f)), module, UsbSync::RESET_OUTPUT));
    }

    void step() override {
        ModuleWidget::step();
        if (midiChoice) {
            midiChoice->setModule(static_cast<UsbSync*>(module));
        }
        if (bpmDisplay) {
            bpmDisplay->module = static_cast<UsbSync*>(module);
        }
    }
};

Model* modelUsbSync = createModel<UsbSync, UsbSyncWidget>("UsbSync");

