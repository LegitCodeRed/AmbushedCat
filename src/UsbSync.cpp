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
    enum InputId {
        CLOCK_INPUT,   // Clock input - measures BPM from incoming clock
        RUN_INPUT,
        RESET_INPUT,
        INPUTS_LEN
    };
    enum OutputId {
        OUTPUTS_LEN
    };
    enum LightId {
        LOCK_LIGHT,
        RUN_LIGHT,
        LIGHTS_LEN
    };

    midi::InputQueue midiInput;
    midi::Output midiOutput;

    // Mode: true = VCV is master (send to hardware), false = Hardware is master (receive from hardware)
    bool vcvIsMaster = true;

    // For Hardware Master mode (receiving)
    bool running = false;
    bool locked = false;
    int lockCounter = 0;
    int64_t lastClockFrame = -1;
    double periodSamples = 0.0;
    bool havePeriod = false;
    double bpm = 0.0;

    // For VCV Master mode (sending)
    int64_t lastSentClockFrame = -1;
    int clocksSinceReset = 0;
    dsp::SchmittTrigger clockTrigger;
    dsp::SchmittTrigger runTrigger;
    dsp::SchmittTrigger resetTrigger;
    bool wasRunning = false;

    // Clock measurement for BPM detection
    int64_t lastInputClockFrame = -1;
    double measuredClockPeriod = 0.0;
    double measuredBpm = 0.0;

    // 24 PPQN subdivision - we need to send 24 MIDI clocks per input clock
    double subClockPhase = 0.0;
    int midiClocksPerBeat = 24;

    std::deque<PulseEvent> eventQueue;

    UsbSync() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
        configInput(CLOCK_INPUT, "Clock input (e.g., 137 BPM)");
        configInput(RUN_INPUT, "Run gate");
        configInput(RESET_INPUT, "Reset trigger");
    }

    void onReset() override {
        midiInput.reset();
        midiOutput.reset();
        running = false;
        locked = false;
        lockCounter = 0;
        lastClockFrame = -1;
        havePeriod = false;
        periodSamples = 0.0;
        bpm = 0.0;
        lastSentClockFrame = -1;
        clocksSinceReset = 0;
        wasRunning = false;
        lastInputClockFrame = -1;
        measuredClockPeriod = 0.0;
        measuredBpm = 0.0;
        subClockPhase = 0.0;
        eventQueue.clear();
    }

    json_t* dataToJson() override {
        json_t* rootJ = Module::dataToJson();
        json_object_set_new(rootJ, "midiInput", midiInput.toJson());
        json_object_set_new(rootJ, "midiOutput", midiOutput.toJson());
        json_object_set_new(rootJ, "vcvIsMaster", json_boolean(vcvIsMaster));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        Module::dataFromJson(rootJ);
        json_t* midiInJ = json_object_get(rootJ, "midiInput");
        if (midiInJ) {
            midiInput.fromJson(midiInJ);
        }
        json_t* midiOutJ = json_object_get(rootJ, "midiOutput");
        if (midiOutJ) {
            midiOutput.fromJson(midiOutJ);
        }
        json_t* modeJ = json_object_get(rootJ, "vcvIsMaster");
        if (modeJ) {
            vcvIsMaster = json_boolean_value(modeJ);
        }
    }

    double sampleRate() const {
        return APP ? APP->engine->getSampleRate() : 44100.0;
    }

    void sendMidiClock() {
        if (!vcvIsMaster || midiOutput.getDeviceId() < 0) return;
        midi::Message msg;
        msg.bytes[0] = 0xF8;  // MIDI Clock (System Real-Time, no channel)
        msg.setSize(1);
        midiOutput.sendMessage(msg);
    }

    void sendMidiStart() {
        if (!vcvIsMaster || midiOutput.getDeviceId() < 0) return;
        midi::Message msg;
        msg.bytes[0] = 0xFA;  // MIDI Start (System Real-Time, no channel)
        msg.setSize(1);
        midiOutput.sendMessage(msg);
        clocksSinceReset = 0;
    }

    void sendMidiStop() {
        if (!vcvIsMaster || midiOutput.getDeviceId() < 0) return;
        midi::Message msg;
        msg.bytes[0] = 0xFC;  // MIDI Stop (System Real-Time, no channel)
        msg.setSize(1);
        midiOutput.sendMessage(msg);
    }

    void sendMidiContinue() {
        if (!vcvIsMaster || midiOutput.getDeviceId() < 0) return;
        midi::Message msg;
        msg.bytes[0] = 0xFB;  // MIDI Continue (System Real-Time, no channel)
        msg.setSize(1);
        midiOutput.sendMessage(msg);
    }

    // Hardware Master mode - receive from hardware
    void handleStart(int64_t frame) {
        running = true;
        lockCounter = 0;
        locked = false;
        lastClockFrame = -1;
        havePeriod = false;
    }

    void handleContinue(int64_t frame) {
        running = true;
    }

    void handleStop(int64_t frame) {
        running = false;
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
    }

    double computeBpmFromSamples(double samples) const {
        double sr = sampleRate();
        if (samples <= 0.0 || sr <= 0.0) {
            return 0.0;
        }
        double tickHz = sr / samples;
        return tickHz * 60.0 / CLOCKS_PER_QUARTER;
    }

    void process(const ProcessArgs& args) override {
        if (vcvIsMaster) {
            // VCV Master Mode: Send MIDI clock to hardware based on CV inputs
            processVcvMaster(args);
        } else {
            // Hardware Master Mode: Receive MIDI clock from hardware
            processHardwareMaster(args);
        }

        lights[RUN_LIGHT].setBrightness(running ? 1.f : 0.f);
        lights[LOCK_LIGHT].setBrightness(locked ? 1.f : 0.f);
    }

    void processVcvMaster(const ProcessArgs& args) {
        // Read CV inputs
        bool clockHigh = inputs[CLOCK_INPUT].getVoltage() >= 1.f;
        bool runHigh = inputs[RUN_INPUT].getVoltage() >= 1.f;
        bool resetHigh = inputs[RESET_INPUT].getVoltage() >= 1.f;

        // Detect run start/stop
        bool runRising = runTrigger.process(runHigh);
        if (runRising && runHigh && !wasRunning) {
            sendMidiStart();
            running = true;
            locked = false;
            subClockPhase = 0.0;
            lastInputClockFrame = -1;
        } else if (!runHigh && wasRunning) {
            sendMidiStop();
            running = false;
            locked = false;
        }
        wasRunning = runHigh;

        // Detect reset
        if (resetTrigger.process(resetHigh)) {
            sendMidiStart();
            clocksSinceReset = 0;
            subClockPhase = 0.0;
            lastInputClockFrame = -1;
        }

        // Detect input clock edges and measure period
        if (clockTrigger.process(clockHigh)) {
            // Rising edge of input clock (one beat)
            if (lastInputClockFrame >= 0) {
                // Measure the period between beats
                int64_t delta = args.frame - lastInputClockFrame;
                if (delta > 0) {
                    measuredClockPeriod = (double)delta;
                    // Calculate BPM: samples per beat -> beats per second -> beats per minute
                    double beatsPerSecond = args.sampleRate / measuredClockPeriod;
                    measuredBpm = beatsPerSecond * 60.0;
                    bpm = measuredBpm;
                    locked = true;
                    havePeriod = true;
                }
            }
            lastInputClockFrame = args.frame;
            subClockPhase = 0.0;  // Reset subdivision phase on each beat
        }

        // Generate 24 MIDI clocks per input clock beat (24 PPQN subdivision)
        if (running && measuredClockPeriod > 0.0) {
            // Calculate how fast to advance the sub-clock phase
            // We want to generate 24 clocks during one beat period
            double subClockIncrement = (double)midiClocksPerBeat / measuredClockPeriod;

            subClockPhase += subClockIncrement;

            // Send MIDI clock every time phase crosses an integer boundary
            while (subClockPhase >= 1.0) {
                subClockPhase -= 1.0;
                sendMidiClock();
                clocksSinceReset++;
                lastSentClockFrame = args.frame;
            }
        }
    }

    void processHardwareMaster(const ProcessArgs& args) {
        // Receive MIDI clock from hardware
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
    }
};

struct MidiDeviceDisplay : LedDisplay {
    UsbSync* module = nullptr;
    bool isOutput = false;  // true for output, false for input

    void setModule(UsbSync* m) {
        module = m;
    }

    void step() override {
        LedDisplay::step();
    }

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

            std::string text = "(No device)";
            if (module) {
                if (isOutput) {
                    text = module->midiOutput.getDeviceName(module->midiOutput.deviceId);
                } else {
                    text = module->midiInput.getDeviceName(module->midiInput.deviceId);
                }
            }
            nvgText(args.vg, box.size.x / 2, box.size.y / 2, text.c_str(), NULL);
        }

        nvgResetScissor(args.vg);
    }

    void onButton(const event::Button& e) override {
        if (e.action == GLFW_PRESS && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            if (!module) {
                return;
            }

            ui::Menu* menu = createMenu();

            if (isOutput) {
                appendMidiMenu(menu, &module->midiOutput);
            } else {
                appendMidiMenu(menu, &module->midiInput);
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
    MidiDeviceDisplay* midiOutDisplay = nullptr;
    MidiDeviceDisplay* midiInDisplay = nullptr;
    BpmDisplay* bpmDisplay = nullptr;

    UsbSyncWidget(UsbSync* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/UsbSync.svg")));

        const float panelWidth = 40.64f;

        // MIDI output selector (VCV -> Hardware)
        midiOutDisplay = createWidget<MidiDeviceDisplay>(mm2px(Vec(2.5f, 12.f)));
        midiOutDisplay->box.size = mm2px(Vec(panelWidth - 5.f, 7.f));
        midiOutDisplay->setModule(module);
        midiOutDisplay->isOutput = true;
        addChild(midiOutDisplay);

        // MIDI input selector (Hardware -> VCV)
        midiInDisplay = createWidget<MidiDeviceDisplay>(mm2px(Vec(2.5f, 21.f)));
        midiInDisplay->box.size = mm2px(Vec(panelWidth - 5.f, 7.f));
        midiInDisplay->setModule(module);
        midiInDisplay->isOutput = false;
        addChild(midiInDisplay);

        // BPM display - positioned in the middle area with proper spacing
        bpmDisplay = createWidget<BpmDisplay>(mm2px(Vec(2.5f, 58.f)));
        bpmDisplay->box.size = mm2px(Vec(panelWidth - 5.f, 12.f));
        bpmDisplay->module = module;
        addChild(bpmDisplay);

        // Status lights - positioned above inputs
        addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(10.16f, 90.f)), module, UsbSync::LOCK_LIGHT));
        addChild(createLightCentered<MediumLight<YellowLight>>(mm2px(Vec(panelWidth - 10.16f, 90.f)), module,
                                                               UsbSync::RUN_LIGHT));

        // CV Inputs - positioned near the bottom
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16f, 107.f)), module, UsbSync::CLOCK_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(panelWidth / 2.f, 107.f)), module, UsbSync::RUN_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(panelWidth - 10.16f, 107.f)), module, UsbSync::RESET_INPUT));
    }

    void appendContextMenu(Menu* menu) override {
        UsbSync* usbSync = dynamic_cast<UsbSync*>(module);
        if (!usbSync) return;

        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Sync Mode"));

        struct ModeItem : MenuItem {
            UsbSync* module;
            bool vcvMaster;
            void onAction(const event::Action& e) override {
                module->vcvIsMaster = vcvMaster;
            }
        };

        ModeItem* vcvMasterItem = new ModeItem;
        vcvMasterItem->text = "VCV is Master (send to hardware)";
        vcvMasterItem->rightText = CHECKMARK(usbSync->vcvIsMaster);
        vcvMasterItem->module = usbSync;
        vcvMasterItem->vcvMaster = true;
        menu->addChild(vcvMasterItem);

        ModeItem* hwMasterItem = new ModeItem;
        hwMasterItem->text = "Hardware is Master (receive from hardware)";
        hwMasterItem->rightText = CHECKMARK(!usbSync->vcvIsMaster);
        hwMasterItem->module = usbSync;
        hwMasterItem->vcvMaster = false;
        menu->addChild(hwMasterItem);
    }

    void step() override {
        ModuleWidget::step();
        if (midiOutDisplay) {
            midiOutDisplay->setModule(static_cast<UsbSync*>(module));
        }
        if (midiInDisplay) {
            midiInDisplay->setModule(static_cast<UsbSync*>(module));
        }
        if (bpmDisplay) {
            bpmDisplay->module = static_cast<UsbSync*>(module);
        }
    }
};

Model* modelUsbSync = createModel<UsbSync, UsbSyncWidget>("UsbSync");

