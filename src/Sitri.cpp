#include "plugin.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

using rack::math::clamp;

namespace sitri {

// -----------------------------------------------------------------------------
// Random utilities
// -----------------------------------------------------------------------------

static inline uint64_t xorshift64(uint64_t& state) {
        if (state == 0)
                state = 0x9e3779b97f4a7c15ull;
        uint64_t x = state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        state = x;
        return x * 2685821657736338717ull;
}

static inline float rand01(uint64_t& state) {
        return (float)((xorshift64(state) >> 11) * (1.0 / (double)(1ull << 53)));
}

static inline int randRange(uint64_t& state, int maxExclusive) {
        return (int)(xorshift64(state) % (uint64_t)maxExclusive);
}

static inline int randChoice(const std::initializer_list<int>& values, uint64_t& state) {
        int idx = randRange(state, (int)values.size());
        auto it = values.begin();
        std::advance(it, idx);
        return *it;
}

static inline float degToVolts(int deg) {
        return deg / 12.f;
}

static inline bool euclidHit(int step, int steps, int pulses) {
        if (steps <= 0 || pulses <= 0)
                return false;
        int wrappedStep = (step % steps + steps) % steps;
        int prevBucket = (wrappedStep * pulses) / steps;
        int nextBucket = ((wrappedStep + 1) * pulses) / steps;
        return prevBucket != nextBucket;
}

// -----------------------------------------------------------------------------
// Step event + algo context
// -----------------------------------------------------------------------------

struct StepEvent {
        bool active = true;
        float pitch = 0.f;
        float prob = 1.f;
        float vel = 0.8f;
        float gateFrac = 0.5f;
};

struct AlgoContext {
        int stepIndex = 0;
        int steps = 16;
        float density = 0.5f;
        float accent = 0.5f;
        float seedNoise = 0.f;
        mutable uint64_t prngState = 1;
        float lastPitch = 0.f;
        float lastVel = 0.8f;
        float phase01 = 0.f;
        float divHz = 1.f;
};

struct IAlgorithm {
        virtual ~IAlgorithm() = default;
        virtual const char* id() const = 0;
        virtual const char* displayName() const = 0;
        virtual int paramCount() const { return 0; }
        virtual const char* paramName(int) const { return ""; }
        virtual void setParam(int, float) {}
        virtual void reset(uint64_t) {}
        virtual StepEvent generate(const AlgoContext& ctx) = 0;
};

using AlgoFactory = std::function<std::unique_ptr<IAlgorithm>()>;

class AlgoRegistry {
public:
        static AlgoRegistry& instance() {
                static AlgoRegistry reg;
                return reg;
        }

        void registerAlgo(const std::string& id, AlgoFactory f) {
                if (factories.count(id))
                        return;

                AlgoFactory factory = std::move(f);
                AlgoFactory probe = factory;
                std::string display = id;
                if (probe) {
                        if (auto sample = probe())
                                display = sample->displayName();
                }

                order.push_back(id);
                factories[id] = std::move(factory);
                displayNames[id] = display;
        }

        std::vector<std::string> ids() const {
                return order;
        }

        std::unique_ptr<IAlgorithm> create(const std::string& id) const {
                auto it = factories.find(id);
                if (it == factories.end())
                        return nullptr;
                return it->second();
        }

        std::string getDisplayName(const std::string& id) const {
                auto it = displayNames.find(id);
                if (it != displayNames.end())
                        return it->second;
                return id;
        }

private:
        std::map<std::string, AlgoFactory> factories;
        std::map<std::string, std::string> displayNames;
        std::vector<std::string> order;
};

#define REGISTER_ALGO(ID, TYPE)                                                     \
        namespace {                                                                 \
        struct TYPE##Reg {                                                          \
                TYPE##Reg() {                                                       \
                        AlgoRegistry::instance().registerAlgo(                      \
                            ID, []() { return std::unique_ptr<IAlgorithm>(new TYPE()); }); \
                }                                                                   \
        };                                                                          \
        static TYPE##Reg TYPE##RegInstance;                                         \
        }

// -----------------------------------------------------------------------------
// Algorithms
// -----------------------------------------------------------------------------

struct AlgoRandom : public IAlgorithm {
        const char* id() const override { return "raxdm"; }
        const char* displayName() const override { return "RAxDOM"; }
        StepEvent generate(const AlgoContext& c) override {
                AlgoContext ctx = c;
                StepEvent e;
                e.active = rand01(ctx.prngState) < ctx.density;
                float step = (rand01(ctx.prngState) - 0.5f) * 0.8f;
                e.pitch = ctx.lastPitch + step;
                e.vel = 0.4f + 0.6f * rand01(ctx.prngState);
                e.gateFrac = 0.25f + 0.65f * rand01(ctx.prngState);
                return e;
        }
};
REGISTER_ALGO("raxdm", AlgoRandom);

struct AlgoAccrete : public IAlgorithm {
        float center = 0.f;
        const char* id() const override { return "acxom"; }
        const char* displayName() const override { return "ACxEOM"; }
        void reset(uint64_t) override { center = 0.f; }
        StepEvent generate(const AlgoContext& c) override {
                AlgoContext ctx = c;
                StepEvent e;
                e.active = rand01(ctx.prngState) < (0.6f * ctx.density + 0.2f);
                center += (rand01(ctx.prngState) - 0.5f) * 0.06f;
                float target = 0.7f * center + 0.3f * ctx.lastPitch;
                e.pitch = target + (rand01(ctx.prngState) - 0.5f) * 0.2f;
                e.vel = 0.5f + 0.5f * ctx.accent * rand01(ctx.prngState);
                e.gateFrac = 0.35f + 0.4f * ctx.density;
                return e;
        }
};
REGISTER_ALGO("acxom", AlgoAccrete);

struct AlgoAcid : public IAlgorithm {
        int lastScaleDeg = 0;
        const char* id() const override { return "xacidx"; }
        const char* displayName() const override { return "XACIDx"; }
        StepEvent generate(const AlgoContext& c) override {
                AlgoContext ctx = c;
                StepEvent e;
                e.active = rand01(ctx.prngState) < (0.8f * ctx.density + 0.1f);
                int delta = (rand01(ctx.prngState) < 0.8f) ? randChoice({-1, 0, 1}, ctx.prngState)
                                                          : randChoice({-5, 5}, ctx.prngState);
                lastScaleDeg = clamp(lastScaleDeg + delta, -12, 12);
                float octave = (rand01(ctx.prngState) < 0.15f) ? 1.f : 0.f;
                e.pitch = degToVolts(lastScaleDeg) + octave;
                e.vel = 0.7f + 0.3f * ctx.accent;
                e.gateFrac = (rand01(ctx.prngState) < 0.3f) ? 0.95f : 0.45f;
                return e;
        }
};
REGISTER_ALGO("xacidx", AlgoAcid);

struct AlgoStingPulse : public IAlgorithm {
        const char* id() const override { return "sting"; }
        const char* displayName() const override { return "STING Pulse"; }

        void reset(uint64_t) override {
                leadOffset = 0;
        }

        StepEvent generate(const AlgoContext& c) override {
                AlgoContext ctx = c;
                StepEvent e;
                static constexpr std::array<bool, 16> anchorMask = {
                    true, false, false, false, true, false, true, false,
                    true, false, false, false, true, false, true, false};
                static constexpr std::array<bool, 16> ghostMask = {
                    false, true, false, true, false, true, false, true,
                    false, true, false, true, false, true, false, true};
                static constexpr std::array<int, 16> degreePattern = {
                    0, 7, 2, 9, 0, 7, 5, 10, 0, 7, 2, 9, 0, 7, 5, 10};
                static constexpr std::array<float, 16> accentShape = {
                    1.f, 0.3f, 0.2f, 0.4f, 0.9f, 0.3f, 0.7f, 0.35f,
                    1.f, 0.35f, 0.25f, 0.45f, 0.85f, 0.35f, 0.75f, 0.4f};

                int idx = ctx.stepIndex % 16;
                bool anchor = anchorMask[idx];
                bool ghost = ghostMask[idx];

                float anchorProbability = clamp(0.65f + 0.25f * ctx.density + 0.1f * ctx.accent, 0.f, 1.f);
                float ghostProbability = clamp(0.25f + 0.55f * ctx.density, 0.f, 1.f);
                float improvProbability = clamp(0.08f + ctx.seedNoise * 0.4f, 0.f, 1.f);

                bool active = false;
                float trigger = rand01(ctx.prngState);
                if (anchor)
                        active = trigger < anchorProbability;
                else if (ghost)
                        active = trigger < ghostProbability;
                else
                        active = trigger < (0.15f + 0.5f * ctx.density);

                if (!active && rand01(ctx.prngState) < improvProbability)
                        active = true;

                e.active = active;
                e.prob = active ? 1.f : 0.f;

                int degree = degreePattern[idx];
                if (anchor && rand01(ctx.prngState) < (0.35f + 0.35f * ctx.accent))
                        degree += 12;
                if (ghost && rand01(ctx.prngState) < (0.25f * ctx.seedNoise))
                        degree += randChoice({-12, 12}, ctx.prngState);

                float expressiveNudge = (rand01(ctx.prngState) - 0.5f) * 0.12f;
                e.pitch = degToVolts(degree + leadOffset) + expressiveNudge;

                if (anchor && rand01(ctx.prngState) < (0.15f + 0.35f * ctx.seedNoise)) {
                        leadOffset = clamp(leadOffset + randChoice({-7, -5, 5, 7}, ctx.prngState), -12, 24);
                }

                float baseVel = anchor ? 0.8f : (ghost ? 0.58f : 0.48f);
                baseVel += accentShape[idx] * 0.22f * ctx.accent;
                baseVel += (rand01(ctx.prngState) - 0.5f) * 0.1f;
                e.vel = clamp(baseVel, 0.f, 1.f);

                float gate = anchor ? 0.6f : 0.38f;
                if (ghost)
                        gate -= 0.08f;
                gate += 0.18f * ctx.accent;
                gate = clamp(gate, 0.2f, 0.95f);
                e.gateFrac = gate;

                return e;
        }

private:
        int leadOffset = 0;
};
REGISTER_ALGO("sting", AlgoStingPulse);

struct AlgoStingSwarm : public IAlgorithm {
        const char* id() const override { return "sting2"; }
        const char* displayName() const override { return "STING Swarm"; }

        void reset(uint64_t) override {
                lastStep = -1;
                phrase = 0;
                runningDegree = 0;
        }

        StepEvent generate(const AlgoContext& c) override {
                AlgoContext ctx = c;
                StepEvent e;

                if (lastStep >= 0 && ctx.stepIndex < lastStep)
                        phrase = (phrase + 1) % 8;
                lastStep = ctx.stepIndex;

                int length = std::max(4, ctx.steps);
                int idx = (ctx.stepIndex + phrase) % length;
                int pulses = clamp((int)std::round((0.3f + 0.65f * ctx.density) * length), 1, length);
                int accentPulses = clamp(pulses / 2 + 1, 1, length);
                bool hit = euclidHit(idx, length, pulses);
                bool accent = euclidHit((idx + phrase * 3) % length, length, accentPulses);

                float improv = clamp(0.1f + ctx.seedNoise * 0.5f, 0.f, 1.f);
                if (!hit && rand01(ctx.prngState) < improv)
                        hit = true;

                e.active = hit;
                e.prob = hit ? 1.f : 0.f;

                if (hit) {
                        int motion = accent ? randChoice({7, 5, 12}, ctx.prngState)
                                            : randChoice({0, 2, -3, 3}, ctx.prngState);
                        if (rand01(ctx.prngState) < (0.2f * ctx.seedNoise))
                                motion = randChoice({-12, -7, 12, 14}, ctx.prngState);
                        runningDegree = clamp(runningDegree + motion, -24, 36);
                        e.pitch = degToVolts(runningDegree);

                        float vel = accent ? 0.88f : 0.6f;
                        vel += 0.22f * ctx.accent;
                        vel += (rand01(ctx.prngState) - 0.5f) * 0.12f;
                        e.vel = clamp(vel, 0.f, 1.f);

                        float gate = accent ? 0.72f : 0.48f;
                        gate += 0.15f * ctx.accent;
                        gate = clamp(gate, 0.25f, 0.95f);
                        e.gateFrac = gate;
                } else {
                        e.pitch = ctx.lastPitch;
                        e.vel = 0.4f;
                        e.gateFrac = 0.3f;
                }

                return e;
        }

private:
        int lastStep = -1;
        int phrase = 0;
        int runningDegree = 0;
};
REGISTER_ALGO("sting2", AlgoStingSwarm);

struct AlgoEuclidGroove : public IAlgorithm {
        const char* id() const override { return "eucl"; }
        const char* displayName() const override { return "EUCLIDEAN"; }

        void reset(uint64_t) override {
                lastDegree = 0;
        }

        StepEvent generate(const AlgoContext& c) override {
                AlgoContext ctx = c;
                StepEvent e;

                int length = std::max(1, ctx.steps);
                int pulses = clamp((int)std::round(ctx.density * length), 1, length);
                int rotate = clamp((int)std::round(ctx.seedNoise * length), 0, length - 1);
                int idx = (ctx.stepIndex + rotate) % length;
                bool hit = euclidHit(idx, length, pulses);

                e.active = hit;
                e.prob = hit ? 1.f : 0.f;

                if (hit) {
                        bool accent = euclidHit((idx + length / 3) % length, length, std::max(1, pulses / 2));
                        int motion = accent ? randChoice({7, 5, 12}, ctx.prngState)
                                            : randChoice({0, 2, -2, -5}, ctx.prngState);
                        if (rand01(ctx.prngState) < (0.2f * ctx.seedNoise))
                                motion = randChoice({-12, 12}, ctx.prngState);
                        lastDegree = clamp(lastDegree + motion, -36, 36);
                        e.pitch = degToVolts(lastDegree);

                        float vel = accent ? 0.9f : 0.6f;
                        vel += 0.2f * ctx.accent;
                        vel += (rand01(ctx.prngState) - 0.5f) * 0.1f;
                        e.vel = clamp(vel, 0.f, 1.f);

                        float gate = accent ? 0.75f : 0.45f;
                        gate += 0.15f * ctx.accent;
                        e.gateFrac = clamp(gate, 0.2f, 0.95f);
                } else {
                        e.pitch = ctx.lastPitch;
                        e.vel = 0.4f;
                        e.gateFrac = 0.3f;
                }

                return e;
        }

private:
        int lastDegree = 0;
};
REGISTER_ALGO("eucl", AlgoEuclidGroove);

// -----------------------------------------------------------------------------
// Quantizer
// -----------------------------------------------------------------------------

class Quantizer {
public:
        struct ScaleDef {
                const char* name;
                std::vector<int> degrees;
        };

        Quantizer() {
                setScaleIndex(0);
        }

        void setScaleIndex(int index) {
                if (index < 0)
                        index = 0;
                if (index >= (int)scales().size())
                        index = (int)scales().size() - 1;
                if (scaleIndex != index) {
                        scaleIndex = index;
                        updateTable();
                        ++revision;
                }
        }

        void setRoot(float v) {
                if (std::fabs(root - v) > 1e-6f) {
                        root = v;
                        ++revision;
                }
        }
        void setTranspose(float v) {
                if (std::fabs(transpose - v) > 1e-6f) {
                        transpose = v;
                        ++revision;
                }
        }

        float snap(float vOct) const {
                // Apply transpose AFTER quantization so it shifts the entire output
                float rel = vOct - root;
                int semitone = (int)std::floor(rel * 12.f + 0.5f);
                int octave = floorDiv(semitone, 12);
                int degree = semitone - octave * 12;
                int snappedDeg = nearestAllowedDegree(degree);
                int snappedSemitone = octave * 12 + snappedDeg;
                return snappedSemitone / 12.f + root + transpose;
        }

        const std::vector<std::string>& scaleNames() const {
                return namesCache();
        }

        int getScaleIndex() const { return scaleIndex; }

        float getRoot() const { return root; }

        uint64_t getRevision() const { return revision; }

private:
        int scaleIndex = -1;
        float root = 0.f;
        float transpose = 0.f;
        std::array<bool, 12> allowed{};
        uint64_t revision = 0;

        void updateTable() {
                allowed.fill(false);
                for (int deg : scales()[scaleIndex].degrees) {
                        if (deg >= 0 && deg < 12)
                                allowed[deg] = true;
                }
                // chromatic fallback
                if (std::none_of(allowed.begin(), allowed.end(), [](bool b) { return b; })) {
                        allowed.fill(true);
                }
        }

        int nearestAllowedDegree(int deg) const {
                deg = (deg % 12 + 12) % 12;
                if (allowed[deg])
                        return deg;
                for (int radius = 1; radius < 12; ++radius) {
                        int up = (deg + radius) % 12;
                        if (allowed[up])
                                return up;
                        int down = (deg - radius + 12) % 12;
                        if (allowed[down])
                                return down;
                }
                return deg;
        }

        static int floorDiv(int a, int b) {
                int q = a / b;
                int r = a % b;
                if ((r != 0) && ((r < 0) != (b < 0)))
                        q--;
                return q;
        }

        static const std::vector<ScaleDef>& scales() {
                static const std::vector<ScaleDef> defs = {
                    {"Chromatic", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}},
                    {"Major", {0, 2, 4, 5, 7, 9, 11}},
                    {"Minor", {0, 2, 3, 5, 7, 8, 10}},
                    {"Dorian", {0, 2, 3, 5, 7, 9, 10}},
                    {"Phrygian", {0, 1, 3, 5, 7, 8, 10}},
                    {"Mixolydian", {0, 2, 4, 5, 7, 9, 10}},
                    {"Locrian", {0, 1, 3, 5, 6, 8, 10}},
                    {"Whole", {0, 2, 4, 6, 8, 10}},
                    {"Harmonic Minor", {0, 2, 3, 5, 7, 8, 11}},
                    {"Melodic Minor", {0, 2, 3, 5, 7, 9, 11}},
                    {"Minor Pent", {0, 3, 5, 7, 10}},
                    {"Major Pent", {0, 2, 4, 7, 9}},
                    {"Phrygian Dom", {0, 1, 4, 5, 7, 8, 10}},
                    {"Whole-Half", {0, 2, 3, 5, 6, 8, 9, 11}},
                };
                return defs;
        }

        static const std::vector<std::string>& namesCache() {
                static std::vector<std::string> cache;
                if (cache.empty()) {
                        for (const auto& s : scales())
                                cache.push_back(s.name);
                }
                return cache;
        }
};

// -----------------------------------------------------------------------------
// Sequencer core
// -----------------------------------------------------------------------------

class SequencerCore {
public:
        void setAlgorithm(std::unique_ptr<IAlgorithm> a) {
                algoOwner = std::move(a);
                algo = algoOwner.get();
        }

        void setQuantizer(Quantizer* q) {
                quantizer = q;
                quantizerRevision = quantizer ? quantizer->getRevision() : 0;
        }

        void reset(uint64_t seed) {
                uint64_t actualSeed = seed ? seed : 0x12345678abcdefULL;
                baseSeed = actualSeed;
                prngState = actualSeed;
                gateOut = false;
                gateTimer = 0.f;
                phase = 0.f;
                playCounter = 0;
                pingDir = 1;
                pingStep = 0;
                totalStepCount = 0;
                lastPitch = 0.f;
                lastVel = 0.8f;
                eocPulse = false;
                lastRawPitch = 0.f;
                lastStepActive = false;
                lastGateFracApplied = clamp(gatePercent, 0.01f, 1.f);
                quantizerRevision = quantizer ? quantizer->getRevision() : 0;
                clockPeriod = 0.5f;
                timeSinceLastClock = 0.f;
                lastRandomStep = -1;
                cycleResetPending = false;
                externalClockActive = false;
                externalClockDivCounter = 0;
                if (algo)
                        algo->reset(actualSeed);
        }

        void setSteps(int s) {
                s = clamp(s, 1, 64);
                if (steps != s) {
                        steps = s;
                        playCounter = 0;
                        pingStep = 0;
                        pingDir = 1;
                        totalStepCount = 0;
                        lastRandomStep = -1;
                        cycleResetPending = true;
                        externalClockDivCounter = 0;
                }
        }
        void setOffset(int o) { offset = o; }
        void setDensity(float d) { density = clamp(d, 0.f, 1.f); }
        void setAccent(float a) { accent = clamp(a, 0.f, 1.f); }
        void setGatePercent(float g) {
                gatePercent = clamp(g, 0.05f, 1.f);
                lastGateFracApplied = clamp(gatePercent, 0.01f, 1.f);
        }
        void setDivHz(float hz) {
                divHz = std::max(hz, 0.01f);
                // Calculate clock division for external clocks
                // divHz of 2Hz = divide by 1, 4Hz = divide by 2, 8Hz = divide by 4, etc.
                externalClockDivAmount = std::max(1, (int)std::round(divHz / 2.0f));
        }
        void setSwing(float s) { swing = clamp(s, 0.f, 0.6f); }
        void setDirection(int dir) {
                if (direction != dir) {
                        direction = dir;
                        playCounter = 0;
                        pingStep = 0;
                        pingDir = 1;
                        totalStepCount = 0;
                        lastRandomStep = -1;
                        cycleResetPending = true;
                        externalClockDivCounter = 0;
                }
        }

        void process(float sampleTime, bool clockEdge, bool externalClockConnected) {
                eocPulse = false;

                if (quantizer && quantizer->getRevision() != quantizerRevision)
                        onQuantizerChanged();

                // Update gate timer before potentially advancing step
                if (gateTimer > 0.f) {
                        gateTimer -= sampleTime;
                        if (gateTimer <= 0.f) {
                                gateOut = false;
                                gateTimer = 0.f;
                        }
                }

                if (externalClockActive != externalClockConnected) {
                        externalClockActive = externalClockConnected;
                        externalClockDivCounter = 0;
                        if (!externalClockActive) {
                                phase = 0.f;
                                timeSinceLastClock = 0.f;
                        }
                }

                timeSinceLastClock += sampleTime;

                if (clockEdge) {
                        // Measure clock period for external clock
                        if (externalClockConnected && timeSinceLastClock > 0.001f) { // Avoid division by zero
                                clockPeriod = timeSinceLastClock;
                        }
                        timeSinceLastClock = 0.f;
                        phase = 0.f;

                        // Apply clock division for external clock
                        externalClockDivCounter++;
                        if (externalClockDivCounter >= externalClockDivAmount) {
                                externalClockDivCounter = 0;
                                advanceStep(true); // true = external clock
                        }
                } else if (!externalClockConnected) {
                        phase += sampleTime;
                        if (phase >= currentStepDuration()) {
                                phase -= currentStepDuration();
                                advanceStep(false); // false = internal clock
                        }
                } else {
                        // External clock active but no edge this sample.
                        phase = 0.f;
                }
        }

        bool gateOut = false;
        float pitchOut = 0.f;
        float velOut = 0.f;
        bool eocPulse = false;

        uint64_t prngState = 0x12345678abcdefULL;

private:
        double phase = 0.0;
        int steps = 16;
        int offset = 0;
        float density = 0.5f;
        float accent = 0.5f;
        float gatePercent = 0.5f;
        float swing = 0.f;
        float divHz = 2.f;
        int direction = 0; // 0 fwd, 1 rev, 2 pingpong, 3 random

        IAlgorithm* algo = nullptr;
        std::unique_ptr<IAlgorithm> algoOwner;
        Quantizer* quantizer = nullptr;

        float lastPitch = 0.f;
        float lastVel = 0.8f;

        int playCounter = 0;
        int pingDir = 1;
        int pingStep = 0;
        int totalStepCount = 0;
        int lastRandomStep = -1;

        float gateTimer = 0.f;
        float currentDuration = 0.5f;
        float clockPeriod = 0.5f;
        float timeSinceLastClock = 0.f;
        float lastRawPitch = 0.f;
        float lastGateFracApplied = 0.5f;
        bool lastStepActive = false;
        uint64_t quantizerRevision = 0;
        uint64_t baseSeed = 0x12345678abcdefULL;
        bool cycleResetPending = false;
        bool externalClockActive = false;
        int externalClockDivCounter = 0;
        int externalClockDivAmount = 1;

        float currentStepDuration() const {
                return currentDuration;
        }

        void onQuantizerChanged() {
                quantizerRevision = quantizer->getRevision();
                if (!lastStepActive)
                        return;
                float newPitch = quantizer->snap(lastRawPitch);
                if (std::fabs(newPitch - pitchOut) < 1e-5f)
                        return;

                float gateFrac = clamp(lastGateFracApplied, 0.01f, 1.f);
                gateOut = true;
                gateTimer = currentStepDuration() * gateFrac;
                pitchOut = newPitch;
                lastPitch = newPitch;
                velOut = lastVel * 10.f;
        }

        void advanceStep(bool externalClock = false) {
                if (!algo || !quantizer || steps <= 0)
                        return;

                if (cycleResetPending)
                        restoreCycleState();

                int baseStep = computeBaseStep();
                int logicalIndex = wrapIndex(baseStep + offset, steps);

                AlgoContext ctx;
                ctx.stepIndex = logicalIndex;
                ctx.steps = steps;
                ctx.density = density;
                ctx.accent = accent;
                ctx.seedNoise = rand01(prngState);
                ctx.prngState = prngState;
                ctx.lastPitch = lastPitch;
                ctx.lastVel = lastVel;
                ctx.phase01 = 0.f;
                ctx.divHz = divHz;

                StepEvent proposal = algo->generate(ctx);
                prngState = ctx.prngState;

                bool active = proposal.active;
                float chance = clamp(proposal.prob * density, 0.f, 1.f);
                if (rand01(prngState) > chance)
                        active = false;

                // Use external clock period if available, otherwise use internal divHz
                float duration = externalClock ? clockPeriod : (1.f / divHz);
                bool oddStep = (totalStepCount % 2) == 1;
                float swingScale = 1.f;
                if (swing > 0.f) {
                        if (oddStep)
                                swingScale = 1.f + swing;
                        else
                                swingScale = std::max(0.1f, 1.f - swing);
                }
                currentDuration = duration * swingScale;
                phase = 0.f;

                if (active) {
                        float shapedVel = proposal.vel * (0.4f + 0.6f * accent);
                        shapedVel = clamp(shapedVel, 0.f, 1.f);
                        float rawPitch = proposal.pitch;
                        float snapped = quantizer->snap(rawPitch);
                        pitchOut = snapped;
                        velOut = shapedVel * 10.f;
                        gateOut = true;
                        float gateFrac = clamp(proposal.gateFrac * gatePercent, 0.01f, 1.f);
                        gateTimer = currentDuration * gateFrac;
                        lastGateFracApplied = gateFrac;
                        lastRawPitch = rawPitch;
                        lastStepActive = true;
                        quantizerRevision = quantizer->getRevision();
                        lastPitch = snapped;
                        lastVel = shapedVel;
                } else {
                        // Inactive step - turn off gate immediately
                        gateOut = false;
                        gateTimer = 0.f;
                        lastStepActive = false;
                }

                advanceCounters();
                totalStepCount++;
        }

        void advanceCounters() {
                switch (direction) {
                case 0: // forward
                        playCounter = (playCounter + 1) % steps;
                        if (playCounter == 0) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        break;
                case 1: // reverse
                        playCounter = (playCounter + 1) % steps;
                        if (playCounter == 0) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        break;
                case 2: // pingpong
                        if (steps <= 1) {
                                eocPulse = true;
                                cycleResetPending = true;
                                break;
                        }
                        pingStep += pingDir;
                        if (pingStep >= steps) {
                                pingDir = -1;
                                pingStep = steps - 2;
                                eocPulse = true;
                                cycleResetPending = true;
                        } else if (pingStep < 0) {
                                pingDir = 1;
                                pingStep = 1;
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        playCounter = (playCounter + 1) % steps;
                        break;
                case 3: // random
                        playCounter = (playCounter + 1) % steps;
                        if (playCounter == 0) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        break;
                default:
                        playCounter = (playCounter + 1) % steps;
                        if (playCounter == 0) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        break;
                }
        }

        int computeBaseStep() {
                switch (direction) {
                case 0: // forward
                        return playCounter;
                case 1: // reverse
                        return steps - 1 - playCounter;
                case 2: // pingpong
                        return clamp(pingStep, 0, steps - 1);
                case 3: { // random
                        if (steps <= 0)
                                return 0;
                        if (steps == 1) {
                                lastRandomStep = 0;
                                return 0;
                        }
                        int candidate = randRange(prngState, steps);
                        int guard = 0;
                        while (candidate == lastRandomStep && guard < 8) {
                                candidate = randRange(prngState, steps);
                                ++guard;
                        }
                        lastRandomStep = candidate;
                        return candidate;
                }
                default:
                        return playCounter;
                }
        }

        void restoreCycleState() {
                cycleResetPending = false;
                prngState = baseSeed;
                if (algo)
                        algo->reset(baseSeed);
                lastPitch = 0.f;
                lastVel = 0.8f;
                lastRawPitch = 0.f;
                lastStepActive = false;
                lastGateFracApplied = clamp(gatePercent, 0.01f, 1.f);
                gateOut = false;
                gateTimer = 0.f;
                totalStepCount = 0;
                pingStep = 0;
                pingDir = 1;
                lastRandomStep = -1;
                phase = 0.f;
        }

        static int wrapIndex(int i, int m) {
                if (m <= 0)
                        return 0;
                int r = i % m;
                if (r < 0)
                        r += m;
                return r;
        }
};

} // namespace sitri

// -----------------------------------------------------------------------------
// Rack module implementation
// -----------------------------------------------------------------------------

struct Sitri : rack::engine::Module {
        struct RootNoteParamQuantity : rack::engine::ParamQuantity {
                std::string getDisplayValueString() override {
                        static const std::array<const char*, 12> names = {
                            "C",  "C#", "D",  "D#", "E",  "F",
                            "F#", "G",  "G#", "A",  "A#", "B"};
                        int idx = clamp((int)std::round(getValue()), 0, 11);
                        return names[idx];
                }
        };

        struct ScaleParamQuantity : rack::engine::ParamQuantity {
                std::string getDisplayValueString() override {
                        auto* sitri = dynamic_cast<Sitri*>(module);
                        if (!sitri)
                                return rack::engine::ParamQuantity::getDisplayValueString();
                        const auto& names = sitri->quantizer.scaleNames();
                        if (names.empty())
                                return rack::engine::ParamQuantity::getDisplayValueString();
                        int idx = clamp((int)std::round(getValue()), 0, (int)names.size() - 1);
                        return names[idx];
                }
        };

        struct AlgorithmParamQuantity : rack::engine::ParamQuantity {
                std::string getDisplayValueString() override {
                        auto* sitri = dynamic_cast<Sitri*>(module);
                        if (!sitri || sitri->algoIds.empty())
                                return rack::engine::ParamQuantity::getDisplayValueString();
                        int idx = clamp((int)std::round(getValue()), 0, (int)sitri->algoIds.size() - 1);
                        const std::string& id = sitri->algoIds[idx];
                        return sitri::AlgoRegistry::instance().getDisplayName(id);
                }
        };

        enum ParamIds {
                TYPE_PARAM,
                DENSITY_PARAM,
                STEPS_PARAM,
                OFFSET_PARAM,
                ACCENT_PARAM,
                GATE_PARAM,
                DIV_PARAM,
                DIR_PARAM,
                SWING_PARAM,
                ROOT_PARAM,
                SCALE_PARAM,
                TRANSPOSE_PARAM,
                SEED_PARAM,
                RESEED_PARAM,
                PARAMS_LEN
        };
        enum InputIds {
                CLOCK_INPUT,
                RESET_INPUT,
                ROOT_INPUT,
                TRANSPOSE_INPUT,
                DENSITY_INPUT,
                ACCENT_INPUT,
                GATE_INPUT,
                SEED_INPUT,
                INPUTS_LEN
        };
        enum OutputIds {
                PITCH_OUTPUT,
                GATE_OUTPUT,
                VEL_OUTPUT,
                EOC_OUTPUT,
                OUTPUTS_LEN
        };
        enum LightIds { ACTIVE_LIGHT, LIGHTS_LEN };

        sitri::SequencerCore core;
        sitri::Quantizer quantizer;
        std::string algoId = "raxdm";
        std::vector<std::string> algoIds;
        int lastAlgoIndex = -1;

        dsp::SchmittTrigger clockTrigger;
        dsp::SchmittTrigger resetTrigger;
        dsp::SchmittTrigger seedTrigger;
        dsp::SchmittTrigger reseedTrigger;

        float heldSeed = 0.f;
        int voltageRange = 2; // 0 = 1V, 1 = 5V, 2 = 10V (default 10V)

        Sitri() {
                config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

                algoIds = sitri::AlgoRegistry::instance().ids();
                if (algoIds.empty())
                        algoIds.push_back("raxdm");
                float algoMax = (float)std::max<int>(0, (int)algoIds.size() - 1);

                // Parameters with snapping for discrete values
                auto algoParam = configParam<AlgorithmParamQuantity>(TYPE_PARAM, 0.f, algoMax, 0.f, "Algorithm");
                algoParam->snapEnabled = true;
                configParam(DENSITY_PARAM, 0.f, 1.f, 0.6f, "Density", "%", 0.f, 100.f);

                auto stepsParam = configParam(STEPS_PARAM, 1.f, 64.f, 16.f, "Sequence Length", " steps");
                stepsParam->snapEnabled = true;

                auto offsetParam = configParam(OFFSET_PARAM, -64.f, 64.f, 0.f, "Step Offset", " steps");
                offsetParam->snapEnabled = true;

                configParam(ACCENT_PARAM, 0.f, 1.f, 0.5f, "Accent", "%", 0.f, 100.f);
                configParam(GATE_PARAM, 0.05f, 1.f, 0.5f, "Gate Length", "%", 0.f, 100.f);

                auto divParam = configParam(DIV_PARAM, -2.f, 4.f, 0.f, "Clock Division", "x", 2.f, 1.f);
                divParam->snapEnabled = true;

                configSwitch(DIR_PARAM, 0.f, 3.f, 0.f, "Direction", {"Forward", "Reverse", "Ping-Pong", "Random"});
                configParam(SWING_PARAM, 0.f, 0.6f, 0.f, "Swing", "%", 0.f, 100.f);

                auto rootParam = configParam<RootNoteParamQuantity>(ROOT_PARAM, 0.f, 11.f, 0.f, "Root Note");
                rootParam->snapEnabled = true;

                auto scaleParam = configParam<ScaleParamQuantity>(SCALE_PARAM, 0.f, (float)(quantizer.scaleNames().size() - 1), 0.f, "Scale");
                scaleParam->snapEnabled = true;

                auto transposeParam = configParam(TRANSPOSE_PARAM, -24.f, 24.f, 0.f, "Transpose", " semitones");
                transposeParam->snapEnabled = true;

                configParam(SEED_PARAM, 0.f, 1.f, 0.5f, "Random Seed", "%", 0.f, 100.f);
                configButton(RESEED_PARAM, "Reseed - Generate new pattern");

                // Inputs with proper names
                configInput(CLOCK_INPUT, "Clock");
                configInput(RESET_INPUT, "Reset");
                configInput(ROOT_INPUT, "Root Note CV");
                configInput(TRANSPOSE_INPUT, "Transpose CV");
                configInput(DENSITY_INPUT, "Density CV");
                configInput(ACCENT_INPUT, "Accent CV");
                configInput(GATE_INPUT, "Gate Length CV");
                configInput(SEED_INPUT, "Random Seed Trigger");

                // Algorithm Outputs
                configOutput(PITCH_OUTPUT, "Pitch CV (V/Oct) - Quantized melody from algorithm");
                configOutput(GATE_OUTPUT, "Gate/Trigger - Note on/off from algorithm");
                configOutput(VEL_OUTPUT, "Velocity CV (0-10V) - Note dynamics from algorithm");
                configOutput(EOC_OUTPUT, "End of Cycle Trigger - Fires when sequence loops");

                core.setQuantizer(&quantizer);
                setAlgorithm(algoId);
        }

        void onReset() override {
                core.reset(computeSeed());
        }

        uint64_t computeSeed() {
                uint64_t base = (uint64_t)std::round(params[SEED_PARAM].getValue() * 1000000.0f);
                uint64_t cv = (uint64_t)std::round(heldSeed * 9973.0f);
                return base ^ (cv << 16) ^ 0x9E3779B97F4A7C15ull;
        }

        void setAlgorithm(const std::string& id) {
                auto newAlgo = sitri::AlgoRegistry::instance().create(id);
                std::string resolved = id;
                if (!newAlgo) {
                        newAlgo = sitri::AlgoRegistry::instance().create("raxdm");
                        resolved = "raxdm";
                }
                core.setAlgorithm(std::move(newAlgo));
                algoId = resolved;
                auto it = std::find(algoIds.begin(), algoIds.end(), algoId);
                if (it != algoIds.end()) {
                        lastAlgoIndex = (int)std::distance(algoIds.begin(), it);
                        params[TYPE_PARAM].setValue((float)lastAlgoIndex);
                }
                core.reset(computeSeed());
        }

        void process(const ProcessArgs& args) override {
                int algoIndex = clamp((int)std::round(params[TYPE_PARAM].getValue()), 0, (int)algoIds.size() - 1);
                if (algoIndex != lastAlgoIndex && algoIndex >= 0 && algoIndex < (int)algoIds.size()) {
                        setAlgorithm(algoIds[algoIndex]);
                        lastAlgoIndex = algoIndex;
                }

                float density = params[DENSITY_PARAM].getValue();
                if (inputs[DENSITY_INPUT].isConnected())
                        density += inputs[DENSITY_INPUT].getVoltage() / 10.f;
                core.setDensity(clamp(density, 0.f, 1.f));

                int steps = (int)std::round(params[STEPS_PARAM].getValue());
                core.setSteps(steps);

                int offset = (int)std::round(params[OFFSET_PARAM].getValue());
                core.setOffset(offset);

                float accent = params[ACCENT_PARAM].getValue();
                if (inputs[ACCENT_INPUT].isConnected())
                        accent += inputs[ACCENT_INPUT].getVoltage() / 10.f;
                core.setAccent(clamp(accent, 0.f, 1.f));

                float gatePct = params[GATE_PARAM].getValue();
                if (inputs[GATE_INPUT].isConnected())
                        gatePct += inputs[GATE_INPUT].getVoltage() / 10.f;
                core.setGatePercent(clamp(gatePct, 0.05f, 1.f));

                float divPow = params[DIV_PARAM].getValue();
                float divHz = std::pow(2.f, divPow) * 2.f; // 0.5 .. 32 Hz (30 - 1920 BPM)
                core.setDivHz(divHz);

                int dir = clamp((int)std::round(params[DIR_PARAM].getValue()), 0, 3);
                core.setDirection(dir);
                core.setSwing(params[SWING_PARAM].getValue());

                float rootKnob = params[ROOT_PARAM].getValue() / 12.f;
                float rootCv = inputs[ROOT_INPUT].isConnected() ? inputs[ROOT_INPUT].getVoltage() : 0.f;
                quantizer.setRoot(rootCv + rootKnob);

                int scaleIndex = clamp((int)std::round(params[SCALE_PARAM].getValue()), 0, (int)quantizer.scaleNames().size() - 1);
                quantizer.setScaleIndex(scaleIndex);

                float transposeKnob = params[TRANSPOSE_PARAM].getValue() / 12.f;
                float transposeCv = inputs[TRANSPOSE_INPUT].isConnected() ? inputs[TRANSPOSE_INPUT].getVoltage() : 0.f;
                quantizer.setTranspose(transposeKnob + transposeCv);

                if (inputs[SEED_INPUT].isConnected()) {
                        if (seedTrigger.process(inputs[SEED_INPUT].getVoltage())) {
                                heldSeed = inputs[SEED_INPUT].getVoltage();
                                core.reset(computeSeed());
                        }
                }

                // Reseed button - the heart of the module
                if (reseedTrigger.process(params[RESEED_PARAM].getValue())) {
                        core.reset(computeSeed());
                }

                bool resetTrig = resetTrigger.process(inputs[RESET_INPUT].getVoltage());

                bool clockConnected = inputs[CLOCK_INPUT].isConnected();
                bool clockEdge = false;
                if (clockConnected) {
                        clockEdge = clockTrigger.process(inputs[CLOCK_INPUT].getVoltage());
                }

                if (resetTrig)
                        core.reset(computeSeed());

                core.process(args.sampleTime, clockEdge, clockConnected);

                // Apply voltage range attenuation (clamps to specified range)
                float pitchOut = core.pitchOut;

                switch (voltageRange) {
                case 0: // 1V range: clamp to ±0.5V
                        pitchOut = clamp(pitchOut, -0.5f, 0.5f);
                        break;
                case 1: // 5V range: clamp to ±2.5V
                        pitchOut = clamp(pitchOut, -2.5f, 2.5f);
                        break;
                case 2: // 10V range (default): no clamping
                default:
                        break;
                }

                outputs[PITCH_OUTPUT].setVoltage(pitchOut);
                outputs[GATE_OUTPUT].setVoltage(core.gateOut ? 10.f : 0.f);
                outputs[VEL_OUTPUT].setVoltage(core.velOut);
                outputs[EOC_OUTPUT].setVoltage(core.eocPulse ? 10.f : 0.f);

                lights[ACTIVE_LIGHT].setBrightness(core.gateOut ? 1.f : 0.f);
        }

        json_t* dataToJson() override {
                json_t* rootJ = json_object();
                json_object_set_new(rootJ, "voltageRange", json_integer(voltageRange));
                return rootJ;
        }

        void dataFromJson(json_t* rootJ) override {
                json_t* voltageRangeJ = json_object_get(rootJ, "voltageRange");
                if (voltageRangeJ)
                        voltageRange = json_integer_value(voltageRangeJ);
        }
};

// -----------------------------------------------------------------------------
// Widget
// -----------------------------------------------------------------------------

struct SitriWidget : rack::app::ModuleWidget {
        SitriWidget(Sitri* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Sitri.svg")));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                // Algorithm sequencer - clean, professional layout
                // 32HP module = 120.952mm width

                // === ALGORITHM SELECTION ===
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 20.0)), module, Sitri::TYPE_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.0, 20.0)), module, Sitri::SEED_PARAM));

                // === BIG ACID RESEED BUTTON (the heart of the module) ===
                addParam(createParamCentered<VCVButton>(mm2px(Vec(16.0, 30.0)), module, Sitri::RESEED_PARAM));

                // === SEQUENCE CONTROL ===
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.0, 38.0)), module, Sitri::STEPS_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(22.0, 38.0)), module, Sitri::OFFSET_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.0, 50.0)), module, Sitri::DIV_PARAM));
                addParam(createParamCentered<CKSSThree>(mm2px(Vec(22.0, 50.0)), module, Sitri::DIR_PARAM));

                // === MUSICAL EXPRESSION ===
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.0, 62.0)), module, Sitri::DENSITY_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(22.0, 62.0)), module, Sitri::ACCENT_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.0, 74.0)), module, Sitri::GATE_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(22.0, 74.0)), module, Sitri::SWING_PARAM));

                // === QUANTIZER ===
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.0, 86.0)), module, Sitri::ROOT_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(22.0, 86.0)), module, Sitri::SCALE_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(16.0, 97.0)), module, Sitri::TRANSPOSE_PARAM));

                // === CV INPUTS (White ports) ===
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 106.0)), module, Sitri::CLOCK_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(16.0, 106.0)), module, Sitri::RESET_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(22.0, 106.0)), module, Sitri::SEED_INPUT));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 113.5)), module, Sitri::DENSITY_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(16.0, 113.5)), module, Sitri::ACCENT_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(22.0, 113.5)), module, Sitri::GATE_INPUT));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.0, 121.0)), module, Sitri::ROOT_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(16.0, 121.0)), module, Sitri::TRANSPOSE_INPUT));

                // === OUTPUTS (Black ports) ===
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(28.0, 106.0)), module, Sitri::PITCH_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(28.0, 113.5)), module, Sitri::GATE_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(28.0, 121.0)), module, Sitri::VEL_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(22.0, 121.0)), module, Sitri::EOC_OUTPUT));

                // === ACTIVITY INDICATOR ===
                addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(16.0, 127.0)), module, Sitri::ACTIVE_LIGHT));
        }

        void appendContextMenu(Menu* menu) override {
                Sitri* module = getModule<Sitri>();
                if (!module)
                        return;

                menu->addChild(new MenuSeparator);
                menu->addChild(createMenuLabel("Pitch CV Range"));

                struct VoltageRangeItem : MenuItem {
                        Sitri* module;
                        int range;
                        void onAction(const event::Action& e) override {
                                module->voltageRange = range;
                        }
                        void step() override {
                                rightText = (module->voltageRange == range) ? "✔" : "";
                                MenuItem::step();
                        }
                };

                VoltageRangeItem* range1V = createMenuItem<VoltageRangeItem>("1V");
                range1V->module = module;
                range1V->range = 0;
                menu->addChild(range1V);

                VoltageRangeItem* range5V = createMenuItem<VoltageRangeItem>("5V");
                range5V->module = module;
                range5V->range = 1;
                menu->addChild(range5V);

                VoltageRangeItem* range10V = createMenuItem<VoltageRangeItem>("10V");
                range10V->module = module;
                range10V->range = 2;
                menu->addChild(range10V);

                menu->addChild(new MenuSeparator);
                menu->addChild(createMenuLabel("Root Note"));

                struct RootMenuItem : MenuItem {
                        Sitri* module = nullptr;
                        int index = 0;
                        void onAction(const event::Action&) override {
                                if (!module)
                                        return;
                                module->params[Sitri::ROOT_PARAM].setValue((float)index);
                        }
                        void step() override {
                                if (module && std::round(module->params[Sitri::ROOT_PARAM].getValue()) == index)
                                        rightText = "✔";
                                else
                                        rightText.clear();
                                MenuItem::step();
                        }
                };

                static const std::array<const char*, 12> rootNames = {
                    "C",  "C#", "D",  "D#", "E",  "F",
                    "F#", "G",  "G#", "A",  "A#", "B"};
                for (size_t i = 0; i < rootNames.size(); ++i) {
                        RootMenuItem* item = createMenuItem<RootMenuItem>(rootNames[i]);
                        item->module = module;
                        item->index = (int)i;
                        menu->addChild(item);
                }

                menu->addChild(new MenuSeparator);
                menu->addChild(createMenuLabel("Scale"));

                struct ScaleMenuItem : MenuItem {
                        Sitri* module = nullptr;
                        int index = 0;
                        void onAction(const event::Action&) override {
                                if (!module)
                                        return;
                                module->params[Sitri::SCALE_PARAM].setValue((float)index);
                                module->quantizer.setScaleIndex(index);
                        }
                        void step() override {
                                if (module && module->quantizer.getScaleIndex() == index)
                                        rightText = "✔";
                                else
                                        rightText.clear();
                                MenuItem::step();
                        }
                };

                const auto& scaleNames = module->quantizer.scaleNames();
                for (size_t i = 0; i < scaleNames.size(); ++i) {
                        ScaleMenuItem* item = createMenuItem<ScaleMenuItem>(scaleNames[i]);
                        item->module = module;
                        item->index = (int)i;
                        menu->addChild(item);
                }
        }
};

Model* modelSitri = createModel<Sitri, SitriWidget>("Sitri");
