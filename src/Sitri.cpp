#include "plugin.hpp"
#include "SitriBus.hpp"

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
        float detune = 0.f; // Post-quantization detune in volts (e.g., 0.01 = ~12 cents)
};

struct AlgoContext {
        int stepIndex = 0;
        int steps = 16;
        float density = 0.5f;
        float accent = 0.5f;
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
                // Allow random jumps across the full 10V range (±5V)
                // Mix small steps with occasional large octave jumps
                float step;
                if (rand01(ctx.prngState) < 0.3f) {
                        // 30% chance of large octave jump (±4 octaves)
                        step = (rand01(ctx.prngState) - 0.5f) * 8.0f;
                } else {
                        // 70% chance of smaller melodic movement
                        step = (rand01(ctx.prngState) - 0.5f) * 2.0f;
                }
                e.pitch = clamp(ctx.lastPitch + step, -5.0f, 5.0f);
                e.vel = 0.4f + 0.6f * rand01(ctx.prngState);
                // Varied gate lengths: short stabs, medium notes, long sustained
                float gateRoll = rand01(ctx.prngState);
                if (gateRoll < 0.25f) {
                        e.gateFrac = 0.15f + 0.15f * rand01(ctx.prngState); // Short staccato
                } else if (gateRoll < 0.65f) {
                        e.gateFrac = 0.35f + 0.25f * rand01(ctx.prngState); // Medium
                } else {
                        e.gateFrac = 0.65f + 0.35f * rand01(ctx.prngState); // Long sustained
                }
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
                // Allow center to drift across wider range for octave exploration
                center += (rand01(ctx.prngState) - 0.5f) * 0.5f;
                center = clamp(center, -4.0f, 4.0f); // Keep center within ±4 octaves
                float target = 0.7f * center + 0.3f * ctx.lastPitch;
                // Add larger expressive variations
                e.pitch = clamp(target + (rand01(ctx.prngState) - 0.5f) * 1.5f, -5.0f, 5.0f);
                e.vel = 0.5f + 0.5f * ctx.accent * rand01(ctx.prngState);
                // Accrete has flowing, varied gate lengths - sometimes sustained, sometimes shorter
                float gateRoll = rand01(ctx.prngState);
                if (gateRoll < 0.3f) {
                        e.gateFrac = 0.2f + 0.2f * rand01(ctx.prngState); // Short
                } else if (gateRoll < 0.7f) {
                        e.gateFrac = 0.45f + 0.25f * rand01(ctx.prngState); // Medium
                } else {
                        e.gateFrac = 0.7f + 0.3f * rand01(ctx.prngState); // Long sustained
                }
                return e;
        }
};
REGISTER_ALGO("acxom", AlgoAccrete);

struct AlgoAcid : public IAlgorithm {
        int runningDegree = 0;
        int stepsSinceRecenter = 0;
        const char* id() const override { return "xacidx"; }
        const char* displayName() const override { return "XACIDx"; }

        void reset(uint64_t) override {
                runningDegree = 0;
                stepsSinceRecenter = 0;
        }

        StepEvent generate(const AlgoContext& c) override {
                AlgoContext ctx = c;
                StepEvent e;
                e.active = rand01(ctx.prngState) < (0.75f + 0.25f * ctx.density);

                // Recenter periodically to prevent drift
                stepsSinceRecenter++;
                if (stepsSinceRecenter > 16 || std::abs(runningDegree) > 24) {
                        // Pull back towards center
                        if (runningDegree > 12) {
                                runningDegree -= randChoice({3, 5, 7, 12}, ctx.prngState);
                        } else if (runningDegree < -12) {
                                runningDegree += randChoice({3, 5, 7, 12}, ctx.prngState);
                        }
                        stepsSinceRecenter = 0;
                }

                // Classic acid movement: mostly small steps with occasional jumps
                int motion;
                float moveRoll = rand01(ctx.prngState);
                if (moveRoll < 0.6f) {
                        // Small chromatic steps
                        motion = randChoice({-1, 0, 1}, ctx.prngState);
                } else if (moveRoll < 0.85f) {
                        // Scale-based jumps
                        motion = randChoice({-5, -3, 3, 5, 7}, ctx.prngState);
                } else {
                        // Octave jump
                        motion = randChoice({-12, 12}, ctx.prngState);
                }

                runningDegree = clamp(runningDegree + motion, -36, 36); // ±3 octaves
                e.pitch = degToVolts(runningDegree);

                // Velocity with accent
                e.vel = 0.65f + 0.35f * ctx.accent;
                if (rand01(ctx.prngState) < ctx.accent) {
                        e.vel += 0.15f * rand01(ctx.prngState);
                }
                e.vel = clamp(e.vel, 0.f, 1.f);

                // Acid: Classic acid slides vs short stabs
                float gateRoll = rand01(ctx.prngState);
                if (gateRoll < 0.35f) {
                        e.gateFrac = 0.88f + 0.12f * rand01(ctx.prngState); // Long slide
                } else if (gateRoll < 0.75f) {
                        e.gateFrac = 0.4f + 0.2f * rand01(ctx.prngState); // Medium
                } else {
                        e.gateFrac = 0.1f + 0.15f * rand01(ctx.prngState); // Short stab
                }
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
                float improvProbability = clamp(0.08f + 0.32f, 0.f, 1.f);

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
                // More frequent octave jumps for anchors
                if (anchor && rand01(ctx.prngState) < (0.45f + 0.35f * ctx.accent))
                        degree += randChoice({12, 24, -12}, ctx.prngState);
                // More dramatic ghost note variations
                if (ghost && rand01(ctx.prngState) < 0.12f)
                        degree += randChoice({-24, -12, 12, 24}, ctx.prngState);

                float expressiveNudge = (rand01(ctx.prngState) - 0.5f) * 0.12f;
                e.pitch = degToVolts(degree + leadOffset) + expressiveNudge;

                // Wider leadOffset range for octave exploration (±60 semitones = 5 octaves)
                if (anchor && rand01(ctx.prngState) < 0.3f) {
                        leadOffset = clamp(leadOffset + randChoice({-12, -7, -5, 5, 7, 12}, ctx.prngState), -60, 60);
                }

                float baseVel = anchor ? 0.8f : (ghost ? 0.58f : 0.48f);
                baseVel += accentShape[idx] * 0.22f * ctx.accent;
                baseVel += (rand01(ctx.prngState) - 0.5f) * 0.1f;
                e.vel = clamp(baseVel, 0.f, 1.f);

                // STING Pulse: Varied gate patterns - anchors can be long or punchy, ghosts are quick
                float gateRoll = rand01(ctx.prngState);
                if (anchor) {
                        // Anchors: mix of sustained and punchy
                        if (gateRoll < 0.4f) {
                                e.gateFrac = 0.7f + 0.25f * rand01(ctx.prngState); // Long
                        } else if (gateRoll < 0.75f) {
                                e.gateFrac = 0.45f + 0.2f * rand01(ctx.prngState); // Medium
                        } else {
                                e.gateFrac = 0.15f + 0.15f * rand01(ctx.prngState); // Short punch
                        }
                } else if (ghost) {
                        // Ghost notes: mostly short with occasional medium
                        if (gateRoll < 0.7f) {
                                e.gateFrac = 0.1f + 0.15f * rand01(ctx.prngState); // Very short
                        } else {
                                e.gateFrac = 0.3f + 0.2f * rand01(ctx.prngState); // Medium ghost
                        }
                } else {
                        // Fill notes: varied
                        e.gateFrac = 0.2f + 0.5f * rand01(ctx.prngState);
                }

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

                if (!hit && rand01(ctx.prngState) < 0.15f)
                        hit = true;

                e.active = hit;
                e.prob = hit ? 1.f : 0.f;

                if (hit) {
                        int motion = accent ? randChoice({7, 5, 12, 24}, ctx.prngState)
                                            : randChoice({0, 2, -3, 3}, ctx.prngState);
                        if (rand01(ctx.prngState) < 0.1f)
                                motion = randChoice({-24, -12, -7, 12, 14, 24}, ctx.prngState);
                        // Expand to full 10V range (±60 semitones)
                        runningDegree = clamp(runningDegree + motion, -60, 60);
                        e.pitch = degToVolts(runningDegree);

                        float vel = accent ? 0.88f : 0.6f;
                        vel += 0.22f * ctx.accent;
                        vel += (rand01(ctx.prngState) - 0.5f) * 0.12f;
                        e.vel = clamp(vel, 0.f, 1.f);

                        // STING Swarm: Swarming rhythms with varied gate lengths
                        float gateRoll = rand01(ctx.prngState);
                        if (accent) {
                                // Accented notes: sustained or punchy
                                if (gateRoll < 0.5f) {
                                        e.gateFrac = 0.75f + 0.25f * rand01(ctx.prngState); // Long
                                } else {
                                        e.gateFrac = 0.45f + 0.25f * rand01(ctx.prngState); // Medium-long
                                }
                        } else {
                                // Non-accent: quick buzzing swarm notes with some variety
                                if (gateRoll < 0.4f) {
                                        e.gateFrac = 0.15f + 0.2f * rand01(ctx.prngState); // Short
                                } else if (gateRoll < 0.75f) {
                                        e.gateFrac = 0.4f + 0.2f * rand01(ctx.prngState); // Medium
                                } else {
                                        e.gateFrac = 0.65f + 0.2f * rand01(ctx.prngState); // Occasional long
                                }
                        }
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
                int idx = ctx.stepIndex % length;
                bool hit = euclidHit(idx, length, pulses);

                e.active = hit;
                e.prob = hit ? 1.f : 0.f;

                if (hit) {
                        bool accent = euclidHit((idx + length / 3) % length, length, std::max(1, pulses / 2));
                        int motion = accent ? randChoice({7, 5, 12, 24}, ctx.prngState)
                                            : randChoice({0, 2, -2, -5}, ctx.prngState);
                        if (rand01(ctx.prngState) < 0.1f)
                                motion = randChoice({-24, -12, 12, 24}, ctx.prngState);
                        // Expand to full 10V range (±60 semitones)
                        lastDegree = clamp(lastDegree + motion, -60, 60);
                        e.pitch = degToVolts(lastDegree);

                        float vel = accent ? 0.9f : 0.6f;
                        vel += 0.2f * ctx.accent;
                        vel += (rand01(ctx.prngState) - 0.5f) * 0.1f;
                        e.vel = clamp(vel, 0.f, 1.f);

                        // EUCLIDEAN: Groovy patterns with varied gate lengths
                        float gateRoll = rand01(ctx.prngState);
                        if (accent) {
                                // Accents: mostly sustained with occasional punch
                                if (gateRoll < 0.6f) {
                                        e.gateFrac = 0.75f + 0.25f * rand01(ctx.prngState); // Long
                                } else if (gateRoll < 0.85f) {
                                        e.gateFrac = 0.5f + 0.2f * rand01(ctx.prngState); // Medium
                                } else {
                                        e.gateFrac = 0.2f + 0.2f * rand01(ctx.prngState); // Short accent
                                }
                        } else {
                                // Regular beats: mix of all lengths for groove
                                if (gateRoll < 0.35f) {
                                        e.gateFrac = 0.15f + 0.2f * rand01(ctx.prngState); // Short
                                } else if (gateRoll < 0.75f) {
                                        e.gateFrac = 0.4f + 0.25f * rand01(ctx.prngState); // Medium
                                } else {
                                        e.gateFrac = 0.7f + 0.25f * rand01(ctx.prngState); // Long
                                }
                        }
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

struct AlgoHypnotic : public IAlgorithm {
        const char* id() const override { return "hypno"; }
        const char* displayName() const override { return "HYPNOTIC"; }

        struct HypnoticStep {
                bool active;
                int degree;
                float vel;
                float gateFrac;
                bool isDetuned;
        };

        void reset(uint64_t seed) override {
                // Generate a FIXED hypnotic pattern that will loop identically
                uint64_t state = seed ? seed : 1;

                // Pick which step will be the detuned "hypnotic" note (typically position 3, 5, or 7 in 8-step)
                detunedStep = randChoice({3, 5, 7}, state);

                // Pick the detune amount (slightly off to create tension)
                detuneAmount = 0.15f + rand01(state) * 0.25f; // 15-40 cents sharp
                if (rand01(state) < 0.3f) {
                        detuneAmount = -detuneAmount; // Sometimes flat instead
                }

                baseDegree = 0;

                // Pre-generate the entire 16-step pattern for perfect repetition
                static constexpr std::array<bool, 16> rhythmMask = {
                    true, true, true, true, true, true, true, false,
                    true, true, true, true, true, true, true, false};

                for (int i = 0; i < 16; ++i) {
                        HypnoticStep& step = pattern[i];
                        int patternIdx = i % 8;
                        bool isDetunedNote = patternIdx == detunedStep;

                        // Determine if step is active (deterministic based on density and rhythm)
                        float hitProb = rhythmMask[i] ? 0.95f : 0.1f;
                        step.active = rand01(state) < hitProb;

                        if (step.active) {
                                // Generate fixed melodic pattern
                                if (isDetunedNote) {
                                        // The hypnotic detuned note - pick one permanently
                                        step.degree = randChoice({5, 7}, state);
                                } else {
                                        // Regular notes follow simple pattern
                                        switch (patternIdx) {
                                        case 0:
                                        case 4:
                                                step.degree = 0; // Root
                                                break;
                                        case 1:
                                        case 5:
                                                step.degree = rand01(state) < 0.7f ? 0 : 2;
                                                break;
                                        case 2:
                                        case 6:
                                                step.degree = randChoice({0, 2, 4}, state);
                                                break;
                                        case 3:
                                        case 7:
                                                step.degree = randChoice({0, 5}, state);
                                                break;
                                        default:
                                                step.degree = 0;
                                        }
                                }

                                // Occasional octave jumps (fixed per step)
                                if (rand01(state) < 0.1f) {
                                        step.degree += randChoice({-12, 12}, state);
                                }

                                // Fixed velocity per step (with slight variation per step, not per play)
                                if (isDetunedNote) {
                                        step.vel = 0.85f + 0.1f * rand01(state);
                                } else {
                                        step.vel = 0.55f + 0.15f * rand01(state);
                                }

                                // Fixed gate length per step
                                if (isDetunedNote) {
                                        step.gateFrac = 0.7f + 0.25f * rand01(state);
                                } else {
                                        step.gateFrac = 0.5f + 0.2f * rand01(state);
                                }

                                step.isDetuned = isDetunedNote;
                        }
                }
        }

        StepEvent generate(const AlgoContext& c) override {
                StepEvent e;

                // Force pattern to work best with 8 or 16 steps
                int effectiveSteps = (c.steps <= 8) ? 8 : 16;
                int patternIdx = c.stepIndex % effectiveSteps;

                // Look up the pre-generated pattern
                const HypnoticStep& step = pattern[patternIdx];

                e.active = step.active;

                if (e.active) {
                        e.pitch = degToVolts(step.degree + baseDegree);

                        // Apply detune to the special hypnotic note (post-quantization)
                        e.detune = step.isDetuned ? detuneAmount : 0.f;

                        // Apply accent control to velocity but keep pattern consistent
                        e.vel = step.vel * (0.5f + 0.5f * c.accent);
                        e.vel = clamp(e.vel, 0.f, 1.f);

                        e.gateFrac = step.gateFrac;
                } else {
                        e.pitch = c.lastPitch;
                        e.vel = 0.4f;
                        e.gateFrac = 0.3f;
                        e.detune = 0.f;
                }

                return e;
        }

private:
        int detunedStep = 3;
        float detuneAmount = 0.2f;
        int baseDegree = 0;
        std::array<HypnoticStep, 16> pattern;
};
REGISTER_ALGO("hypno", AlgoHypnotic);

struct AlgoHypnoticEvolve : public IAlgorithm {
        const char* id() const override { return "hypnoev"; }
        const char* displayName() const override { return "HYPNO EVOLVE"; }

        void reset(uint64_t seed) override {
                uint64_t state = seed ? seed : 1;

                // Pick which step will be the detuned "hypnotic" note
                detunedStep = randChoice({3, 5, 7}, state);

                // Pick the detune amount
                detuneAmount = 0.15f + rand01(state) * 0.25f;
                if (rand01(state) < 0.3f) {
                        detuneAmount = -detuneAmount;
                }

                baseDegree = 0;
                evolutionCounter = 0;

                // Generate initial pattern
                generatePattern(state);
        }

        void generatePattern(uint64_t& state) {
                static constexpr std::array<bool, 16> rhythmMask = {
                    true, true, true, true, true, true, true, false,
                    true, true, true, true, true, true, true, false};

                for (int i = 0; i < 16; ++i) {
                        HypnoticStep& step = pattern[i];
                        int patternIdx = i % 8;
                        bool isDetunedNote = patternIdx == detunedStep;

                        float hitProb = rhythmMask[i] ? 0.95f : 0.1f;
                        step.active = rand01(state) < hitProb;

                        if (step.active) {
                                if (isDetunedNote) {
                                        step.degree = randChoice({5, 7}, state);
                                } else {
                                        switch (patternIdx) {
                                        case 0:
                                        case 4:
                                                step.degree = 0;
                                                break;
                                        case 1:
                                        case 5:
                                                step.degree = rand01(state) < 0.7f ? 0 : 2;
                                                break;
                                        case 2:
                                        case 6:
                                                step.degree = randChoice({0, 2, 4}, state);
                                                break;
                                        case 3:
                                        case 7:
                                                step.degree = randChoice({0, 5}, state);
                                                break;
                                        default:
                                                step.degree = 0;
                                        }
                                }

                                if (rand01(state) < 0.1f) {
                                        step.degree += randChoice({-12, 12}, state);
                                }

                                if (isDetunedNote) {
                                        step.vel = 0.85f + 0.1f * rand01(state);
                                } else {
                                        step.vel = 0.55f + 0.15f * rand01(state);
                                }

                                if (isDetunedNote) {
                                        step.gateFrac = 0.7f + 0.25f * rand01(state);
                                } else {
                                        step.gateFrac = 0.5f + 0.2f * rand01(state);
                                }

                                step.isDetuned = isDetunedNote;
                        }
                }
        }

        StepEvent generate(const AlgoContext& c) override {
                StepEvent e;

                int effectiveSteps = (c.steps <= 8) ? 8 : 16;
                int patternIdx = c.stepIndex % effectiveSteps;

                // Evolve the pattern every 4-8 loops based on density
                if (patternIdx == 0) {
                        evolutionCounter++;
                        int evolutionInterval = (int)(4 + 4 * (1.0f - c.density));
                        if (evolutionCounter >= evolutionInterval) {
                                evolutionCounter = 0;
                                // Subtle evolution: mutate 1-2 steps
                                uint64_t evolveState = c.prngState;
                                int mutateCount = randChoice({1, 2}, evolveState);
                                for (int m = 0; m < mutateCount; ++m) {
                                        int mutateIdx = randRange(evolveState, 16);
                                        // Don't mutate the detuned step - keep it sacred
                                        if ((mutateIdx % 8) != detunedStep && pattern[mutateIdx].active) {
                                                // Subtle degree shift
                                                int shift = randChoice({-2, 0, 0, 2}, evolveState);
                                                pattern[mutateIdx].degree = clamp(pattern[mutateIdx].degree + shift, -12, 12);
                                        }
                                }
                        }
                }

                const HypnoticStep& step = pattern[patternIdx];
                e.active = step.active;

                if (e.active) {
                        e.pitch = degToVolts(step.degree + baseDegree);
                        e.detune = step.isDetuned ? detuneAmount : 0.f;
                        e.vel = step.vel * (0.5f + 0.5f * c.accent);
                        e.vel = clamp(e.vel, 0.f, 1.f);
                        e.gateFrac = step.gateFrac;
                } else {
                        e.pitch = c.lastPitch;
                        e.vel = 0.4f;
                        e.gateFrac = 0.3f;
                        e.detune = 0.f;
                }

                return e;
        }

private:
        struct HypnoticStep {
                bool active;
                int degree;
                float vel;
                float gateFrac;
                bool isDetuned;
        };

        int detunedStep = 3;
        float detuneAmount = 0.2f;
        int baseDegree = 0;
        std::array<HypnoticStep, 16> pattern;
        int evolutionCounter = 0;
};
REGISTER_ALGO("hypnoev", AlgoHypnoticEvolve);

// -----------------------------------------------------------------------------
// Algorithm Documentation
// -----------------------------------------------------------------------------
/*
 * SITRI ALGORITHM GUIDE
 * =====================
 *
 * This module provides various generative algorithms for creating melodic sequences.
 * Each algorithm has a different character and use case.
 *
 * ALGORITHMS:
 * -----------
 *
 * RAxDOM (Random)
 * - Character: Chaotic, exploratory, unpredictable
 * - Use when: You want wild melodic jumps and experimental sequences
 * - Features: Large octave jumps mixed with small steps, varied gate lengths
 * - Density control: Affects note probability
 * - Best for: Experimental music, ambient textures, generative chaos
 *
 * ACxEOM (Accrete)
 * - Character: Drifting, organic, evolving around a center point
 * - Use when: You want melodies that wander but stay coherent
 * - Features: Center point that drifts slowly, notes orbit around it
 * - Density control: Affects hit probability (0.6-0.8 recommended)
 * - Best for: Ambient, evolving pads, organic melodic movement
 *
 * XACIDx (Acid)
 * - Character: Classic TB-303 style acid sequences
 * - Use when: You want squelchy, funky acid basslines
 * - Features: Chromatic steps, scale jumps, octave leaps, periodic recentering
 * - Accent control: Heavily affects velocity for classic acid accents
 * - Best for: Acid techno, electro, funky basslines
 * - Gate tip: Mix of slides (long) and stabs (short) for authentic acid feel
 *
 * STING Pulse
 * - Character: Complex polyrhythmic patterns with anchor notes
 * - Use when: You want intricate, musical sequences with ghost notes
 * - Features: Anchor beats, ghost notes, improvisation fills, accent shaping
 * - Density control: Affects ghost note and fill probability
 * - Best for: Techno, house, complex melodic sequences
 *
 * STING Swarm
 * - Character: Euclidean rhythms with swarming melodic movement
 * - Use when: You want mathematically perfect rhythmic patterns
 * - Features: Euclidean hit distribution, phrase rotation, accent layer
 * - Density control: Directly affects euclidean pulse count
 * - Best for: Hypnotic techno, polyrhythmic sequences, rhythmic exploration
 *
 * EUCLIDEAN
 * - Character: Pure euclidean rhythm with melodic movement
 * - Use when: You want mathematically distributed rhythms
 * - Features: Euclidean distribution, accent layer, scale-based melody
 * - Density control: Determines number of hits in the pattern
 * - Best for: Grooves, polyrhythms, world music-inspired patterns
 *
 * HYPNOTIC
 * - Character: Locked, repetitive patterns with one detuned note
 * - Use when: You want a hypnotic, trance-inducing loop with tension
 * - Features:
 *   * 100% identical loops - pattern never changes
 *   * One note is detuned 15-40 cents for hypnotic tension
 *   * Focuses on 8/16 step sequences
 *   * Simple, repetitive melodic patterns
 *   * The detuned note is accented in velocity and gate length
 * - Density control: Fixed pattern, density doesn't affect much
 * - Best for: Minimal techno, hypnotic house, trance, repetitive grooves
 * - Technical: Uses post-quantization detune (see developer notes below)
 *
 * HYPNO EVOLVE
 * - Character: Hypnotic patterns that slowly evolve over time
 * - Use when: You want hypnotic loops that subtly change
 * - Features:
 *   * Starts like HYPNOTIC with locked pattern
 *   * Slowly evolves: 1-2 notes mutate every 4-8 loops
 *   * Evolution speed controlled by density (low = slow evolution)
 *   * The sacred detuned note NEVER changes - always the anchor
 *   * Mutations are subtle: ±2 semitone shifts
 * - Density control: Higher density = faster evolution
 * - Best for: Long evolving tracks, hypnotic grooves that develop, live performance
 * - Technical: Uses post-quantization detune (see developer notes below)
 *
 * PARAMETER GUIDE:
 * ----------------
 * - Density: Algorithm-specific, usually controls note probability or pattern complexity
 * - Accent: Affects velocity shaping and accent patterns
 * - Gate: Global gate length multiplier (applied to algorithm's gate suggestions)
 * - Steps: Sequence length (some algorithms optimize for specific lengths)
 * - Offset: Rotates the sequence playback position
 * - Direction: Forward, Reverse, Ping-Pong, Random
 * - Swing: Adds groove by lengthening odd steps
 * - Division: Clock speed multiplier/divider
 *
 * DEVELOPER NOTES - POST-QUANTIZATION DETUNE:
 * -------------------------------------------
 * The StepEvent structure includes a 'detune' field for algorithms that need
 * to intentionally detune notes AFTER quantization. This is crucial for
 * algorithms like HYPNOTIC that rely on microtonal detuning for their effect.
 *
 * How it works:
 * 1. Algorithm generates pitch as usual (scale degrees)
 * 2. Sequencer quantizes pitch to selected scale
 * 3. Sequencer then adds the 'detune' value (in volts)
 * 4. Final pitch = quantized_pitch + detune
 *
 * Usage in your algorithm:
 *   StepEvent e;
 *   e.pitch = degToVolts(5);        // Generate a fifth
 *   e.detune = 0.02f;                // Detune by ~24 cents (0.02V = 24 cents)
 *
 * The detune is preserved when:
 * - User changes scale/root
 * - Quantizer settings are modified
 * - Pattern loops
 *
 * Typical detune values:
 * - 0.01V = ~12 cents (subtle)
 * - 0.02V = ~24 cents (noticeable)
 * - 0.03V = ~36 cents (strong tension)
 * - 0.05V = ~60 cents (quarter-tone)
 *
 * Note: Use detune sparingly! Most algorithms should set detune = 0.
 *       It's specifically for creating intentional harmonic tension.
 */

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

        int getCurrentStepIndex() const { return currentStepIndex; }
        int getStepCount() const { return steps; }
        int getStepsAdvancedThisFrame() const { return stepsAdvancedThisFrame; }
        bool takeStepEdge() {
                bool edge = stepEdge;
                stepEdge = false;
                return edge;
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
                lastDetune = 0.f;
                lastStepActive = false;
                lastGateFracApplied = clamp(gatePercent, 0.01f, 1.f);
                quantizerRevision = quantizer ? quantizer->getRevision() : 0;
                clockPeriod = 0.5f;
                timeSinceLastClock = 0.f;
                lastRandomStep = -1;
                cycleResetPending = false;
                externalClockActive = false;
                externalClockDivCounter = 0;
                externalClockMultCounter = 0;
                currentStepIndex = 0;
                stepEdge = false;
                stepsAdvancedThisFrame = 0;
                stepHistoryWriteIndex = 0;
                for (int i = 0; i < 16; ++i) {
                        stepHistory[i].pitch = 0.f;
                        stepHistory[i].gate = false;
                        stepHistory[i].newNote = false;
                        stepHistory[i].stepIndex = -1;  // Mark as invalid initially
                }
                if (algo)
                        algo->reset(actualSeed);
        }

        void restart() {
                // Restart the sequence from the beginning WITHOUT changing the seed
                cycleResetPending = true;
                currentStepIndex = 0;
                stepEdge = false;
        }

        uint64_t getSeed() const {
                return baseSeed;
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
                        externalClockMultCounter = 0;
                }
                currentStepIndex = clamp(currentStepIndex, 0, steps - 1);
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
                // Calculate clock division/multiplication for external clocks
                // divHz < 2Hz = slower (need multiple clocks per step)
                // divHz > 2Hz = faster (generate multiple steps per clock)
                if (divHz < 2.0f) {
                        // Slower: need multiple incoming clocks before advancing one step
                        externalClockDivAmount = 1;
                        externalClockMultAmount = std::max(1, (int)std::round(2.0f / divHz));
                } else {
                        // Faster or equal: generate multiple steps per clock using internal subdivision
                        // When external clock connected, we'll subdivide the clock period
                        externalClockDivAmount = std::max(1, (int)std::round(divHz / 2.0f));
                        externalClockMultAmount = 1;
                }
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
                        externalClockMultCounter = 0;
                }
        }

        void process(float sampleTime, bool clockEdge, bool externalClockConnected) {
                eocPulse = false;
                stepEdge = false;
                stepsAdvancedThisFrame = 0;

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
                        externalClockMultCounter = 0;
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

                        // Handle clock multiplication (slower speeds - need multiple clocks per step)
                        if (externalClockMultAmount > 1) {
                                // Don't reset phase - we're not subdividing
                                phase = 0.f;
                                externalClockMultCounter++;
                                if (externalClockMultCounter >= externalClockMultAmount) {
                                        externalClockMultCounter = 0;
                                        advanceStep(true); // true = external clock
                                }
                        } else {
                                // At base speed or faster - advance on every clock
                                // Don't reset phase if we're subdividing (externalClockDivAmount > 1)
                                if (externalClockDivAmount == 1) {
                                        phase = 0.f;
                                }
                                advanceStep(true);
                        }
                } else if (!externalClockConnected) {
                        // Internal clock - use divHz directly
                        phase += sampleTime;
                        while (phase >= currentStepDuration()) {
                                phase -= currentStepDuration();
                                advanceStep(false); // false = internal clock
                        }
                } else if (externalClockConnected && externalClockDivAmount > 1) {
                        // External clock connected and we want faster speeds
                        // Subdivide the clock period to generate multiple steps per clock
                        phase += sampleTime;
                        float subdivisionPeriod = clockPeriod / (float)externalClockDivAmount;
                        // Use while loop to handle accumulated phase properly (prevents skipped steps)
                        while (phase >= subdivisionPeriod) {
                                phase -= subdivisionPeriod;
                                advanceStep(true);
                        }
                }
        }

        bool gateOut = false;
        float pitchOut = 0.f;
        float velOut = 0.f;
        bool eocPulse = false;
        bool newNoteTrigger = false;  // True when a new note starts (pitch changed or gate re-triggered)

        uint64_t prngState = 0x12345678abcdefULL;

        // Step history for high-speed capture support
        struct StepHistoryEntry {
                float pitch = 0.f;
                bool gate = false;
                bool newNote = false;
                int stepIndex = 0;
        };
        std::array<StepHistoryEntry, 16> stepHistory;  // 16 slots for safety margin
        int stepHistoryWriteIndex = 0;

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

        int currentStepIndex = 0;
        bool stepEdge = false;
        int stepsAdvancedThisFrame = 0;  // Track multiple steps per process() call

        float gateTimer = 0.f;
        float currentDuration = 0.5f;
        float clockPeriod = 0.5f;
        float timeSinceLastClock = 0.f;
        float lastRawPitch = 0.f;
        float lastDetune = 0.f;
        float lastGateFracApplied = 0.5f;
        bool lastStepActive = false;
        uint64_t quantizerRevision = 0;
        uint64_t baseSeed = 0x12345678abcdefULL;
        bool cycleResetPending = false;
        bool externalClockActive = false;
        int externalClockDivCounter = 0;
        int externalClockDivAmount = 1;
        int externalClockMultAmount = 1;
        int externalClockMultCounter = 0;

        float currentStepDuration() const {
                return currentDuration;
        }

        void onQuantizerChanged() {
                quantizerRevision = quantizer->getRevision();
                if (!lastStepActive)
                        return;
                float newPitch = quantizer->snap(lastRawPitch);
                // Apply the detune after quantization
                float finalPitch = newPitch + lastDetune;
                if (std::fabs(finalPitch - pitchOut) < 1e-5f)
                        return;

                float gateFrac = clamp(lastGateFracApplied, 0.01f, 1.f);
                gateOut = true;
                gateTimer = currentStepDuration() * gateFrac;
                pitchOut = finalPitch;
                lastPitch = finalPitch;
                velOut = lastVel * 10.f;
        }

        void advanceStep(bool externalClock = false) {
                if (!algo || !quantizer || steps <= 0)
                        return;

                if (cycleResetPending)
                        restoreCycleState();

                int baseStep = computeBaseStep();

                // Calculate the actual step we're generating (with offset rotation)
                int rotatedIndex = wrapIndex(baseStep + offset, steps);

                currentStepIndex = rotatedIndex;
                stepEdge = true;
                stepsAdvancedThisFrame++;

                // Build context for current step
                AlgoContext ctx;
                ctx.stepIndex = rotatedIndex; // Use rotated index for pattern lookup (offset support)
                ctx.steps = steps;
                ctx.density = density;
                ctx.accent = accent;
                ctx.prngState = prngState;
                ctx.lastPitch = lastPitch;
                ctx.lastVel = lastVel;
                ctx.phase01 = 0.f;
                ctx.divHz = divHz;

                StepEvent proposal = algo->generate(ctx);
                prngState = ctx.prngState;

                bool active = proposal.active;
                // Apply probability (algorithms already handle density internally)
                if (proposal.prob < 1.f && rand01(prngState) > proposal.prob)
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
                // Don't reset phase here - it needs to continue accumulating for subdivisions

                if (active) {
                        float shapedVel = proposal.vel * (0.4f + 0.6f * accent);
                        shapedVel = clamp(shapedVel, 0.f, 1.f);
                        float rawPitch = proposal.pitch;
                        float snapped = quantizer->snap(rawPitch);
                        // Apply post-quantization detune (for algorithms like HYPNOTIC)
                        float finalPitch = snapped + proposal.detune;

                        // Detect if this is a new note (pitch changed or gate was off)
                        bool pitchChanged = std::fabs(finalPitch - lastPitch) > 1e-5f;
                        bool gateWasOff = !gateOut;
                        newNoteTrigger = pitchChanged || gateWasOff;

                        pitchOut = finalPitch;
                        velOut = shapedVel * 10.f;
                        gateOut = true;
                        float gateFrac = clamp(proposal.gateFrac * gatePercent, 0.01f, 1.f);
                        gateTimer = currentDuration * gateFrac;
                        lastGateFracApplied = gateFrac;
                        lastRawPitch = rawPitch;
                        lastDetune = proposal.detune; // Store detune for quantizer changes
                        lastStepActive = true;
                        quantizerRevision = quantizer->getRevision();
                        lastPitch = finalPitch;
                        lastVel = shapedVel;

                        // Store in history buffer for Lilith capture at high speeds
                        stepHistory[stepHistoryWriteIndex].pitch = finalPitch;
                        stepHistory[stepHistoryWriteIndex].gate = true;
                        stepHistory[stepHistoryWriteIndex].newNote = newNoteTrigger;
                        stepHistory[stepHistoryWriteIndex].stepIndex = currentStepIndex;
                        stepHistoryWriteIndex = (stepHistoryWriteIndex + 1) % 16;
                } else {
                        // Inactive step - turn off gate immediately
                        gateOut = false;
                        gateTimer = 0.f;
                        lastStepActive = false;
                        lastDetune = 0.f;
                        newNoteTrigger = false;

                        // Store in history buffer for Lilith capture at high speeds
                        stepHistory[stepHistoryWriteIndex].pitch = lastPitch;
                        stepHistory[stepHistoryWriteIndex].gate = false;
                        stepHistory[stepHistoryWriteIndex].newNote = false;
                        stepHistory[stepHistoryWriteIndex].stepIndex = currentStepIndex;
                        stepHistoryWriteIndex = (stepHistoryWriteIndex + 1) % 16;
                }

                advanceCounters();
                totalStepCount++;
        }

        void advanceCounters() {
                switch (direction) {
                case 0: // forward
                        // Check if we're about to wrap before incrementing
                        if ((playCounter + 1) >= steps) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        playCounter = (playCounter + 1) % steps;
                        break;
                case 1: // reverse
                        // Check if we're about to wrap before incrementing
                        if ((playCounter + 1) >= steps) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        playCounter = (playCounter + 1) % steps;
                        break;
                case 2: // pingpong
                        if (steps <= 1) {
                                eocPulse = true;
                                cycleResetPending = true;
                                break;
                        }
                        // Move in current direction
                        pingStep += pingDir;

                        // Check if we need to reverse direction at the end
                        if (pingStep >= steps - 1) {
                                pingStep = steps - 1;
                                pingDir = -1;
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        // Check if we need to reverse direction at the beginning
                        else if (pingStep <= 0) {
                                pingStep = 0;
                                pingDir = 1;
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        playCounter = (playCounter + 1) % steps;
                        break;
                case 3: // random
                        // Check if we're about to wrap before incrementing
                        if ((playCounter + 1) >= steps) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        playCounter = (playCounter + 1) % steps;
                        break;
                default:
                        // Check if we're about to wrap before incrementing
                        if ((playCounter + 1) >= steps) {
                                eocPulse = true;
                                cycleResetPending = true;
                        }
                        playCounter = (playCounter + 1) % steps;
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
                lastDetune = 0.f;
                lastStepActive = false;
                lastGateFracApplied = clamp(gatePercent, 0.01f, 1.f);
                gateOut = false;
                gateTimer = 0.f;
                totalStepCount = 0;
                pingStep = 0;
                pingDir = 1;
                lastRandomStep = -1;
                phase = 0.f;
                currentStepIndex = clamp(currentStepIndex, 0, steps - 1);
                stepEdge = false;
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
                RESEED_PARAM,
                RUN_PARAM,
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
                INPUTS_LEN
        };
        enum OutputIds {
                PITCH_OUTPUT,
                GATE_OUTPUT,
                VEL_OUTPUT,
                EOC_OUTPUT,
                OUTPUTS_LEN
        };
        enum LightIds { ACTIVE_LIGHT, RUN_LIGHT, LIGHTS_LEN };

        sitri::SequencerCore core;
        sitri::Quantizer quantizer;
        std::string algoId = "raxdm";
        std::vector<std::string> algoIds;
        int lastAlgoIndex = -1;

        dsp::SchmittTrigger clockTrigger;
        dsp::SchmittTrigger resetTrigger;
        dsp::SchmittTrigger reseedTrigger;
        dsp::SchmittTrigger runTrigger;
        std::random_device randomDevice; // For true random seed generation
        bool seedLoaded = false; // Track if seed was loaded from JSON
        bool running = true; // Run/stop state
        bool reseedTriggered = false; // Track reseed button press for expander

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

                configButton(RESEED_PARAM, "Random - Generate completely new random sequence");
                configButton(RUN_PARAM, "Run/Stop");

                // Inputs with proper names
                configInput(CLOCK_INPUT, "Clock");
                configInput(RESET_INPUT, "Reset");
                configInput(ROOT_INPUT, "Root Note CV");
                configInput(TRANSPOSE_INPUT, "Transpose CV");
                configInput(DENSITY_INPUT, "Density CV");
                configInput(ACCENT_INPUT, "Accent CV");
                configInput(GATE_INPUT, "Gate Length CV");

                // Algorithm Outputs
                configOutput(PITCH_OUTPUT, "Pitch CV (V/Oct) - Quantized melody from algorithm");
                configOutput(GATE_OUTPUT, "Gate/Trigger - Note on/off from algorithm");
                configOutput(VEL_OUTPUT, "Velocity CV (0-10V) - Note dynamics from algorithm");
                configOutput(EOC_OUTPUT, "End of Cycle Trigger - Fires when sequence loops");

                core.setQuantizer(&quantizer);
                setAlgorithm(algoId);

                // Expander messages are initialized but not registered here
                // Sitri will write directly to Lilith's producer buffer in process()
        }

        void onReset() override {
                // Only generate new seed if one wasn't loaded from JSON
                if (!seedLoaded) {
                        core.reset(computeSeed());
                        seedLoaded = true; // Mark that we now have a seed
                }
        }

        uint64_t computeSeed() {
                // Generate truly random seed using random_device
                return ((uint64_t)randomDevice() << 32) | randomDevice();
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
                // Don't reset seed here - let onReset() handle initialization
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

                // Run/Stop button - toggle running state
                if (runTrigger.process(params[RUN_PARAM].getValue())) {
                        running = !running;
                }

                // Reseed button - generates a completely new random sequence
                reseedTriggered = false;
                if (reseedTrigger.process(params[RESEED_PARAM].getValue())) {
                        // Generate truly random seed using random_device
                        uint64_t newSeed = ((uint64_t)randomDevice() << 32) | randomDevice();
                        // Ensure we get a non-zero seed
                        if (newSeed == 0) {
                                newSeed = 0x12345678abcdefULL;
                        }
                        core.reset(newSeed);
                        seedLoaded = true; // Mark that we have a valid seed
                        reseedTriggered = true; // Signal to expander
                }

                bool resetTrig = resetTrigger.process(inputs[RESET_INPUT].getVoltage());

                bool clockConnected = inputs[CLOCK_INPUT].isConnected();
                bool clockEdge = false;
                if (clockConnected) {
                        // Always process clock trigger to keep state updated
                        clockEdge = clockTrigger.process(inputs[CLOCK_INPUT].getVoltage());
                        // But only use the edge if running
                        if (!running) {
                                clockEdge = false;
                        }
                }

                if (resetTrig)
                        core.restart();  // Restart sequence from beginning without reseeding

                // Update run light
                lights[RUN_LIGHT].setBrightness(running ? 1.f : 0.f);

                // Only process if running
                if (running) {
                        core.process(args.sampleTime, clockEdge, clockConnected);

                        // Output when running
                        outputs[PITCH_OUTPUT].setVoltage(core.pitchOut);
                        outputs[GATE_OUTPUT].setVoltage(core.gateOut ? 10.f : 0.f);
                        outputs[VEL_OUTPUT].setVoltage(core.velOut);
                        outputs[EOC_OUTPUT].setVoltage(core.eocPulse ? 10.f : 0.f);
                        lights[ACTIVE_LIGHT].setBrightness(core.gateOut ? 1.f : 0.f);
                } else {
                        // When stopped, turn off gate and clear outputs
                        outputs[GATE_OUTPUT].setVoltage(0.f);
                        outputs[EOC_OUTPUT].setVoltage(0.f);
                        lights[ACTIVE_LIGHT].setBrightness(0.f);
                        // Keep pitch and velocity at last values
                }

                bool stepEdge = core.takeStepEdge();
                int coreSteps = core.getStepCount();
                int busSteps = clamp(coreSteps, 1, 16);  // Support up to 16 steps for LilithAdvance
                int stepIndex = core.getCurrentStepIndex();
                if (busSteps > 0) {
                        stepIndex %= busSteps;
                        if (stepIndex < 0)
                                stepIndex += busSteps;
                } else {
                        busSteps = 1;
                        stepIndex = 0;
                }

                // Write to Lilith expander using VCV Rack's message flipping system
                Module* rightModule = getRightExpander().module;
                bool connectedToLilith = rightModule && rightModule->model &&
                                        (rightModule->model->slug == "Lilith" ||
                                         rightModule->model->slug == "LilithAdvance");

                if (connectedToLilith) {
                        // Write to Lilith's producer buffer (VCV will flip it to consumer on Lilith's side)
                        auto* msg = static_cast<SitriBus::MasterToExpander*>(rightModule->leftExpander.producerMessage);
                        if (msg) {
                                msg->magic = SitriBus::MAGIC;
                                msg->version = 1;
                                msg->running = running ? 1 : 0;
                                msg->numSteps = (uint8_t)busSteps;
                                msg->stepIndex = (uint8_t)clamp(stepIndex + 1, 1, busSteps);
                                msg->resetEdge = resetTrig ? 1 : 0;
                                msg->clockEdge = stepEdge ? 1 : 0;
                                msg->eocPulse = core.eocPulse ? 1 : 0;
                                msg->reseedEdge = reseedTriggered ? 1 : 0;
                                msg->stepsAdvanced = (uint8_t)core.getStepsAdvancedThisFrame();

                                // Send current step's pitch, gate, and note trigger status to Lilith
                                msg->currentPitch = core.pitchOut;
                                msg->currentGate = core.gateOut ? 1 : 0;
                                msg->newNote = core.newNoteTrigger ? 1 : 0;

                                // Send global parameters
                                msg->gateLength = params[GATE_PARAM].getValue();

                                // Send step history for high-speed capture
                                // Clear the step history buffer first to avoid stale data
                                for (int i = 0; i < 16; ++i) {
                                        msg->stepHistory[i].pitch = 0.f;
                                        msg->stepHistory[i].gate = 0;
                                        msg->stepHistory[i].newNote = 0;
                                        msg->stepHistory[i].valid = 0;  // Mark as empty
                                }

                                // Iterate backwards from writeIndex - 1 (most recent) to find unique step data
                                // Only send the FIRST (newest) occurrence of each step index
                                bool stepSeen[16] = {false};  // Track which step indices we've already sent (supports up to 16 steps)
                                for (int offset = 0; offset < 16; ++offset) {
                                        // Read backwards: most recent entry is at writeIndex - 1
                                        int histIdx = (core.stepHistoryWriteIndex - 1 - offset + 16) % 16;
                                        int stepIdx = core.stepHistory[histIdx].stepIndex;
                                        if (stepIdx >= 0) {  // Valid entry in history buffer
                                                int actualStepIndex = stepIdx % busSteps;
                                                if (actualStepIndex >= 0 && actualStepIndex < 16 && !stepSeen[actualStepIndex]) {
                                                        msg->stepHistory[actualStepIndex].pitch = core.stepHistory[histIdx].pitch;
                                                        msg->stepHistory[actualStepIndex].gate = core.stepHistory[histIdx].gate ? 1 : 0;
                                                        msg->stepHistory[actualStepIndex].newNote = core.stepHistory[histIdx].newNote ? 1 : 0;
                                                        msg->stepHistory[actualStepIndex].valid = 1;  // Mark as valid
                                                        stepSeen[actualStepIndex] = true;  // Mark this step as sent
                                                }
                                        }
                                }

                                // Request VCV Rack to flip the buffers
                                rightModule->leftExpander.requestMessageFlip();
                        }
                }
        }

        json_t* dataToJson() override {
                json_t* rootJ = json_object();
                // Save the current seed so the sequence is preserved across sessions
                uint64_t seed = core.getSeed();
                json_object_set_new(rootJ, "seed", json_integer(seed));
                // Save run/stop state
                json_object_set_new(rootJ, "running", json_boolean(running));
                return rootJ;
        }

        void dataFromJson(json_t* rootJ) override {
                // Restore the saved seed to get the same sequence
                json_t* seedJ = json_object_get(rootJ, "seed");
                if (seedJ) {
                        uint64_t savedSeed = json_integer_value(seedJ);
                        core.reset(savedSeed);
                        seedLoaded = true; // Mark that we loaded a seed so onReset() doesn't overwrite it
                }
                // Restore run/stop state (defaults to true if not found)
                json_t* runningJ = json_object_get(rootJ, "running");
                if (runningJ) {
                        running = json_boolean_value(runningJ);
                } else {
                        running = true; // Default to running for old patches
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
			auto svg = APP->window->loadSvg(asset::plugin(pluginInstance, "res/Sitri.svg"));
			if (svg) {
				svgWidget->setSvg(svg);
			} else {
				WARN("SVG returned null: res/Sitri.svg");
			}
		} catch (const std::exception& e) {
			WARN("Exception loading SVG res/Sitri.svg: %s", e.what());
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

// -----------------------------------------------------------------------------
// Widget
// -----------------------------------------------------------------------------

struct SitriWidget : rack::app::ModuleWidget {
        SitriWidget(Sitri* module) {
                setModule(module);
                 // Set panel size (6HP module: 30.48mm width, 128.5mm height)
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Sitri.svg")));

                auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size;
		addChild(bg);

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                // Algorithm sequencer - clean, professional layout

               // --- Layout helpers ----------------------------------------------------------
                constexpr float COL1 = 6.5f;   // left column (mm)
                constexpr float COL2 = 16.0f;  // center column (mm) -> panel center on a 6HP (≈30.48mm) panel
                constexpr float COL3 = 25.5f;  // right column (mm)

                auto P = [](float x, float y) { return mm2px(Vec(x, y)); };

                // Vertical rhythm (mm). Adjust these together if you tweak overall density.
                constexpr float Y_TOP      = 18.0f;
                constexpr float Y_TRANSPORT= 30.0f;
                constexpr float Y_SEQ_1    = 44.0f;
                constexpr float Y_SEQ_2    = 56.0f;
                constexpr float Y_EXPR_1   = 70.0f;
                constexpr float Y_EXPR_2   = 82.0f;
                constexpr float Y_IO_1     = 96.0f;
                constexpr float Y_IO_2     = 105.0f;

                // === ALGORITHM / MODE (Hero control) ========================================
                addParam(createParamCentered<RoundBlackKnob>(P(COL1, Y_TOP), module, Sitri::TYPE_PARAM));

                // === TRANSPORT & STATUS ======================================================
                addParam(createParamCentered<VCVButton>(P(COL1, Y_TRANSPORT), module, Sitri::RUN_PARAM));
                addChild(createLightCentered<SmallLight<GreenLight>>(P(COL1, Y_TRANSPORT - 5.5f), module, Sitri::RUN_LIGHT)); // tiny LED above RUN
                addParam(createParamCentered<CKSSThree>(P(COL2, Y_TRANSPORT), module, Sitri::DIR_PARAM));

                addChild(createLightCentered<MediumLight<GreenLight>>(P(COL3, Y_TRANSPORT - 5.5f), module, Sitri::ACTIVE_LIGHT));        // heartbeat/status in the center column

                addParam(createParamCentered<LEDButton>(P(COL3, Y_TRANSPORT), module, Sitri::RESEED_PARAM));

                // === SEQUENCE CONTROL (2×2 block) ===========================================
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL1, Y_SEQ_1), module, Sitri::STEPS_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL3, Y_SEQ_1), module, Sitri::OFFSET_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL1, Y_SEQ_2), module, Sitri::DIV_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL3, Y_SEQ_2), module, Sitri::TRANSPOSE_PARAM));

                // === MUSICAL EXPRESSION (2×2 block) =========================================
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL1, Y_EXPR_1), module, Sitri::DENSITY_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL3, Y_EXPR_1), module, Sitri::ACCENT_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL1, Y_EXPR_2), module, Sitri::GATE_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL2, Y_EXPR_1), module, Sitri::SWING_PARAM));

                // === QUANTIZER (paired + center transpose) ==================================
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL2, Y_SEQ_1), module, Sitri::ROOT_PARAM));
                addParam(createParamCentered<RoundSmallBlackKnob>(P(COL2, Y_SEQ_2), module, Sitri::SCALE_PARAM));


                // === CV INPUTS (white) – left side ==========================================
                addInput(createInputCentered<PJ301MPort>(P(COL1, Y_IO_1), module, Sitri::CLOCK_INPUT));
                addInput(createInputCentered<PJ301MPort>(P(COL2, Y_IO_1), module, Sitri::RESET_INPUT));

                addInput(createInputCentered<PJ301MPort>(P(COL1, Y_IO_2), module, Sitri::DENSITY_INPUT));
                addInput(createInputCentered<PJ301MPort>(P(COL2, Y_IO_2), module, Sitri::ACCENT_INPUT));
                addInput(createInputCentered<PJ301MPort>(P(COL3, Y_IO_2), module, Sitri::GATE_INPUT));

                // Extra pitch controls just above the bottom, still on the left half
                addInput(createInputCentered<PJ301MPort>(P(COL1, Y_IO_2 + 9.0f), module, Sitri::ROOT_INPUT));
                addInput(createInputCentered<PJ301MPort>(P(COL2, Y_IO_2 + 9.0f), module, Sitri::TRANSPOSE_INPUT));

                // === OUTPUTS (black) – right column =========================================
                addOutput(createOutputCentered<DarkPJ301MPort>(P(COL3, Y_IO_1), module, Sitri::PITCH_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(P(COL3, Y_IO_1 + 9.0f), module, Sitri::GATE_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(P(COL3, Y_IO_2 + 9.0f), module, Sitri::VEL_OUTPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(P(COL3, Y_EXPR_2), module, Sitri::EOC_OUTPUT)); // tucked upper right
        }

        void appendContextMenu(Menu* menu) override {
                Sitri* module = getModule<Sitri>();
                if (!module)
                        return;

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
