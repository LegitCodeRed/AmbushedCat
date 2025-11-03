#include "plugin.hpp"
#include "effects/compressor.h"
#include "framework/value.h"
#include "utilities/smooth_value.h"
#include <cmath>
#include <algorithm>
#include <array>

using namespace rack;

struct SabnockOTT : Module {
	enum ParamId {
		PARAM_ATTACK,          // Attack time
		PARAM_RELEASE,         // Release time
		PARAM_MIX,             // Dry/Wet mix (was depth)
		PARAM_LOW_GAIN,        // Low band output gain
		PARAM_MID_GAIN,        // Mid band output gain
		PARAM_HIGH_GAIN,       // High band output gain
		PARAM_INPUT,           // Input gain
		PARAM_OUTPUT,          // Output gain
		// Low band controls
		PARAM_LOW_UP_THRESH,   // Low band upward threshold
		PARAM_LOW_UP_RATIO,    // Low band upward ratio
		PARAM_LOW_DOWN_THRESH, // Low band downward threshold
		PARAM_LOW_DOWN_RATIO,  // Low band downward ratio
		// Mid band controls
		PARAM_MID_UP_THRESH,   // Mid band upward threshold
		PARAM_MID_UP_RATIO,    // Mid band upward ratio
		PARAM_MID_DOWN_THRESH, // Mid band downward threshold
		PARAM_MID_DOWN_RATIO,  // Mid band downward ratio
		// High band controls
		PARAM_HIGH_UP_THRESH,  // High band upward threshold
		PARAM_HIGH_UP_RATIO,   // High band upward ratio
		PARAM_HIGH_DOWN_THRESH,// High band downward threshold
		PARAM_HIGH_DOWN_RATIO, // High band downward ratio
		NUM_PARAMS
	};
	enum InputId {
		INPUT_L,
		INPUT_R,
		// Main control CV inputs
		INPUT_ATTACK_CV,
		INPUT_RELEASE_CV,
		INPUT_MIX_CV,
		// Band gain CV inputs
		INPUT_LOW_GAIN_CV,
		INPUT_MID_GAIN_CV,
		INPUT_HIGH_GAIN_CV,
		// I/O gain CV inputs
		INPUT_INPUT_GAIN_CV,
		INPUT_OUTPUT_GAIN_CV,
		// Low band CV inputs
		INPUT_LOW_UP_THRESH_CV,
		INPUT_LOW_UP_RATIO_CV,
		INPUT_LOW_DOWN_THRESH_CV,
		INPUT_LOW_DOWN_RATIO_CV,
		// Mid band CV inputs
		INPUT_MID_UP_THRESH_CV,
		INPUT_MID_UP_RATIO_CV,
		INPUT_MID_DOWN_THRESH_CV,
		INPUT_MID_DOWN_RATIO_CV,
		// High band CV inputs
		INPUT_HIGH_UP_THRESH_CV,
		INPUT_HIGH_UP_RATIO_CV,
		INPUT_HIGH_DOWN_THRESH_CV,
		INPUT_HIGH_DOWN_RATIO_CV,
		NUM_INPUTS
	};
	enum OutputId {
		OUTPUT_L,
		OUTPUT_R,
		NUM_OUTPUTS
	};
	enum LightId {
		NUM_LIGHTS
	};

	// Vital DSP components
	std::unique_ptr<vital::MultibandCompressor> compressor;
	std::unique_ptr<vital::Output> sig_in;
	std::array<vital::Value*, 19> vals = {};

	float in_gain = 1.0f;
	float out_gain = 1.0f;

	float crossover1Freq = 120.f;  // Low/Mid split
	float crossover2Freq = 2500.f; // Mid/High split

	// Previous threshold values for tracking changes
	float prev_ll_thres = -35.0f;
	float prev_lu_thres = -28.5f;
	float prev_bl_thres = -36.0f;
	float prev_bu_thres = -25.0f;
	float prev_hl_thres = -35.0f;
	float prev_hu_thres = -30.0f;

	SabnockOTT() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

		// Main controls (top row)
		configParam(PARAM_ATTACK, 0.f, 1.f, 0.5f, "Attack", "%", 0.f, 100.f);
		configParam(PARAM_RELEASE, 0.f, 1.f, 0.5f, "Release", "%", 0.f, 100.f);
		configParam(PARAM_MIX, 0.f, 1.f, 1.0f, "Mix", "%", 0.f, 100.f);

		// Band gain controls (makeup gain per band)
		configParam(PARAM_LOW_GAIN, -30.f, 30.f, 16.3f, "Low Band Gain", " dB");
		configParam(PARAM_MID_GAIN, -30.f, 30.f, 11.7f, "Mid Band Gain", " dB");
		configParam(PARAM_HIGH_GAIN, -30.f, 30.f, 16.3f, "High Band Gain", " dB");

		// Input/Output gain
		configParam(PARAM_INPUT, -60.f, 30.f, 0.f, "Input Gain", " dB");
		configParam(PARAM_OUTPUT, -60.f, 30.f, 0.f, "Output Gain", " dB");

		// LOW BAND (Top = Downward/Upper, Bottom = Upward/Lower)
		// Top knob: Downward (compression) - Upper threshold
		configParam(PARAM_LOW_DOWN_THRESH, -79.f, -1.f, -28.5f, "Low Downward Threshold", " dB");
		configParam(PARAM_LOW_DOWN_RATIO, 0.f, 1.f, 0.9f, "Low Downward Ratio", "%", 0.f, 100.f);
		// Bottom knob: Upward (expansion) - Lower threshold
		configParam(PARAM_LOW_UP_THRESH, -79.f, -1.f, -35.0f, "Low Upward Threshold", " dB");
		configParam(PARAM_LOW_UP_RATIO, 0.f, 1.f, 0.8f, "Low Upward Ratio", "%", 0.f, 100.f);

		// MID BAND (Top = Downward/Upper, Bottom = Upward/Lower)
		// Top knob: Downward (compression) - Upper threshold
		configParam(PARAM_MID_DOWN_THRESH, -79.f, -1.f, -25.0f, "Mid Downward Threshold", " dB");
		configParam(PARAM_MID_DOWN_RATIO, 0.f, 1.f, 0.857f, "Mid Downward Ratio", "%", 0.f, 100.f);
		// Bottom knob: Upward (expansion) - Lower threshold
		configParam(PARAM_MID_UP_THRESH, -79.f, -1.f, -36.0f, "Mid Upward Threshold", " dB");
		configParam(PARAM_MID_UP_RATIO, 0.f, 1.f, 0.8f, "Mid Upward Ratio", "%", 0.f, 100.f);

		// HIGH BAND (Top = Downward/Upper, Bottom = Upward/Lower)
		// Top knob: Downward (compression) - Upper threshold
		configParam(PARAM_HIGH_DOWN_THRESH, -79.f, -1.f, -30.0f, "High Downward Threshold", " dB");
		configParam(PARAM_HIGH_DOWN_RATIO, 0.f, 1.f, 1.0f, "High Downward Ratio", "%", 0.f, 100.f);
		// Bottom knob: Upward (expansion) - Lower threshold
		configParam(PARAM_HIGH_UP_THRESH, -79.f, -1.f, -35.0f, "High Upward Threshold", " dB");
		configParam(PARAM_HIGH_UP_RATIO, 0.f, 1.f, 0.8f, "High Upward Ratio", "%", 0.f, 100.f);

		// Audio I/O
		configInput(INPUT_L, "Left");
		configInput(INPUT_R, "Right");
		configOutput(OUTPUT_L, "Left");
		configOutput(OUTPUT_R, "Right");

		// Main control CV inputs
		configInput(INPUT_ATTACK_CV, "Attack CV");
		configInput(INPUT_RELEASE_CV, "Release CV");
		configInput(INPUT_MIX_CV, "Mix CV");

		// Band gain CV inputs
		configInput(INPUT_LOW_GAIN_CV, "Low Band Gain CV");
		configInput(INPUT_MID_GAIN_CV, "Mid Band Gain CV");
		configInput(INPUT_HIGH_GAIN_CV, "High Band Gain CV");

		// I/O gain CV inputs
		configInput(INPUT_INPUT_GAIN_CV, "Input Gain CV");
		configInput(INPUT_OUTPUT_GAIN_CV, "Output Gain CV");

		// Low band CV inputs
		configInput(INPUT_LOW_UP_THRESH_CV, "Low Upward Threshold CV");
		configInput(INPUT_LOW_UP_RATIO_CV, "Low Upward Ratio CV");
		configInput(INPUT_LOW_DOWN_THRESH_CV, "Low Downward Threshold CV");
		configInput(INPUT_LOW_DOWN_RATIO_CV, "Low Downward Ratio CV");

		// Mid band CV inputs
		configInput(INPUT_MID_UP_THRESH_CV, "Mid Upward Threshold CV");
		configInput(INPUT_MID_UP_RATIO_CV, "Mid Upward Ratio CV");
		configInput(INPUT_MID_DOWN_THRESH_CV, "Mid Downward Threshold CV");
		configInput(INPUT_MID_DOWN_RATIO_CV, "Mid Downward Ratio CV");

		// High band CV inputs
		configInput(INPUT_HIGH_UP_THRESH_CV, "High Upward Threshold CV");
		configInput(INPUT_HIGH_UP_RATIO_CV, "High Upward Ratio CV");
		configInput(INPUT_HIGH_DOWN_THRESH_CV, "High Downward Threshold CV");
		configInput(INPUT_HIGH_DOWN_RATIO_CV, "High Downward Ratio CV");

		// Initialize vital DSP
		compressor = std::make_unique<vital::MultibandCompressor>();
		sig_in = std::make_unique<vital::Output>(vital::kMaxBufferSize);  // Allocate with proper buffer size
		compressor->plug(sig_in.get(), vital::MultibandCompressor::kAudio);

		// Initialize sample rate BEFORE setting up values
		onSampleRateChange();

		initVals();
		for (int i = 0; i < vals.size(); i++) {
			compressor->plug(vals[i], i + 1);
		}

		// Clear all buffers after initialization
		sig_in->clearBuffer();
		compressor->reset(vital::constants::kFullMask);
		updateParams();
	}

	void initVals() {
		for (int i = 0; i < vals.size(); i++) {
			vals[i] = new vital::SmoothValue(0);
		}
		vals[vital::MultibandCompressor::kEnabledBands - 1]->set(vital::MultibandCompressor::kMultiband);
	}

	void updateParams() {
		// Get parameters from Rack with CV modulation
		// Attack/Release: CV is 0-10V, map to 0-1 range (0.1 per volt)
		float attackParam = params[PARAM_ATTACK].getValue();
		attackParam += inputs[INPUT_ATTACK_CV].getVoltage() * 0.1f;
		attackParam = clamp(attackParam, 0.0f, 1.0f);

		float releaseParam = params[PARAM_RELEASE].getValue();
		releaseParam += inputs[INPUT_RELEASE_CV].getVoltage() * 0.1f;
		releaseParam = clamp(releaseParam, 0.0f, 1.0f);

		// Mix: CV is 0-10V, map to 0-1 range (0.1 per volt)
		float mix = params[PARAM_MIX].getValue();
		mix += inputs[INPUT_MIX_CV].getVoltage() * 0.1f;
		mix = clamp(mix, 0.0f, 1.0f);

		// I/O Gains: CV is 0-10V, map to -60 to +30 dB range (9 dB per volt)
		in_gain = params[PARAM_INPUT].getValue();
		in_gain += inputs[INPUT_INPUT_GAIN_CV].getVoltage() * 9.0f;
		in_gain = clamp(in_gain, -60.0f, 30.0f);

		out_gain = params[PARAM_OUTPUT].getValue();
		out_gain += inputs[INPUT_OUTPUT_GAIN_CV].getVoltage() * 9.0f;
		out_gain = clamp(out_gain, -60.0f, 30.0f);

		// Calculate attack/release times (scale from 0-1 with exponential curve)
		// Using exponential scaling for more musical response
		float att_time = std::pow(attackParam, 2.0f);   // 0-1 range
		float rel_time = std::pow(releaseParam, 2.0f);  // 0-1 range

		// Get band gains from controls with CV modulation
		// CV is 0-10V, map to -30 to +30 dB range (6 dB per volt)
		float lgain = params[PARAM_LOW_GAIN].getValue();
		lgain += inputs[INPUT_LOW_GAIN_CV].getVoltage() * 6.0f;
		lgain = clamp(lgain, -30.0f, 30.0f);

		float mgain = params[PARAM_MID_GAIN].getValue();
		mgain += inputs[INPUT_MID_GAIN_CV].getVoltage() * 6.0f;
		mgain = clamp(mgain, -30.0f, 30.0f);

		float hgain = params[PARAM_HIGH_GAIN].getValue();
		hgain += inputs[INPUT_HIGH_GAIN_CV].getVoltage() * 6.0f;
		hgain = clamp(hgain, -30.0f, 30.0f);

		// Get thresholds from controls with CV modulation
		// Lower threshold = upward compression (expansion) - bottom knob
		// Upper threshold = downward compression - top knob
		// CV is 0-10V, map to -79 to -1 dB range (7.8 dB per volt)
		float ll_thres = params[PARAM_LOW_UP_THRESH].getValue();
		ll_thres += inputs[INPUT_LOW_UP_THRESH_CV].getVoltage() * 7.8f;
		ll_thres = clamp(ll_thres, -79.0f, -1.0f);

		float lu_thres = params[PARAM_LOW_DOWN_THRESH].getValue();
		lu_thres += inputs[INPUT_LOW_DOWN_THRESH_CV].getVoltage() * 7.8f;
		lu_thres = clamp(lu_thres, -79.0f, -1.0f);

		float bl_thres = params[PARAM_MID_UP_THRESH].getValue();
		bl_thres += inputs[INPUT_MID_UP_THRESH_CV].getVoltage() * 7.8f;
		bl_thres = clamp(bl_thres, -79.0f, -1.0f);

		float bu_thres = params[PARAM_MID_DOWN_THRESH].getValue();
		bu_thres += inputs[INPUT_MID_DOWN_THRESH_CV].getVoltage() * 7.8f;
		bu_thres = clamp(bu_thres, -79.0f, -1.0f);

		float hl_thres = params[PARAM_HIGH_UP_THRESH].getValue();
		hl_thres += inputs[INPUT_HIGH_UP_THRESH_CV].getVoltage() * 7.8f;
		hl_thres = clamp(hl_thres, -79.0f, -1.0f);

		float hu_thres = params[PARAM_HIGH_DOWN_THRESH].getValue();
		hu_thres += inputs[INPUT_HIGH_DOWN_THRESH_CV].getVoltage() * 7.8f;
		hu_thres = clamp(hu_thres, -79.0f, -1.0f);

		// Enforce threshold constraint: upper threshold must be >= lower threshold
		// Detect which knob changed and push the other one

		// LOW BAND
		bool ll_changed = (ll_thres != prev_ll_thres);
		bool lu_changed = (lu_thres != prev_lu_thres);

		if (ll_changed && ll_thres > lu_thres) {
			// Lower moved up past upper, push upper up
			lu_thres = ll_thres;
			params[PARAM_LOW_DOWN_THRESH].setValue(lu_thres);
		} else if (lu_changed && lu_thres < ll_thres) {
			// Upper moved down past lower, push lower down
			ll_thres = lu_thres;
			params[PARAM_LOW_UP_THRESH].setValue(ll_thres);
		}
		prev_ll_thres = ll_thres;
		prev_lu_thres = lu_thres;

		// MID BAND
		bool bl_changed = (bl_thres != prev_bl_thres);
		bool bu_changed = (bu_thres != prev_bu_thres);

		if (bl_changed && bl_thres > bu_thres) {
			// Lower moved up past upper, push upper up
			bu_thres = bl_thres;
			params[PARAM_MID_DOWN_THRESH].setValue(bu_thres);
		} else if (bu_changed && bu_thres < bl_thres) {
			// Upper moved down past lower, push lower down
			bl_thres = bu_thres;
			params[PARAM_MID_UP_THRESH].setValue(bl_thres);
		}
		prev_bl_thres = bl_thres;
		prev_bu_thres = bu_thres;

		// HIGH BAND
		bool hl_changed = (hl_thres != prev_hl_thres);
		bool hu_changed = (hu_thres != prev_hu_thres);

		if (hl_changed && hl_thres > hu_thres) {
			// Lower moved up past upper, push upper up
			hu_thres = hl_thres;
			params[PARAM_HIGH_DOWN_THRESH].setValue(hu_thres);
		} else if (hu_changed && hu_thres < hl_thres) {
			// Upper moved down past lower, push lower down
			hl_thres = hu_thres;
			params[PARAM_HIGH_UP_THRESH].setValue(hl_thres);
		}
		prev_hl_thres = hl_thres;
		prev_hu_thres = hu_thres;

		// Get ratios from controls with CV modulation
		// CV is 0-10V, map to 0-1 range (0.1 per volt)
		float ll_ratio = params[PARAM_LOW_UP_RATIO].getValue();
		ll_ratio += inputs[INPUT_LOW_UP_RATIO_CV].getVoltage() * 0.1f;
		ll_ratio = clamp(ll_ratio, 0.0f, 1.0f);

		float lu_ratio = params[PARAM_LOW_DOWN_RATIO].getValue();
		lu_ratio += inputs[INPUT_LOW_DOWN_RATIO_CV].getVoltage() * 0.1f;
		lu_ratio = clamp(lu_ratio, 0.0f, 1.0f);

		float bl_ratio = params[PARAM_MID_UP_RATIO].getValue();
		bl_ratio += inputs[INPUT_MID_UP_RATIO_CV].getVoltage() * 0.1f;
		bl_ratio = clamp(bl_ratio, 0.0f, 1.0f);

		float bu_ratio = params[PARAM_MID_DOWN_RATIO].getValue();
		bu_ratio += inputs[INPUT_MID_DOWN_RATIO_CV].getVoltage() * 0.1f;
		bu_ratio = clamp(bu_ratio, 0.0f, 1.0f);

		float hl_ratio = params[PARAM_HIGH_UP_RATIO].getValue();
		hl_ratio += inputs[INPUT_HIGH_UP_RATIO_CV].getVoltage() * 0.1f;
		hl_ratio = clamp(hl_ratio, 0.0f, 1.0f);

		float hu_ratio = params[PARAM_HIGH_DOWN_RATIO].getValue();
		hu_ratio += inputs[INPUT_HIGH_DOWN_RATIO_CV].getVoltage() * 0.1f;
		hu_ratio = clamp(hu_ratio, 0.0f, 1.0f);

		// Set thresholds
		vals[vital::MultibandCompressor::kLowLowerThreshold - 1]->set(ll_thres);
		vals[vital::MultibandCompressor::kLowUpperThreshold - 1]->set(lu_thres);
		vals[vital::MultibandCompressor::kBandLowerThreshold - 1]->set(bl_thres);
		vals[vital::MultibandCompressor::kBandUpperThreshold - 1]->set(bu_thres);
		vals[vital::MultibandCompressor::kHighLowerThreshold - 1]->set(hl_thres);
		vals[vital::MultibandCompressor::kHighUpperThreshold - 1]->set(hu_thres);

		// Set ratios
		// Note: Lower = Upward compression (expansion), Upper = Downward compression
		vals[vital::MultibandCompressor::kLowLowerRatio - 1]->set(ll_ratio);
		vals[vital::MultibandCompressor::kLowUpperRatio - 1]->set(lu_ratio);
		vals[vital::MultibandCompressor::kBandLowerRatio - 1]->set(bl_ratio);
		vals[vital::MultibandCompressor::kBandUpperRatio - 1]->set(bu_ratio);
		vals[vital::MultibandCompressor::kHighLowerRatio - 1]->set(hl_ratio);
		vals[vital::MultibandCompressor::kHighUpperRatio - 1]->set(hu_ratio);

		// Set attack/release
		vals[vital::MultibandCompressor::kAttack - 1]->set(att_time);
		vals[vital::MultibandCompressor::kRelease - 1]->set(rel_time);

		// Set output gains (from controls)
		vals[vital::MultibandCompressor::kLowOutputGain - 1]->set(lgain);
		vals[vital::MultibandCompressor::kBandOutputGain - 1]->set(mgain);
		vals[vital::MultibandCompressor::kHighOutputGain - 1]->set(hgain);

		// Note: Crossover frequencies are hardcoded in Vital's MultibandCompressor
		// at 120Hz (low-mid) and 2500Hz (mid-high) and cannot be changed via parameters
		// The crossover1Freq and crossover2Freq variables are kept for future use
		// but are not currently applied to the Vital compressor
		(void)crossover1Freq;  // Suppress unused variable warning
		(void)crossover2Freq;  // Suppress unused variable warning

		// Set mix (dry/wet)
		vals[vital::MultibandCompressor::kMix - 1]->set(mix);
	}

	void onReset() override {
		compressor->reset(vital::constants::kFullMask);
	}

	void onSampleRateChange() override {
		if (!compressor) return;
		float sr = APP ? APP->engine->getSampleRate() : 44100.f;
		compressor->setSampleRate(sr);
	}

	void readAudio(vital::poly_float* comp_buf, float* leftBuf, float* rightBuf, int samples) {
		vital::mono_float* comp_output = (vital::mono_float*)comp_buf;
		float mag = vital::utils::dbToMagnitude(in_gain);

		for (int i = 0; i < samples; ++i) {
			comp_output[vital::poly_float::kSize * i + 0] = leftBuf[i] * mag;
			comp_output[vital::poly_float::kSize * i + 1] = rightBuf[i] * mag;
		}
	}

	void writeAudio(vital::poly_float* comp_buf, float* leftBuf, float* rightBuf, int samples) {
		const vital::mono_float* comp_output = (const vital::mono_float*)comp_buf;
		float mag = vital::utils::dbToMagnitude(out_gain);

		for (int i = 0; i < samples; ++i) {
			leftBuf[i] = comp_output[vital::poly_float::kSize * i + 0] * mag;
			rightBuf[i] = comp_output[vital::poly_float::kSize * i + 1] * mag;
		}
	}

	void process(const ProcessArgs& args) override {
		// Update parameters
		updateParams();

		// Get input
		float inL = inputs[INPUT_L].getVoltage();
		float inR = inputs[INPUT_R].isConnected() ? inputs[INPUT_R].getVoltage() : inL;

		// Convert from 5V to normalized float (assuming +/- 5V range)
		float leftBuf = inL / 5.0f;
		float rightBuf = inR / 5.0f;

		// Process values (smooth parameters)
		for (auto& val : vals) {
			val->process(1);
		}

		// Read audio into vital buffer
		readAudio(sig_in->buffer, &leftBuf, &rightBuf, 1);

		// Process compression
		compressor->process(1);

		// Write audio from vital buffer
		writeAudio(compressor->output(vital::MultibandCompressor::kAudioOut)->buffer,
		           &leftBuf, &rightBuf, 1);

		// Convert back to 5V range and output
		outputs[OUTPUT_L].setVoltage(leftBuf * 5.0f);
		outputs[OUTPUT_R].setVoltage(rightBuf * 5.0f);
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
			auto svg = APP->window->loadSvg(asset::plugin(pluginInstance, "res/SabnockOTT.svg"));
			if (svg) {
				svgWidget->setSvg(svg);
			} else {
				WARN("SVG returned null: res/SabnockOTT.svg");
			}
		} catch (const std::exception& e) {
			WARN("Exception loading SVG res/SabnockOTT.svg: %s", e.what());
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


struct SabnockOTTWidget : ModuleWidget {
	SabnockOTTWidget(SabnockOTT* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/SabnockOTT.svg")));

		auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size;
		addChild(bg);

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		// Top row: Attack, Release, Mix (3 medium knobs) - Andras style spacing
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(11.f, 22.f)), module, SabnockOTT::PARAM_ATTACK));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48f, 22.f)), module, SabnockOTT::PARAM_RELEASE));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(50.f, 22.f)), module, SabnockOTT::PARAM_MIX));

		// Second row: Band Gains (Low, Mid, High - 3 small knobs) - more space
		float gainY = 38.f;
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(11.f, gainY)), module, SabnockOTT::PARAM_LOW_GAIN));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(30.48f, gainY)), module, SabnockOTT::PARAM_MID_GAIN));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(50.f, gainY)), module, SabnockOTT::PARAM_HIGH_GAIN));

		// Third row: Input and Output gain (small knobs) - more space
		float ioGainY = 51.f;
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(20.74f, ioGainY)), module, SabnockOTT::PARAM_INPUT));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(40.74f, ioGainY)), module, SabnockOTT::PARAM_OUTPUT));

		// Band compression controls: Each column has TOP and BOTTOM knobs
		// TOP = Downward (compression), BOTTOM = Upward (expansion)
		// Layout: Small knob (threshold) with Trimpot (ratio) below

		float bandStartY = 63.f;  // More space from I/O gains
		float pairSpacing = 17.f;  // More vertical space between pairs
		float knobToTrimOffset = 5.5f;

		float lowX = 11.f;
		float midX = 30.48f;
		float highX = 50.f;

		// LOW BAND (Left column)
		float lowDownY = bandStartY;
		// Top: Downward (compression)
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(lowX, lowDownY)), module, SabnockOTT::PARAM_LOW_DOWN_THRESH));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(lowX, lowDownY + knobToTrimOffset)), module, SabnockOTT::PARAM_LOW_DOWN_RATIO));

		float lowUpY = lowDownY + pairSpacing;
		// Bottom: Upward (expansion)
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(lowX, lowUpY)), module, SabnockOTT::PARAM_LOW_UP_THRESH));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(lowX, lowUpY + knobToTrimOffset)), module, SabnockOTT::PARAM_LOW_UP_RATIO));

		// MID BAND (Middle column)
		float midDownY = bandStartY;
		// Top: Downward (compression)
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(midX, midDownY)), module, SabnockOTT::PARAM_MID_DOWN_THRESH));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(midX, midDownY + knobToTrimOffset)), module, SabnockOTT::PARAM_MID_DOWN_RATIO));

		float midUpY = midDownY + pairSpacing;
		// Bottom: Upward (expansion)
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(midX, midUpY)), module, SabnockOTT::PARAM_MID_UP_THRESH));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(midX, midUpY + knobToTrimOffset)), module, SabnockOTT::PARAM_MID_UP_RATIO));

		// HIGH BAND (Right column)
		float highDownY = bandStartY;
		// Top: Downward (compression)
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(highX, highDownY)), module, SabnockOTT::PARAM_HIGH_DOWN_THRESH));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(highX, highDownY + knobToTrimOffset)), module, SabnockOTT::PARAM_HIGH_DOWN_RATIO));

		float highUpY = highDownY + pairSpacing;
		// Bottom: Upward (expansion)
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(highX, highUpY)), module, SabnockOTT::PARAM_HIGH_UP_THRESH));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(highX, highUpY + knobToTrimOffset)), module, SabnockOTT::PARAM_HIGH_UP_RATIO));

		// === CV INPUTS - Fitted to 60.96mm width ===
		float cvStartY = 93.f;
		float cvRowSpacing = 8.f;
		float cvColSpacing = 7.f;  // Tighter spacing for narrower module
		float cvStartX = 4.5f;

		// Row 1: Attack, Release, Mix, In Gain, Out Gain, Low Gain, Mid Gain, High Gain
		float row1Y = cvStartY;
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX, row1Y)), module, SabnockOTT::INPUT_ATTACK_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + cvColSpacing, row1Y)), module, SabnockOTT::INPUT_RELEASE_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 2*cvColSpacing, row1Y)), module, SabnockOTT::INPUT_MIX_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 3*cvColSpacing, row1Y)), module, SabnockOTT::INPUT_INPUT_GAIN_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 4*cvColSpacing, row1Y)), module, SabnockOTT::INPUT_OUTPUT_GAIN_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 5*cvColSpacing, row1Y)), module, SabnockOTT::INPUT_LOW_GAIN_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 6*cvColSpacing, row1Y)), module, SabnockOTT::INPUT_MID_GAIN_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 7*cvColSpacing, row1Y)), module, SabnockOTT::INPUT_HIGH_GAIN_CV));

		// Row 2: Low band controls (4) + Mid band controls (4)
		float row2Y = row1Y + cvRowSpacing;
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX, row2Y)), module, SabnockOTT::INPUT_LOW_DOWN_THRESH_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + cvColSpacing, row2Y)), module, SabnockOTT::INPUT_LOW_DOWN_RATIO_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 2*cvColSpacing, row2Y)), module, SabnockOTT::INPUT_LOW_UP_THRESH_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 3*cvColSpacing, row2Y)), module, SabnockOTT::INPUT_LOW_UP_RATIO_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 4*cvColSpacing, row2Y)), module, SabnockOTT::INPUT_MID_DOWN_THRESH_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 5*cvColSpacing, row2Y)), module, SabnockOTT::INPUT_MID_DOWN_RATIO_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 6*cvColSpacing, row2Y)), module, SabnockOTT::INPUT_MID_UP_THRESH_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 7*cvColSpacing, row2Y)), module, SabnockOTT::INPUT_MID_UP_RATIO_CV));

		// Row 3: High band controls (4)
		float row3Y = row2Y + cvRowSpacing;
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX, row3Y)), module, SabnockOTT::INPUT_HIGH_DOWN_THRESH_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + cvColSpacing, row3Y)), module, SabnockOTT::INPUT_HIGH_DOWN_RATIO_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 2*cvColSpacing, row3Y)), module, SabnockOTT::INPUT_HIGH_UP_THRESH_CV));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + 3*cvColSpacing, row3Y)), module, SabnockOTT::INPUT_HIGH_UP_RATIO_CV));

		// === Audio I/O at bottom - INPUTS LEFT, OUTPUTS RIGHT (black) ===
		float audioIOY = row3Y + cvRowSpacing;
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX, audioIOY)), module, SabnockOTT::INPUT_L));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(cvStartX + cvColSpacing, audioIOY)), module, SabnockOTT::INPUT_R));
		// Outputs on the right side
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(cvStartX + 5*cvColSpacing, audioIOY)), module, SabnockOTT::OUTPUT_L));
		addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(cvStartX + 6*cvColSpacing, audioIOY)), module, SabnockOTT::OUTPUT_R));
	}
};

Model* modelSabnockOTT = createModel<SabnockOTT, SabnockOTTWidget>("SabnockOTT");
