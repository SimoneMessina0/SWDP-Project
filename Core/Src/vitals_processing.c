/**
 * \file vitals_processing.c
 * \brief Implementation of PPG algorithms: HR, SpO2, HRV, RR, VO2Max.
 *
 * General Implementation Notes:
 *  - Implementation based on sliding window and IIR filters.
 *  - AC signal obtained with separate biquad IIR band-pass filter for IR and RED.
 *  - DC signal (baseline) calculated using Exponential Moving Average (EMA).
 */

#include "vitals_processing.h"
#include "arm_math.h"
#include "ppg_filter.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 *  RESPIRATORY FILTER (band-pass 0.1 - 0.4 Hz, on the IR DC baseline)
 *  2 stage biquad Butterworth, effective fs = VITALS_FS_HZ (100 Hz). The
 *  0.1-0.4 Hz band corresponds to 6-24 breaths/minute, physiological range
 *  at rest / light effort.
 * ======================================================================== */

#define VITALS_RESP_FILTER_STAGES 2

static float32_t resp_filter_state[4 * VITALS_RESP_FILTER_STAGES];
static arm_biquad_casd_df1_inst_f32 resp_filter_inst;

/* Butterworth 2 stage coefficients, fc1=0.4Hz (LPF) + fc2=0.1Hz (HPF), will be calculated dynamically for fs=800Hz. */
static float32_t resp_filter_coeffs[5 * VITALS_RESP_FILTER_STAGES];

/* ========================================================================
 *  INTERNAL STATE: DC TRACKER (exponential moving average)
 * ======================================================================== */

/* EMA time constant for DC baseline: tau ~ 1.25s
 * alpha = 1 - exp(-1/(fs*tau)) approx; scaled for 800Hz */
#define VITALS_DC_ALPHA 0.001f

static float dc_red = 0.0f;
static float dc_ir = 0.0f;
static bool dc_initialized = false;

/* ========================================================================
 *  INTERNAL STATE: PEAK DETECTION (for HR / HRV) on filtered IR signal
 * ======================================================================== */

/* Adaptive threshold for local maxima (80% of recent max) */
#define VITALS_PEAK_THRESHOLD_RATIO 0.8f
#define VITALS_MAX_DECAY_RATE (1.0f / (3.0f * VITALS_FS_HZ)) /* decay in ~3 sec */

static float ac_ir_recent_max = 0.0f; /* recent maximum of IR AC signal */
static float prev_sample_1 = 0.0f;	  /* previous sample (for derivative/descent) */
static float prev_sample_2 = 0.0f;	  /* sample before previous */
static bool rising_edge_seen = false;
static uint32_t sample_counter = 0; /* total sample counter, for peak timestamps */
static uint32_t last_peak_sample_idx = 0;
static bool last_peak_valid = false;

/* Circular buffer of the latest NN intervals (peak-to-peak), in milliseconds */
static float nn_intervals_ms[VITALS_HRV_NN_BUFFER_LEN];
static uint16_t nn_count = 0;	  /* number of valid intervals in buffer (saturates at VITALS_HRV_NN_BUFFER_LEN) */
static uint16_t nn_write_idx = 0; /* circular write index */

/* Current HR estimation, calculated as moving average over the latest NN intervals */
static float current_hr_bpm = -1.0f;
static bool current_hr_valid = false;

/* ========================================================================
 *  INTERNAL STATE: SpO2
 * ======================================================================== */

/* To estimate AC (peak-to-peak amplitude) over a window, we keep min/max
 * of the filtered cardiac signal for RED and IR on the current window. */
static float red_ac_min, red_ac_max;
static float ir_ac_min, ir_ac_max;
static uint32_t spo2_window_counter = 0;

static float current_spo2 = -1.0f;
static bool current_spo2_valid = false;

/* ========================================================================
 *  INTERNAL STATE: RESPIRATION (RR)
 * ======================================================================== */

/* Filtered respiration signal buffer (decimated to reduce memory
 * cost: 1 sample every 10 -> 10 Hz effective, 0.1-0.4Hz band
 * is well below Nyquist even at 10Hz). */
static float resp_buffer[VITALS_RESP_BUFFER_LEN];
static uint16_t resp_write_idx = 0;
static uint16_t resp_count = 0; /* how many valid samples are in buffer (saturates) */
static uint8_t resp_decim_counter = 0;
#define VITALS_RESP_DECIMATION 80U /* 800Hz / 80 = 10 Hz */
#define VITALS_RESP_FS_HZ (VITALS_FS_HZ / (float)VITALS_RESP_DECIMATION)

static float current_rr_brpm = -1.0f;
static bool current_rr_valid = false;

/* ========================================================================
 *  PRIVATE FUNCTIONS
 * ======================================================================== */

/**
 * @brief Updates the circular buffer of NN intervals with a new
 * peak-to-peak interval (in ms) and recalculates the current HR estimate as
 * average of the latest available intervals.
 */
static void vitals_push_nn_interval(float interval_ms) {
	/* Discard physiologically impossible intervals (noise / false peaks) */
	float bpm_instant = 60000.0f / interval_ms;
	if (bpm_instant < VITALS_HR_MIN_BPM || bpm_instant > VITALS_HR_MAX_BPM) {
		return;
	}

	nn_intervals_ms[nn_write_idx] = interval_ms;
	nn_write_idx = (uint16_t)((nn_write_idx + 1U) % VITALS_HRV_NN_BUFFER_LEN);
	if (nn_count < VITALS_HRV_NN_BUFFER_LEN) {
		nn_count++;
	}

	/* Recalculate HR as average of available NN intervals (more stable
	 * than just the last interval) */
	float sum_ms = 0.0f;
	for (uint16_t i = 0; i < nn_count; i++) {
		sum_ms += nn_intervals_ms[i];
	}
	float mean_interval_ms = sum_ms / (float)nn_count;
	current_hr_bpm = 60000.0f / mean_interval_ms;
	current_hr_valid = true;
}

/**
 * @brief Peak detection on filtered IR signal (cardiac band).
 * Uses an adaptive threshold based on the estimated amplitude envelope and a
 * refractoriness check (minimum distance between peaks) to avoid
 * double counting on high frequency noise.
 *
 * @param ac_ir_filtered Current sample of the filtered IR signal (AC).
 */
static void vitals_detect_peak(float ac_ir_filtered) {
	/* Update the decaying local maximum to calculate the 70% threshold */
	ac_ir_recent_max -= ac_ir_recent_max * VITALS_MAX_DECAY_RATE;
	if (ac_ir_filtered > ac_ir_recent_max) {
		ac_ir_recent_max = ac_ir_filtered;
	}

	float threshold = VITALS_PEAK_THRESHOLD_RATIO * ac_ir_recent_max;

	/* Beat detection via local maximum validated by threshold:
	 * the previous sample is greater than both the sample before it
	 * and the current one, and is above the threshold. */
	bool is_beat = (prev_sample_1 > prev_sample_2) &&
				   (prev_sample_1 > ac_ir_filtered) &&
				   (prev_sample_1 >= threshold);

	if (is_beat) {
		uint32_t min_distance_samples =
			(uint32_t)(VITALS_MIN_PEAK_DISTANCE_S * VITALS_FS_HZ);

		/* sample_counter points to the current sample; the detected peak is
		 * at the previous sample (sample_counter - 1) */
		uint32_t peak_idx = sample_counter - 1U;

		if (!last_peak_valid ||
			(peak_idx - last_peak_sample_idx) >= min_distance_samples) {

			if (last_peak_valid) {
				float interval_samples = (float)(peak_idx - last_peak_sample_idx);
				float interval_ms = (interval_samples / VITALS_FS_HZ) * 1000.0f;
				vitals_push_nn_interval(interval_ms);
			}
			last_peak_sample_idx = peak_idx;
			last_peak_valid = true;
		}
	}

	prev_sample_2 = prev_sample_1;
	prev_sample_1 = ac_ir_filtered;
}

/**
 * @brief Updates the min/max statistics of the current SpO2 window for
 * RED and IR; at the end of the window it calculates R and converts it to SpO2,
 * then resets the window.
 */
static void vitals_update_spo2_window(float ac_red_filtered, float ac_ir_filtered,
									  float dc_red_val, float dc_ir_val) {
	if (spo2_window_counter == 0) {
		red_ac_min = red_ac_max = ac_red_filtered;
		ir_ac_min = ir_ac_max = ac_ir_filtered;
	} else {
		if (ac_red_filtered < red_ac_min) red_ac_min = ac_red_filtered;
		if (ac_red_filtered > red_ac_max) red_ac_max = ac_red_filtered;
		if (ac_ir_filtered < ir_ac_min) ir_ac_min = ac_ir_filtered;
		if (ac_ir_filtered > ir_ac_max) ir_ac_max = ac_ir_filtered;
	}

	spo2_window_counter++;

	if (spo2_window_counter >= VITALS_SPO2_WINDOW_SAMPLES) {
		float ac_red_pp = red_ac_max - red_ac_min; /* peak-to-peak amplitude RED */
		float ac_ir_pp = ir_ac_max - ir_ac_min;	   /* peak-to-peak amplitude IR */

		/* Avoid division by zero / garbage data (finger not placed) */
		if (dc_red_val > 1.0f && dc_ir_val > 1.0f && ac_ir_pp > 1e-6f) {
			float ratio_red = ac_red_pp / dc_red_val;
			float ratio_ir = ac_ir_pp / dc_ir_val;

			if (ratio_ir > 1e-9f) {
				float R = ratio_red / ratio_ir;

				/* Empirical equation for SpO2 calculation */
				float spo2_est = 110.0f - 25.0f * R;
				if (spo2_est > 100.0f) spo2_est = 100.0f;
				if (spo2_est < 70.0f) spo2_est = 70.0f;

				current_spo2 = spo2_est;
				current_spo2_valid = true;
			}
		}

		spo2_window_counter = 0;
	}
}

/**
 * @brief Feeds the respiration buffer with a new sample (already
 * filtered in 0.1-0.4Hz band and decimated), and if the buffer is full updates
 * the RR estimation by counting zero-crossings (from negative to positive) on
 * the available window.
 */
static void vitals_update_respiration(float resp_sample_filtered) {
	resp_decim_counter++;
	if (resp_decim_counter < VITALS_RESP_DECIMATION) {
		return;
	}
	resp_decim_counter = 0;

	resp_buffer[resp_write_idx] = resp_sample_filtered;
	resp_write_idx = (uint16_t)((resp_write_idx + 1U) % VITALS_RESP_BUFFER_LEN);
	if (resp_count < VITALS_RESP_BUFFER_LEN) {
		resp_count++;
	}

	/* Wait for a sufficient window (at least 30s) before estimating RR. */
	uint16_t min_samples_for_rr = (uint16_t)(30.0f * VITALS_RESP_FS_HZ);
	if (resp_count < min_samples_for_rr) {
		return;
	}

	/* Detects local maxima peaks over the entire available buffer,
	 * in chronological order. The buffer is circular: the chronological
	 * order starts from resp_write_idx if the buffer is full, otherwise from 0. */
	uint16_t start_idx = (resp_count < VITALS_RESP_BUFFER_LEN) ? 0 : resp_write_idx;
	uint16_t n = resp_count;

	/* Find the global maximum in the buffer to calculate the amplitude threshold */
	float max_val = 0.0f;
	for (uint16_t i = 0; i < n; i++) {
		uint16_t idx = (uint16_t)((start_idx + i) % VITALS_RESP_BUFFER_LEN);
		if (resp_buffer[idx] > max_val) {
			max_val = resp_buffer[idx];
		}
	}

	float threshold = 0.50f * max_val;
	uint16_t peaks = 0;
	int32_t last_peak_idx = -1;
	uint32_t min_distance_samples = (uint32_t)(0.2f * VITALS_RESP_FS_HZ);

	for (uint16_t i = 1; i < n - 1; i++) {
		uint16_t idx_prev = (uint16_t)((start_idx + i - 1) % VITALS_RESP_BUFFER_LEN);
		uint16_t idx_curr = (uint16_t)((start_idx + i) % VITALS_RESP_BUFFER_LEN);
		uint16_t idx_next = (uint16_t)((start_idx + i + 1) % VITALS_RESP_BUFFER_LEN);

		float val_prev = resp_buffer[idx_prev];
		float val_curr = resp_buffer[idx_curr];
		float val_next = resp_buffer[idx_next];

		if (val_curr > val_prev && val_curr > val_next && val_curr >= threshold) {
			if (last_peak_idx == -1 || (i - last_peak_idx) >= min_distance_samples) {
				peaks++;
				last_peak_idx = i;
			}
		}
	}

	float duration_s = (float)(n - 1) / VITALS_RESP_FS_HZ;
	if (duration_s > 0.0f && peaks > 0) {
		float breaths_per_second = (float)peaks / duration_s;
		float brpm = breaths_per_second * 60.0f;

		/* Plausible physiological range: 4-40 breaths/min */
		if (brpm >= 4.0f && brpm <= 40.0f) {
			current_rr_brpm = brpm;
			current_rr_valid = true;
		}
	}
}

/* ========================================================================
 *  PUBLIC API
 * ======================================================================== */

void Vitals_Init(void) {
	compute_butterworth_biquads(2, 0.4, 800.0, false, &resp_filter_coeffs[0]);
	compute_butterworth_biquads(2, 0.1, 800.0, true, &resp_filter_coeffs[5]);

	arm_biquad_cascade_df1_init_f32(&resp_filter_inst, VITALS_RESP_FILTER_STAGES,
									resp_filter_coeffs, resp_filter_state);

	dc_red = 0.0f;
	dc_ir = 0.0f;
	dc_initialized = false;

	ac_ir_recent_max = 0.0f;
	prev_sample_1 = 0.0f;
	prev_sample_2 = 0.0f;
	rising_edge_seen = false;
	sample_counter = 0;
	last_peak_sample_idx = 0;
	last_peak_valid = false;

	memset(nn_intervals_ms, 0, sizeof(nn_intervals_ms));
	nn_count = 0;
	nn_write_idx = 0;
	current_hr_bpm = -1.0f;
	current_hr_valid = false;

	red_ac_min = red_ac_max = 0.0f;
	ir_ac_min = ir_ac_max = 0.0f;
	spo2_window_counter = 0;
	current_spo2 = -1.0f;
	current_spo2_valid = false;

	memset(resp_buffer, 0, sizeof(resp_buffer));
	resp_write_idx = 0;
	resp_count = 0;
	resp_decim_counter = 0;
	current_rr_brpm = -1.0f;
	current_rr_valid = false;
}

void Vitals_ProcessSample(uint32_t raw_red, uint32_t raw_ir, float ac_red_filtered, float ac_ir_filtered) {
	float red_f = (float)raw_red;
	float ir_f = (float)raw_ir;

	/* --- DC baseline update (EMA) --- */
	if (!dc_initialized) {
		dc_red = red_f;
		dc_ir = ir_f;
		dc_initialized = true;
	} else {
		dc_red = (1.0f - VITALS_DC_ALPHA) * dc_red + VITALS_DC_ALPHA * red_f;
		dc_ir = (1.0f - VITALS_DC_ALPHA) * dc_ir + VITALS_DC_ALPHA * ir_f;
	}

	/* Invert AC components since reflective PPG yields inverted morphology */
	ac_red_filtered = -ac_red_filtered;
	ac_ir_filtered = -ac_ir_filtered;

	sample_counter++;

	/* --- HR / HRV: peak detection on the filtered IR channel --- */
	vitals_detect_peak(ac_ir_filtered);

	/* --- SpO2: R-ratio window update --- */
	vitals_update_spo2_window(ac_red_filtered, ac_ir_filtered, dc_red, dc_ir);

	/* --- RR: 0.1-0.4Hz band-pass filter on the IR (DC) baseline --- */
	float resp_sample_filtered = 0.0f;
	float dc_ir_in = dc_ir;
	arm_biquad_cascade_df1_f32(&resp_filter_inst, &dc_ir_in, &resp_sample_filtered, 1);
	vitals_update_respiration(resp_sample_filtered);
}

bool Vitals_GetHR(float *hr_bpm) {
	if (hr_bpm == NULL) return false;
	*hr_bpm = current_hr_valid ? current_hr_bpm : -1.0f;
	return current_hr_valid;
}

bool Vitals_GetSpO2(float *spo2_percent) {
	if (spo2_percent == NULL) return false;
	*spo2_percent = current_spo2_valid ? current_spo2 : -1.0f;
	return current_spo2_valid;
}

bool Vitals_GetHRV(float *sdnn_ms, float *rmssd_ms) {
	if (sdnn_ms == NULL || rmssd_ms == NULL) return false;

	if (nn_count < 2) {
		*sdnn_ms = -1.0f;
		*rmssd_ms = -1.0f;
		return false;
	}

	/* SDNN: standard deviation of the NN intervals */
	float sum = 0.0f;
	for (uint16_t i = 0; i < nn_count; i++) {
		sum += nn_intervals_ms[i];
	}
	float mean = sum / (float)nn_count;

	float sq_sum = 0.0f;
	for (uint16_t i = 0; i < nn_count; i++) {
		float diff = nn_intervals_ms[i] - mean;
		sq_sum += diff * diff;
	}
	float sdnn = sqrtf(sq_sum / (float)nn_count);

	/* RMSSD: root mean square of successive differences between NN intervals.
	 * Must be calculated on the chronological sequence: we reconstruct
	 * the order starting from the circular write index. */
	uint16_t start_idx = (nn_count < VITALS_HRV_NN_BUFFER_LEN) ? 0 : nn_write_idx;
	float sq_diff_sum = 0.0f;
	uint16_t diff_count = 0;
	float prev_nn = nn_intervals_ms[start_idx];
	for (uint16_t i = 1; i < nn_count; i++) {
		uint16_t idx = (uint16_t)((start_idx + i) % VITALS_HRV_NN_BUFFER_LEN);
		float cur_nn = nn_intervals_ms[idx];
		float d = cur_nn - prev_nn;
		sq_diff_sum += d * d;
		diff_count++;
		prev_nn = cur_nn;
	}

	float rmssd = (diff_count > 0) ? sqrtf(sq_diff_sum / (float)diff_count) : -1.0f;

	*sdnn_ms = sdnn;
	*rmssd_ms = rmssd;
	return true;
}

bool Vitals_GetRR(float *rr_brpm) {
	if (rr_brpm == NULL) return false;
	*rr_brpm = current_rr_valid ? current_rr_brpm : -1.0f;
	return current_rr_valid;
}

void Vitals_GetAllResults(Vitals_Results *out) {
	if (out == NULL) return;

	out->hr_valid = Vitals_GetHR(&out->hr_bpm);
	out->spo2_valid = Vitals_GetSpO2(&out->spo2_percent);
	out->hrv_valid = Vitals_GetHRV(&out->hrv_sdnn_ms, &out->hrv_rmssd_ms);
	out->rr_valid = Vitals_GetRR(&out->rr_brpm);
}