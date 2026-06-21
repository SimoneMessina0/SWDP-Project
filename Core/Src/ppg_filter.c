#include "ppg_filter.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PPG_BUTTERWORTH_ORDER 12
#define PPG_FILTER_STAGES (PPG_BUTTERWORTH_ORDER) // 6 LPF + 6 HPF = 12 stadi

static float32_t ppg_red_filter_state[4 * PPG_FILTER_STAGES];
static float32_t ppg_ir_filter_state[4 * PPG_FILTER_STAGES];
static arm_biquad_casd_df1_inst_f32 ppg_red_filter_inst;
static arm_biquad_casd_df1_inst_f32 ppg_ir_filter_inst;
static float32_t ppg_filter_coeffs[5 * PPG_FILTER_STAGES];

static float32_t dc_baseline_red = -1.0f; // Usato solo come offset iniziale per ridurre il transitorio
static float32_t dc_baseline_ir = -1.0f;

void compute_butterworth_biquads(int order, double fc, double fs, bool is_highpass, float32_t *coeffs_out) {
	int num_biquads = order / 2;
	double wa = tan(M_PI * fc / fs);
	double wa2 = wa * wa;

	for (int k = 1; k <= num_biquads; k++) {
		double theta = M_PI * (2.0 * k - 1.0 + order) / (2.0 * order);
		double cos_theta = cos(theta);

		double a0 = 1.0 - 2.0 * wa * cos_theta + wa2;
		double a1_textbook = 2.0 * (wa2 - 1.0) / a0;
		double a2_textbook = (1.0 + 2.0 * wa * cos_theta + wa2) / a0;

		double b0, b1, b2;
		if (is_highpass) {
			b0 = 1.0 / a0;
			b1 = -2.0 / a0;
			b2 = 1.0 / a0;
		} else {
			b0 = wa2 / a0;
			b1 = 2.0 * wa2 / a0;
			b2 = wa2 / a0;
		}

		int idx = (k - 1) * 5;
		coeffs_out[idx + 0] = (float32_t)b0;
		coeffs_out[idx + 1] = (float32_t)b1;
		coeffs_out[idx + 2] = (float32_t)b2;
		coeffs_out[idx + 3] = (float32_t)(-a1_textbook);
		coeffs_out[idx + 4] = (float32_t)(-a2_textbook);
	}
}

/**
 * @brief Initializes the PPG filter.
 */
void PPG_Filter_Init(void) {
	compute_butterworth_biquads(PPG_BUTTERWORTH_ORDER, 10.0, 800.0, false, &ppg_filter_coeffs[0]);
	compute_butterworth_biquads(PPG_BUTTERWORTH_ORDER, 0.6, 800.0, true, &ppg_filter_coeffs[(PPG_BUTTERWORTH_ORDER / 2) * 5]);

	arm_biquad_cascade_df1_init_f32(&ppg_red_filter_inst, PPG_FILTER_STAGES, ppg_filter_coeffs, ppg_red_filter_state);
	arm_biquad_cascade_df1_init_f32(&ppg_ir_filter_inst, PPG_FILTER_STAGES, ppg_filter_coeffs, ppg_ir_filter_state);
	dc_baseline_red = -1.0f;
	dc_baseline_ir = -1.0f;
}

/**
 * @brief Filters a single PPG sample.
 * @param sample The raw input sample.
 * @return The filtered output sample.
 */
void PPG_Filter_ProcessSamples(float32_t raw_red, float32_t raw_ir, float32_t *filtered_red, float32_t *filtered_ir) {
	// Rilevatore dito: se il segnale grezzo IR è sotto 1000, il dito non è appoggiato
	if (raw_ir < 1000.0f) {
		dc_baseline_red = -1.0f;
		dc_baseline_ir = -1.0f;
		// Resetta lo stato interno del filtro Biquad per evitare ringing o spike
		for (int i = 0; i < 4 * PPG_FILTER_STAGES; i++) {
			ppg_red_filter_state[i] = 0.0f;
			ppg_ir_filter_state[i] = 0.0f;
		}
		*filtered_red = 0.0f;
		*filtered_ir = 0.0f;
		return;
	}

	// Inizializza la baseline al primo campione valido per evitare un lungo transitorio
	if (dc_baseline_ir == -1.0f) {
		dc_baseline_red = raw_red;
		dc_baseline_ir = raw_ir;
	}

	// Rimuoviamo l'offset DC grezzo per minimizzare il gradino iniziale in ingresso al filtro
	float32_t ac_red = raw_red - dc_baseline_red;
	float32_t ac_ir = raw_ir - dc_baseline_ir;
	
	float32_t out_red = 0.0f;
	float32_t out_ir = 0.0f;

	// Applica il filtro passa-banda 12esimo ordine (0.6 - 10 Hz)
	arm_biquad_cascade_df1_f32(&ppg_red_filter_inst, &ac_red, &out_red, 1);
	arm_biquad_cascade_df1_f32(&ppg_ir_filter_inst, &ac_ir, &out_ir, 1);

	// Siccome il filtro 12esimo ordine è molto reattivo al rumore da movimento,
	// potremmo avere spike. Limitarli (o riavviare il filtro) aiuta.
	if (out_ir > 100.0f || out_ir < -100.0f || out_red > 100.0f || out_red < -100.0f) {
		dc_baseline_red = raw_red;
		dc_baseline_ir = raw_ir;
		for (int i = 0; i < 4 * PPG_FILTER_STAGES; i++) {
			ppg_red_filter_state[i] = 0.0f;
			ppg_ir_filter_state[i] = 0.0f;
		}
		out_red = 0.0f;
		out_ir = 0.0f;
	}

	*filtered_red = out_red;
	*filtered_ir = out_ir;
}
