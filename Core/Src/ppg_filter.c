#include "ppg_filter.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PPG_BUTTERWORTH_ORDER 12
#define PPG_FILTER_STAGES (PPG_BUTTERWORTH_ORDER) // 6 LPF + 6 HPF = 12 stadi

static float32_t ppg_filter_state[4 * PPG_FILTER_STAGES];
static arm_biquad_casd_df1_inst_f32 ppg_filter_inst;
static float32_t ppg_filter_coeffs[5 * PPG_FILTER_STAGES];

static float32_t dc_baseline1 = -1.0f; // Usato solo come offset iniziale per ridurre il transitorio

static void compute_butterworth_biquads(int order, double fc, double fs, bool is_highpass, float32_t* coeffs_out) {
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

    arm_biquad_cascade_df1_init_f32(&ppg_filter_inst, PPG_FILTER_STAGES, ppg_filter_coeffs, ppg_filter_state);
    dc_baseline1 = -1.0f;
}

/**
 * @brief Filters a single PPG sample.
 * @param sample The raw input sample.
 * @return The filtered output sample.
 */
float32_t PPG_Filter_ProcessSample(float32_t sample) {
    // Rilevatore dito: se il segnale grezzo è sotto 1000, il dito non è appoggiato
    if (sample < 1000.0f) {
        dc_baseline1 = -1.0f; // Resetta la baseline per il prossimo appoggio
        // Resetta lo stato interno del filtro Biquad per evitare ringing o spike
        for (int i = 0; i < 4 * PPG_FILTER_STAGES; i++) {
            ppg_filter_state[i] = 0.0f;
        }
        return 0.0f; // Output piatto se non c'è il dito
    }

    // Inizializza la baseline al primo campione valido per evitare un lungo transitorio
    if (dc_baseline1 == -1.0f) {
        dc_baseline1 = sample;
    }
    
    // Rimuoviamo l'offset DC grezzo per minimizzare il gradino iniziale in ingresso al filtro
    float32_t ac_component = sample - dc_baseline1;
    float32_t output = 0.0f;
    
    // Applica il filtro passa-banda 12esimo ordine (0.6 - 10 Hz)
    arm_biquad_cascade_df1_f32(&ppg_filter_inst, &ac_component, &output, 1);
    
    // Siccome il filtro 12esimo ordine è molto reattivo al rumore da movimento, 
    // potremmo avere spike. Limitarli (o riavviare il filtro) aiuta.
    if (output > 2000.0f || output < -2000.0f) {
        dc_baseline1 = sample;
        for (int i = 0; i < 4 * PPG_FILTER_STAGES; i++) {
            ppg_filter_state[i] = 0.0f;
        }
        output = 0.0f;
    }
    
    return output;
}
