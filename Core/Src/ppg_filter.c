#include "ppg_filter.h"

#define PPG_FILTER_STAGES 1

static float32_t ppg_filter_state[4 * PPG_FILTER_STAGES];
static arm_biquad_casd_df1_inst_f32 ppg_filter_inst;

// Stage 1: 2nd-order Butterworth LPF (fc = 15.0 Hz, fs = 800.0 Hz)
static float32_t ppg_filter_coeffs[5 * PPG_FILTER_STAGES] = {
    0.00319981f, 0.00639962f, 0.00319981f, 1.8337339f, -0.8465330f
};

static float32_t dc_baseline1 = -1.0f;
static float32_t dc_baseline2 = 0.0f;

/**
 * @brief Initializes the PPG filter.
 */
void PPG_Filter_Init(void) {
    arm_biquad_cascade_df1_init_f32(&ppg_filter_inst, PPG_FILTER_STAGES, ppg_filter_coeffs, ppg_filter_state);
    dc_baseline1 = -1.0f;
    dc_baseline2 = 0.0f;
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
        dc_baseline2 = 0.0f;
        // Resetta lo stato interno del filtro Biquad per evitare ringing o spike
        for (int i = 0; i < 4 * PPG_FILTER_STAGES; i++) {
            ppg_filter_state[i] = 0.0f;
        }
        return 0.0f; // Output piatto se non c'è il dito
    }

    // Inizializza la baseline al primo campione valido per evitare il lungo transitorio
    if (dc_baseline1 == -1.0f) {
        dc_baseline1 = sample;
        dc_baseline2 = 0.0f;
    }
    
    // Filtro passa-alto "potenziato" (2° ordine): applichiamo due stadi in cascata
    // Primo stadio
    dc_baseline1 = 0.9921f * dc_baseline1 + 0.0079f * sample;
    float32_t ac1 = sample - dc_baseline1;
    
    // Secondo stadio
    dc_baseline2 = 0.9921f * dc_baseline2 + 0.0079f * ac1;
    float32_t ac_component = ac1 - dc_baseline2;

    float32_t output = 0.0f;
    // Applica il filtro passa-basso a 15 Hz sull'onda AC per rimuovere il rumore ad alta frequenza
    arm_biquad_cascade_df1_f32(&ppg_filter_inst, &ac_component, &output, 1);
    
    // Rilevatore di spike: ATTENZIONE, se la soglia è troppo bassa (es. 100), 
    // scatta ad ogni normale battito cardiaco! Meglio tenerla ad almeno 400.
    if (output > 400.0f || output < -400.0f) {
        dc_baseline1 = sample; // Re-inizializza la baseline al valore corrente
        dc_baseline2 = 0.0f;
        // Resetta lo stato interno del filtro Biquad per evitare ringing o spike
        for (int i = 0; i < 4 * PPG_FILTER_STAGES; i++) {
            ppg_filter_state[i] = 0.0f;
        }
        output = 0.0f; // Azzera l'output per questo campione
    }
    
    return output;
}
