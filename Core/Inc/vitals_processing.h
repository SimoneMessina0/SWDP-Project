/**
 * \file vitals_processing.h
 * \brief PPG algorithms for calculating HR, SpO2, HRV, RR, and estimated VO2Max.
 *
 * This module works downstream of the MAX30101 driver and the PPG filter
 * (ppg_filter.c) already present in the project. All calculations are done
 * in pure C, sample-by-sample, designed to run inside the
 * TIM2 IRQ callback (100 Hz) without dynamic allocations.
 *
 * Planned pipeline (called at each new sample, i.e. at each
 * TIM2 tick at 800 Hz):
 *
 *   raw_IR, raw_RED  --(gia' letti da MAX30101_Read_Data)-->
 *     Vitals_ProcessSample(raw_red, raw_ir)
 *        |--> aggiorna filtro AC/DC per RED e IR
 *        |--> peak detection on filtered IR signal -> HR + HRV (NN
 *             intervals)
 *        |--> calculation R = (AC_red/DC_red)/(AC_ir/DC_ir) -> SpO2
 *        |--> band-pass filter 0.1-0.4 Hz on DC IR -> RR
 *
 * The *_GetLatest() functions return the last stable calculated value
 * (NaN/default value if a valid result is not yet available).
 *
 * chosen for a PPG acquisition fs of 200 Hz, consistent with TIM2.
 */

#ifndef VITALS_PROCESSING_H
#define VITALS_PROCESSING_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 *  CONFIGURAZIONE GENERALE
 * ======================================================================== */

#define VITALS_FS_HZ                800.0f   /* PPG sampling frequency (Hz), must match TIM2 */

/* --- HR / peak detection --- */
#define VITALS_HR_MIN_BPM           35.0f
#define VITALS_HR_MAX_BPM           300.0f /* 300 bpm => refractory period 0.2s */
/* Minimum accepted distance between two peaks, derived from HR_MAX_BPM */
#define VITALS_MIN_PEAK_DISTANCE_S  (60.0f / VITALS_HR_MAX_BPM)

/* --- HRV --- */
#define VITALS_HRV_NN_BUFFER_LEN    30U   /* number of NN intervals (RR-to-RR) stored for SDNN/RMSSD */

/* --- SpO2 --- */
#define VITALS_SPO2_WINDOW_SAMPLES  800U  /* window (samples) on which to calculate AC/DC for each SpO2 estimate, ~1s @800Hz */

/* --- RR (respiration) --- */
#define VITALS_RESP_BUFFER_LEN      512U  /* decimated samples buffer for breath estimation (~ few minutes at decimated fs) */

/* ========================================================================
 *  STRUTTURE DATI
 * ======================================================================== */

/** Aggregated result, convenient to send via BLE/USB or save to NAND. */
typedef struct {
    float    hr_bpm;          /* Beats per minute, -1 if not available */
    float    spo2_percent;    /* SpO2 percentage, -1 if not available */
    float    hrv_sdnn_ms;     /* SDNN in milliseconds, -1 if not available */
    float    hrv_rmssd_ms;    /* RMSSD in milliseconds, -1 if not available */
    float    rr_brpm;         /* Breaths per minute, -1 if not available */

    bool     hr_valid;
    bool     spo2_valid;
    bool     hrv_valid;
    bool     rr_valid;

} Vitals_Results;

/* ========================================================================
 *  API PUBBLICA
 * ======================================================================== */

/**
 * @brief Initializes all internal states of the module (filters, buffers,
 * indices). To be called once at startup, after PPG_Filter_Init().
 */
void Vitals_Init(void);

/**
 * @brief To be called at each new available sample (200 Hz), with the
 * RAW values (unfiltered) read from the MAX30101 FIFO.
 *
 * Executes, in order:
 *  - update DC estimate (moving average) and AC (band-pass filtered signal
 *    cardiac, 0.5-5 Hz) for RED and IR
 *  - peak detection on IR signal for HR/HRV
 *  - window update for SpO2 (R ratio)
 *  - respiration filter feed (0.1-0.4 Hz) on the DC component
 *    of the IR signal
 *
 * @param raw_red  Raw RED sample (18 useful bits, already masked by driver)
 * @param raw_ir   Raw IR sample (18 useful bits, already masked by driver)
 */
void Vitals_ProcessSample(uint32_t raw_red, uint32_t raw_ir, float ac_red_filtered, float ac_ir_filtered);

/**
 * @brief Returns the last calculated HR value (moving average over
 * the last valid peak-to-peak intervals).
 * @param hr_bpm Output pointer.
 * @return true if the value is valid (at least 2 peaks detected and in range).
 */
bool Vitals_GetHR(float *hr_bpm);

/**
 * @brief Returns the last SpO2 estimate.
 * @param spo2_percent Output pointer (0-100).
 * @return true if the value is valid.
 */
bool Vitals_GetSpO2(float *spo2_percent);

/**
 * @brief Returns the current time-domain HRV metrics (SDNN, RMSSD),
 * calculated on the last VITALS_HRV_NN_BUFFER_LEN available NN intervals.
 * @param sdnn_ms  Output pointer for SDNN in ms.
 * @param rmssd_ms Output pointer for RMSSD in ms.
 * @return true if there are at least 2 NN intervals available.
 */
bool Vitals_GetHRV(float *sdnn_ms, float *rmssd_ms);

/**
 * @brief Returns the last respiratory rate estimate.
 * @param rr_brpm Output pointer (breaths/min).
 * @return true if the value is valid (respiration buffer full and spectral peak/
 * zero-crossing found in band).
 */
bool Vitals_GetRR(float *rr_brpm);



/**
 * @brief Convenience helper: fills a Vitals_Results struct with all the
 * latest available values at once (useful before building a
 * BLE packet or a field to save in NAND).
 */
void Vitals_GetAllResults(Vitals_Results *out);

#ifdef __cplusplus
}
#endif

#endif /* VITALS_PROCESSING_H */