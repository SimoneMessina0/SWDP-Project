/**
 * \file vitals_processing.h
 * \brief Algoritmi PPG per il calcolo di HR, SpO2, HRV, RR e VO2Max stimato.
 *
 * Questo modulo lavora a valle del driver MAX30101 e del filtro PPG
 * (ppg_filter.c) gia' presenti nel progetto. Tutto il calcolo e' fatto
 * in C puro, campione-per-campione, pensato per girare dentro alla
 * TIM2 IRQ callback (100 Hz) senza allocazioni dinamiche.
 *
 * Pipeline prevista (chiamata ad ogni nuovo campione, cioe' ad ogni
 * tick del TIM2 a 800 Hz):
 *
 *   raw_IR, raw_RED  --(gia' letti da MAX30101_Read_Data)-->
 *     Vitals_ProcessSample(raw_red, raw_ir)
 *        |--> aggiorna filtro AC/DC per RED e IR
 *        |--> rilevazione picchi sul segnale IR filtrato -> HR + HRV (NN
 *             intervals)
 *        |--> calcolo R = (AC_red/DC_red)/(AC_ir/DC_ir) -> SpO2
 *        |--> filtro passa-banda 0.1-0.4 Hz sul DC IR -> RR
 *
 * Le funzioni *_GetLatest() restituiscono l'ultimo valore stabile calcolato
 * (NaN/valore di default se non ancora disponibile un risultato valido).
 *
 * scelte per un fs di acquisizione PPG di 200 Hz, coerente con TIM2.
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

#define VITALS_FS_HZ                800.0f   /* Frequenza di campionamento PPG (Hz), deve combaciare con TIM2 */

/* --- HR / peak detection --- */
#define VITALS_HR_MIN_BPM           35.0f
#define VITALS_HR_MAX_BPM           300.0f /* 300 bpm => periodo refrattario 0.2s */
/* Distanza minima tra due picchi accettata, derivata da HR_MAX_BPM */
#define VITALS_MIN_PEAK_DISTANCE_S  (60.0f / VITALS_HR_MAX_BPM)

/* --- HRV --- */
#define VITALS_HRV_NN_BUFFER_LEN    30U   /* numero di intervalli NN (RR-to-RR) memorizzati per SDNN/RMSSD */

/* --- SpO2 --- */
#define VITALS_SPO2_WINDOW_SAMPLES  800U  /* finestra (campioni) su cui calcolare AC/DC per ogni stima SpO2, ~1s @800Hz */

/* --- RR (respiration) --- */
#define VITALS_RESP_BUFFER_LEN      512U  /* buffer campioni decimati per stima respiro (~ alcuni minuti a fs decimata) */

/* ========================================================================
 *  STRUTTURE DATI
 * ======================================================================== */

/** Risultato aggregato, comodo da inviare via BLE/USB o salvare in NAND. */
typedef struct {
    float    hr_bpm;          /* Battiti per minuto, -1 se non disponibile */
    float    spo2_percent;    /* Percentuale SpO2, -1 se non disponibile */
    float    hrv_sdnn_ms;     /* SDNN in millisecondi, -1 se non disponibile */
    float    hrv_rmssd_ms;    /* RMSSD in millisecondi, -1 se non disponibile */
    float    rr_brpm;         /* Respiri per minuto, -1 se non disponibile */

    bool     hr_valid;
    bool     spo2_valid;
    bool     hrv_valid;
    bool     rr_valid;

} Vitals_Results;

/* ========================================================================
 *  API PUBBLICA
 * ======================================================================== */

/**
 * @brief Inizializza tutti gli stati interni del modulo (filtri, buffer,
 * indici). Da chiamare una volta all'avvio, dopo PPG_Filter_Init().
 */
void Vitals_Init(void);

/**
 * @brief Da chiamare ad ogni nuovo campione disponibile (200 Hz), con i
 * valori RAW (non filtrati) letti dal FIFO del MAX30101.
 *
 * Esegue, in ordine:
 *  - aggiornamento stima DC (media mobile) e AC (segnale filtrato passa-banda
 *    cardiaco, 0.5-5 Hz) per RED e IR
 *  - peak detection sul segnale IR per HR/HRV
 *  - aggiornamento finestra per SpO2 (R ratio)
 *  - alimentazione del filtro di respirazione (0.1-0.4 Hz) sulla componente
 *    DC dell'IR
 *
 * @param raw_red  Campione raw RED (18 bit utili, gia' mascherato dal driver)
 * @param raw_ir   Campione raw IR  (18 bit utili, gia' mascherato dal driver)
 */
void Vitals_ProcessSample(uint32_t raw_red, uint32_t raw_ir, float ac_red_filtered, float ac_ir_filtered);

/**
 * @brief Restituisce l'ultimo valore di HR calcolato (media mobile sugli
 * ultimi intervalli picco-picco validi).
 * @param hr_bpm Puntatore di output.
 * @return true se il valore e' valido (almeno 2 picchi rilevati e in range).
 */
bool Vitals_GetHR(float *hr_bpm);

/**
 * @brief Restituisce l'ultima stima di SpO2.
 * @param spo2_percent Puntatore di output (0-100).
 * @return true se il valore e' valido.
 */
bool Vitals_GetSpO2(float *spo2_percent);

/**
 * @brief Restituisce le metriche HRV time-domain correnti (SDNN, RMSSD),
 * calcolate sugli ultimi VITALS_HRV_NN_BUFFER_LEN intervalli NN disponibili.
 * @param sdnn_ms  Puntatore di output per SDNN in ms.
 * @param rmssd_ms Puntatore di output per RMSSD in ms.
 * @return true se ci sono almeno 2 intervalli NN disponibili.
 */
bool Vitals_GetHRV(float *sdnn_ms, float *rmssd_ms);

/**
 * @brief Restituisce l'ultima stima di frequenza respiratoria.
 * @param rr_brpm Puntatore di output (respiri/min).
 * @return true se il valore e' valido (buffer di respirazione pieno e picco
 * spettrale/zero-crossing trovato in banda).
 */
bool Vitals_GetRR(float *rr_brpm);



/**
 * @brief Helper di comodo: riempie una struct Vitals_Results con tutti gli
 * ultimi valori disponibili in un colpo solo (utile prima di costruire un
 * packet BLE o un campo da salvare in NAND).
 */
void Vitals_GetAllResults(Vitals_Results *out);

#ifdef __cplusplus
}
#endif

#endif /* VITALS_PROCESSING_H */