#ifndef __PPG_FILTER_H__
#define __PPG_FILTER_H__

#include "arm_math.h"
#include <stdbool.h>

/**
 * @brief Initializes the PPG bandpass filter.
 */
void PPG_Filter_Init(void);

/**
 * @brief Filters a single PPG sample (red and ir).
 * @param raw_red The raw red sample.
 * @param raw_ir The raw ir sample.
 * @param filtered_red Pointer to store the filtered red sample.
 * @param filtered_ir Pointer to store the filtered ir sample.
 */
void PPG_Filter_ProcessSamples(float32_t raw_red, float32_t raw_ir, float32_t *filtered_red, float32_t *filtered_ir);

void compute_butterworth_biquads(int order, double fc, double fs, bool is_highpass, float32_t *coeffs_out);

#endif /* __PPG_FILTER_H__ */
