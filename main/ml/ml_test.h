#ifndef ML_ML_TEST_H
#define ML_ML_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/**
 * @brief Run ML performance tests
 * @return ESP_OK on success
 */
esp_err_t ml_test_performance(void);

/**
 * @brief Test model accuracy with known gestures
 * @return ESP_OK on success
 */
esp_err_t ml_test_accuracy(void);

/**
 * @brief Benchmark inference speed
 * @return ESP_OK on success
 */
esp_err_t ml_test_benchmark(void);

#ifdef __cplusplus
}
#endif

#endif /* ML_ML_TEST_H */