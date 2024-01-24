#ifndef REF_TEST_H
#define REF_TEST_H

#include "./types.h"

// The below functions are for frame-by-frame reference testing.

/**
 * @brief Compare the two provided images.
 *
 * @param ref reference image
 * @param test test image
 * @param height height of both images
 * @param width width of both images
 * @return statistics from comparing the images
 */
ref_stats_t compare_images(const float *ref, const float *test, int height,
                           int width);

/**
 * @brief Run n_frames initialized by start_spec.
 *
 * @param[in] start_spec test specification to run
 * @param[in] n_frames number of frames to run test with
 * @return output of the test
 */
ref_out_t run_test(const ref_spec_t *start_spec, size_t n_frames);

#endif // REF_TEST_H