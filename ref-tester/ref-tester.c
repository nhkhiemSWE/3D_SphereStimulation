#include "./ref-tester.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "../misc_utils.h"

ref_stats_t compare_images(const float *ref, const float *test,
                           const int height, const int width) {
  // printf("new frame \n");               
  float sum = 0;
  float sumSq = 0;
  float min = 1;
  float max = 0;

  float *diff = malloc(sizeof(float) * (size_t)height * (size_t)width);
  bool correct = true;

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      float red_diff =
          fabs(ref[(x + y * width) * 3] - test[(x + y * width) * 3]);
      float green_diff =
          fabs(ref[(x + y * width) * 3 + 1] - test[(x + y * width) * 3 + 1]);
      float blue_diff =
          fabs(ref[(x + y * width) * 3 + 2] - test[(x + y * width) * 3 + 2]);

      float px_diff = (red_diff + green_diff + blue_diff) / 3;
      if (px_diff != 0) {
        printf("%d %d (%f %f %f) (%f %f %f)\n", x, y, ref[(x + y * width) * 3], ref[(x + y * width) * 3 + 1], ref[(x + y * width) * 3 + 2], test[(x + y * width) * 3], test[(x + y * width) * 3 + 1], test[(x + y * width) * 3 + 2]);
        correct = false;
      }

      sum += px_diff;
      sumSq += px_diff * px_diff;

      if (px_diff > max) {
        max = px_diff;
      }
      if (px_diff < min) {
        min = px_diff;
      }
      diff[(x + y * width)] = px_diff;
    }
  }

  const image_diff_t image_diff = {
      .buf = diff, .height = height, .width = width};
  const ref_stats_t stats = {
      .correct = correct,
      .avg = sum / (height * width),
      .std_dev =
          sqrt((sumSq - (sum * sum) / (width * height)) / (width * height)),
      .min = min,
      .max = max,
      .diff = image_diff,
  };
  return stats;
}

/**
 * @brief Initialize impl with the provided state, returning the image that
 * results from simulating and rendering one frame.
 *
 * @param[in, out] spheres_out a pointer within which a copy of the spheres
 * generated is returned, if pointer is not null
 * @param[in] spec specification of implementation and simulation to run
 * @return the generated image, which is heap-allocated and the caller's
 * responsibility to free
 */
static const float *end_to_end_reinit(sphere_t **const spheres_out,
                                      e2e_spec_t *const spec) {
  init_impl(&spec->impl, &spec->r_spec, &spec->s_spec);
  sphere_t* spheres = spec->impl.simulate(spec->impl.simulate_this);
  // We have no guarantees about if destroy_impl might free image, which is a
  // reference inside the renderer. Therefore, we copy it here, and then destroy
  // the implementation.
  const size_t img_bytes =
      3 * sizeof(float) * (size_t)spec->r_spec.resolution * (size_t)spec->r_spec.resolution;
  float* image =
      clone(spec->impl.render(spec->impl.renderer_this, spheres, spec->s_spec.n_spheres), img_bytes);

  if (spheres_out != NULL) {
    *spheres_out = clone(spheres, spec->s_spec.n_spheres * sizeof(sphere_t));
  }

  destroy_impl(&spec->impl);
  return image;
}

/**
 * @brief Generate a new end-to-end spec with the provided spheres.
 *
 * @param[in] src end-to-end spec to clone except for spheres
 * @param[in] spheres spheres to use in the new end-to-end spec
 * @return an end-to-end spec that is the same as src except with the provided
 * spheres as spheres, but does not alias src
 */
static e2e_spec_t with_spheres(const e2e_spec_t *const src,
                               sphere_t *const spheres) {
  e2e_spec_t new_spec;
  new_spec.impl = src->impl;
  new_spec.r_spec = clone_renderer_spec(&src->r_spec);
  new_spec.s_spec = clone_simulator_spec(&src->s_spec);
  free((sphere_t *)new_spec.s_spec.spheres);

  new_spec.s_spec.spheres = spheres;
  return new_spec;
}

static int height_from_spec(const e2e_spec_t *const spec) {
  return spec->r_spec.resolution;
}

static int width_from_spec(const e2e_spec_t *const spec) {
  return spec->r_spec.resolution;
}

static ref_stats_t run_frame(ref_spec_t *const spec,
                             ref_spec_t *next_spec) {
  assert(height_from_spec(&spec->ref) == height_from_spec(&spec->test) &&
         "heights under test should agree");
  assert(width_from_spec(&spec->ref) == width_from_spec(&spec->test) &&
         "widths under test should agree");

  const int height = height_from_spec(&spec->ref);
  const int width = width_from_spec(&spec->ref);

  // First, we have to generate the outputs for comparison.
  sphere_t *ref_spheres, *test_spheres;
  const float *ref_image = end_to_end_reinit(&ref_spheres, &spec->ref);
  const float *test_image = end_to_end_reinit(&test_spheres, &spec->test);

  const ref_stats_t stats =
      compare_images(ref_image, test_image, height, width);
  free((void *)ref_image);
  free((void *)test_image);

  ref_spec_t next;
  next.ref = with_spheres(&spec->ref, ref_spheres);
  if (spec->reset_sim) {
    free(test_spheres);
    next.test = with_spheres(
        &spec->test,
        clone(ref_spheres, sizeof(sphere_t) * spec->ref.s_spec.n_spheres));
  } else {
    next.test = with_spheres(&spec->test, test_spheres);
  }
  next.reset_sim = spec->reset_sim;
  *next_spec = next;

  return stats;
}

static e2e_spec_t clone_e2e_spec(const e2e_spec_t *const spec) {
  const e2e_spec_t new = {
      .impl = spec->impl,
      .r_spec = clone_renderer_spec(&spec->r_spec),
      .s_spec = clone_simulator_spec(&spec->s_spec),
  };

  return new;
}

static ref_spec_t clone_ref_spec(const ref_spec_t *const spec) {
  const ref_spec_t new = {.ref = clone_e2e_spec(&spec->ref),
                          .test = clone_e2e_spec(&spec->test),
                          .reset_sim = spec->reset_sim};

  return new;
}

ref_out_t run_test(const ref_spec_t *const start_spec, size_t n_frames) {
  assert(n_frames > 0);

  ref_stats_t *stats = malloc(sizeof(ref_stats_t) * n_frames);
  if (stats == NULL)
    exit(OOM_ERROR);

  ref_spec_t cur_spec = clone_ref_spec(start_spec);
  ref_spec_t next_spec;

  for (size_t f = 0; f < n_frames; f++) {
    stats[f] = run_frame(&cur_spec, &next_spec);

    destroy_ref_spec(&cur_spec);
    cur_spec = next_spec;
  }

  destroy_ref_spec(&next_spec);

  const ref_out_t out = {
      .n_elts = n_frames,
      .stats = stats,
  };

  return out;
}
