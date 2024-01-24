/**
 * Author: Jay Hilton, jhilton@mit.edu
 **/

#ifndef MISC_UTILS_H
#define MISC_UTILS_H

#include <stddef.h>
#include <math.h>
#include "../../common/types.h"

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

inline __attribute__((always_inline))
static vector_t qsubtract(vector_t v1, vector_t v2) {
  vector_t v;
  v.x = (float)((double)v1.x - (double)v2.x);
  v.y = (float)((double)v1.y - (double)v2.y);
  v.z = (float)((double)v1.z - (double)v2.z);
  return v;
}

inline __attribute__((always_inline))
static float qdot(vector_t v1, vector_t v2) {
  float x = v1.x * v2.x;
  float y = v1.y * v2.y;
  float z = v1.z * v2.z;
  return (float)((double)x + (double)y + (double)z);
}

inline __attribute__((always_inline))
static vector_t qcross(vector_t v1, vector_t v2) {
  vector_t v;
  v.x = (float)((double)(v1.y * v2.z) - (double)(v1.z * v2.y));
  v.y = (float)((double)(v1.z * v2.x) - (double)(v1.x * v2.z));
  v.z = (float)((double)(v1.x * v2.y) - (double)(v1.y * v2.x));
  return v;
}

inline __attribute__((always_inline))
static float qsize(vector_t v) {
  float x = v.x * v.x;
  float y = v.y * v.y;
  float z = v.z * v.z;
  return sqrt((float)((double)x + (double)y + (double)z));
}

inline __attribute__((always_inline))
static vector_t scale(float c, vector_t v1) {
  vector_t v;
  v.x = v1.x * c;
  v.y = v1.y * c;
  v.z = v1.z * c;
  return v;
}

inline __attribute__((always_inline))
static vector_t qadd(vector_t v1, vector_t v2) {
  vector_t v;
  v.x = (float)((double)v1.x + (double)v2.x);
  v.y = (float)((double)v1.y + (double)v2.y);
  v.z = (float)((double)v1.z + (double)v2.z);
  return v;
}

inline __attribute__((always_inline))
static float qdist(vector_t v1, vector_t v2) {
  double f = ((double)v1.x - (double)v2.x) * ((double)v1.x - (double)v2.x);
  f += ((double)v1.y - (double)v2.y) * ((double)v1.y - (double)v2.y);
  f += ((double)v1.z - (double)v2.z) * ((double)v1.z - (double)v2.z);
  return sqrt((float)f);
}

/**
 * @brief Copy the first nbytes of src into a new, heap-allocated array, and return the new array. Returns NULL if allocation fails.
 * 
 * @param src array to copy
 * @param nbytes number of bytes to copy from src
 * @return the new, heap-allocated array, or NULL on allocation failure
 */
void *clone(const void *src, size_t nbytes);

renderer_spec_t clone_renderer_spec(const renderer_spec_t *src);

simulator_spec_t clone_simulator_spec(const simulator_spec_t *src);

#endif // MISC_UTILS_H