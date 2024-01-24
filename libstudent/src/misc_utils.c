/**
 * Authors: Isabel Rosa, isrosa@mit.edu and Jay Hilton, jhilton@mit.edu
 **/
 
#include "../include/misc_utils.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void *clone(const void *src, size_t nbytes) {
  void *dst = malloc(nbytes);
  if (dst == NULL) return NULL;
  memcpy(dst, src, nbytes);
  return dst;
}

renderer_spec_t clone_renderer_spec(const renderer_spec_t *src) {
  renderer_spec_t spec = *src;
  spec.lights = clone(src->lights, sizeof(light_t) * src->n_lights);
  return spec;
}

simulator_spec_t clone_simulator_spec(const simulator_spec_t *src) {
  simulator_spec_t spec = *src;
  spec.spheres = clone(src->spheres, sizeof(sphere_t) * src->n_spheres);
  return spec;
}

