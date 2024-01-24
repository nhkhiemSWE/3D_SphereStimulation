/**
 * Author: Isabel Rosa, isarosa@mit.edu, 
 * Jay Hilton, jhilton@mit.edu, 
 * Krit Boonsiriseth, talkon@mit.edu
 **/

#include <assert.h>
#include <cilk/cilk.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../common/render.h"
#include "../include/misc_utils.h"

typedef struct renderer_state {
  renderer_spec_t r_spec;
  float *img;
  vector_t plane_normal;
  // newly added below:
  int total_pixels;
  float pixel_size;
  ray_t* origin_rays;
  float precompute1;
  sphere_t* copy_spheres;
} renderer_state_t;

//Additional functions:

/*
@param given a point(x0,y0,z0) and a normal vector of the plane
@return return a point on that plane where x = x0, y = y0 + 1
*/
inline __attribute__((always_inline))
vector_t find_dir_in_plane(vector_t* point, vector_t* normal) {
  vector_t res;
  res.x = point->x;
  res.y = point->y + 1;
  vector_t temp = {res.x, res.y, 0};
  res.z = (float) ((double)qdot(*normal,*point) - (double) qdot(temp,*normal))/((double) normal->z);
  vector_t unit_res = qsubtract(res,*point);
  unit_res = scale(1/qsize(unit_res), unit_res);
  return unit_res;
}

/*
@param: given a sphere with its center's position and radius (r)
@return: the position of the 4 corners of the sphere's bounding square surface from the eye's perspective
*/
inline __attribute__((always_inline))
void get_corners(sphere_t* sphere, vector_t* corners, vector_t* eye){
  // Connect the eye to the center of the sphere, find the intersection of this line and the sphere's surface
  vector_t normal = qsubtract(*eye, sphere->pos);
  vector_t intersect = qadd(sphere->pos, scale(sphere->r, scale(1/qsize(normal), normal)));
  float half_diagonal = (float) ((double) sphere->r * (double) (sqrt(2)));

  // Having the normal vector of the surface plane and the intersect on that plane, find a unit dir in the plane
  vector_t diagonal_1 = find_dir_in_plane(&intersect, &normal);

  corners[0] = qadd(intersect, scale(half_diagonal, diagonal_1));
  corners[1] = qadd(intersect, scale(-half_diagonal, diagonal_1));

  //Find the second diagonal of the bounding square
  vector_t diagonal_2 = qcross(normal, diagonal_1);
  diagonal_2 = scale(1/qsize(diagonal_2), diagonal_2);

  corners[2] = qadd(intersect, scale(half_diagonal, diagonal_2));
  corners[3] = qadd(intersect, scale(-half_diagonal, diagonal_2));
}

/*
@param given a sphere and a render spec
@return the projections of the 8 corners of the bounding box of the sphere on the plane
*/
inline __attribute__((always_inline))
void corner_projection(sphere_t* sphere, renderer_state_t* state, vector_t* projections) {

  get_corners(sphere, projections, &state->r_spec.eye);

  for (int i = 0; i < 4; i ++) {
    //Reference: https://en.wikipedia.org/wiki/Line–plane_intersection#:~:text=In%20analytic%20geometry%2C%20the%20intersection,the%20plane%20but%20outside%20it.
    //The scalar value is -(P0-I0).N/(I.N) where P0 is a point on the plane, I0 is a point on the line, I is the unit vector of the line
    vector_t ray = qsubtract(projections[i], state->r_spec.eye);
    projections[i] = qadd(state->r_spec.eye, 
    scale(-qdot(state->plane_normal, state->r_spec.eye) / qdot(state->plane_normal, ray), ray));
  }
}

/*
@param a sphere and a render spec
@return after finding the projections of the bounding box, find the smallest rectangle that contains all 8 projections
*/
inline __attribute__((always_inline))
void find_bounding_region(sphere_t* sphere, renderer_state_t* state, int* boundaries) {
  double offset = (double)state->r_spec.viewport_size / (double)2;
  vector_t projections[4];

  corner_projection(sphere, state, projections);

  float max_x = qdot(projections[0], state->r_spec.proj_plane_u);
  float max_y = qdot(projections[0], state->r_spec.proj_plane_v);
  float min_x = max_x;
  float min_y = max_y;

  for (int i = 1; i < 4; i++){
    float cur_x = qdot(projections[i], state->r_spec.proj_plane_u);
    float cur_y = qdot(projections[i], state->r_spec.proj_plane_v);

    if (cur_y > max_y) {
      max_y = cur_y;
    }
    if (cur_x > max_x) {
      max_x = cur_x;
    }
    if (cur_x < min_x) {
      min_x = cur_x;
    }
    if (cur_y < min_y) {
      min_y = cur_y;
    }
  }

  boundaries[0] = (int)(min((max_x + offset)/state->pixel_size + 1, state->r_spec.resolution)); //max_x
  boundaries[1] = (int)(min((max_y + offset)/state->pixel_size + 1, state->r_spec.resolution)); //max_y
  boundaries[2] = (int)(max((min_x + offset)/state->pixel_size - 1, 0));          //min_x
  boundaries[3] = (int)(max((min_y + offset)/state->pixel_size - 1, 0));  
}
// End additional functions

renderer_state_t* init_renderer(const renderer_spec_t *spec) {
  renderer_state_t *state = (renderer_state_t*)malloc(sizeof(renderer_state_t));
  state->r_spec = *spec;
  int n_pixels = state->r_spec.resolution * state->r_spec.resolution;
  state->img = calloc(3ull * (size_t) n_pixels, sizeof(float));
  assert(state->img != NULL);
  state->plane_normal = qcross(state->r_spec.proj_plane_u, state->r_spec.proj_plane_v);
  state->total_pixels = (state->r_spec.resolution)*(state->r_spec.resolution);
  state->pixel_size = state->r_spec.viewport_size / state->r_spec.resolution;
  state->origin_rays = calloc((size_t) state->total_pixels, sizeof(ray_t));
  state->precompute1 = -qdot(state->plane_normal, state->r_spec.eye);
  state->copy_spheres = NULL;
  return state;
}

void destroy_renderer(renderer_state_t *state) {
  free(state->img);
  free(state->origin_rays);
  free(state->copy_spheres);
  free(state);
}

// Computes the ray from a given origin (usually the eye location) to the pixel (x, y)
// in image coordinates.
ray_t origin_to_pixel(renderer_state_t *state, int x, int y) {
  ray_t viewingRay;

  // Center image frame
  float us = -state->r_spec.resolution / 2 + x;
  float vs = -state->r_spec.resolution / 2 + y;

  viewingRay.origin = state->r_spec.eye;
  viewingRay.dir =
    qsubtract(
      qadd(
        scale(us * state->pixel_size, state->r_spec.proj_plane_u), 
        scale(vs * state->pixel_size, state->r_spec.proj_plane_v)
      ),
      viewingRay.origin
    );
  viewingRay.dir = scale(1 / qsize(viewingRay.dir), viewingRay.dir);

  return viewingRay;
}

// Determines whether the ray r and the sphere s intersect.
// 
// If the ray and the sphere intersect, writes the distance to the closer intersection
// to `out`, and returns 1. Otherwise, returns 0.
int ray_sphere_intersection(ray_t *r, const sphere_t* s, float *out) {
  vector_t dist = qsubtract(r->origin, s->pos);
  // Uses quadratic formula to compute intersection
  float a = qdot(r->dir, r->dir);
  float b = 2 * qdot(r->dir, dist);
  float c = (float)((double)qdot(dist, dist) - (double)(s->r * s->r));
  float discr = (float)((double)(b * b) - (double)(4 * a * c));

  if (discr >= 0) {
    // Ray hits sphere
    float sqrtdiscr = sqrtf(discr);

    float min_dist;
    if (b >= 0) {
      float sol1 = (float)((double)-b - (double)sqrtdiscr) / (2 * a);
      float sol2 = (float)(2 * (double)c) / ((double)-b - (double)sqrtdiscr);
      min_dist = min(sol1, sol2);
    } else {
      float sol1 = (float)(2 * (double)c) / ((double)-b + (double)sqrtdiscr);
      float sol2 = (float)((double)-b + (double)sqrtdiscr) / (2 * a);
      min_dist = min(sol1, sol2);
    }

    // If new_t > 0 and smaller than original t, we
    // found a new, closer ray-sphere intersection
    if (min_dist > 0) {
      *out = min_dist;
      return 1;
    }
  }

  return 0;
}

// Sorts given spheres by length of the tangent (NOT in-place). 
// Returns a pointer to the sorted spheres. Returned pointer must be freed.
//
// Since the spheres are non-intersecting, this ensures that 
// if sphere S comes before sphere T in this ordering, then 
// sphere S is in front of sphere T in the rendering.
void sort(renderer_state_t* restrict state, const sphere_t* restrict orig_spheres, sphere_t* restrict spheres, int n_spheres) {
  float* distance = calloc((size_t) n_spheres, sizeof(float));
  int* new_ind = calloc((size_t) n_spheres, sizeof(int));
  cilk_for (int i = 0; i < n_spheres; i++){
    distance[i] = (qdist(spheres[i].pos, state->r_spec.eye) * qdist(spheres[i].pos, state->r_spec.eye) - spheres[i].r * spheres[i].r);
  }
  cilk_for(int i = 0; i < n_spheres; i++){
    for (int j = 0; j < n_spheres; j++){
      if (distance[i] > distance[j] || (distance[i] == distance[j] && i > j)){
        new_ind[i]++;
      }
    }
  }
  cilk_for (int i = 0; i < n_spheres; i++){
    spheres[new_ind[i]] = orig_spheres[i];
  }
  free(distance);
  free(new_ind);
}

void render_slice(renderer_state_t* restrict state, int* restrict bounding_region, char* restrict marks,
const sphere_t* restrict spheres, int n_spheres, int x_low, int x_high, int y_low, int y_high){
  for (int i = 0 ; i < n_spheres; i ++) {
    float t = INFINITY;
    material_t currentMat = spheres[i].mat;
    if (bounding_region[i * 4 + 2] >= x_high || bounding_region[i * 4 + 0] <= x_low) continue;
    for (int y = max(y_low, bounding_region[i * 4 + 3]); y < min(y_high, bounding_region[i * 4 + 1]); y++){
      int row = y*state->r_spec.resolution;
      for (int x = max(x_low, bounding_region[i * 4 + 2]); x < min(x_high, bounding_region[i * 4 + 0]); x++) {
        if (!marks[row + x]) {
          if (ray_sphere_intersection(&state->origin_rays[row + x], &spheres[i], &t)){
            marks[row + x] = 1;
            vector_t intersection = qadd(state->origin_rays[row + x].origin, scale(t, state->origin_rays[row + x].dir));

            // Normal vector at intersection point, perpendicular to the surface
            // of the sphere
            vector_t normal = qsubtract(intersection, spheres[i].pos);
            float n_size = qsize(normal);
            // Note: n_size should be the radius of the sphere, which is nonzero.
            normal = scale(1 / n_size, normal);

            double red = 0;
            double green = 0; 
            double blue = 0; 

            for (int j = 0; j < state->r_spec.n_lights; j++) {
              light_t currentLight = state->r_spec.lights[j];
              vector_t intersection_to_light = qsubtract(currentLight.pos, intersection);
              if (qdot(normal, intersection_to_light) <= 0)
                continue;

              ray_t lightRay;
              lightRay.origin = intersection;
              lightRay.dir = scale(1 / qsize(intersection_to_light), intersection_to_light);

              // Calculate Lambert diffusion
              float lambert = qdot(lightRay.dir, normal);
              red += (double)(currentLight.intensity.red * currentMat.diffuse.red *
                              lambert);
              green += (double)(currentLight.intensity.green *
                                currentMat.diffuse.green * lambert);
              blue += (double)(currentLight.intensity.blue * currentMat.diffuse.blue *
                              lambert);
              }
              state->img[(x + y * state->r_spec.resolution) * 3 + 0] = min((float)red, 1.0);
              state->img[(x + y * state->r_spec.resolution) * 3 + 1] = min((float)green, 1.0);
              state->img[(x + y * state->r_spec.resolution) * 3 + 2] = min((float)blue, 1.0);
          }
        }
      }
    }
  }
}

const float* render(renderer_state_t *state, const sphere_t *spheres, int n_spheres) {
  if (state->copy_spheres == NULL) {
    state->copy_spheres = clone(spheres, sizeof(sphere_t )* n_spheres);
  }
  sphere_t* sorted_spheres = state->copy_spheres;
  sort(state, spheres, sorted_spheres, n_spheres);

  // Compute all bounding regions in parallel
  int* bounding_region = malloc(n_spheres * 4 * sizeof(int));
  cilk_for (int i = 0; i < n_spheres; i++){
    find_bounding_region(&sorted_spheres[i], state, &bounding_region[i * 4]);
  } 
  char* marks = calloc((size_t)state->total_pixels, sizeof(char));

  // Calculate origin rays
  cilk_for (int y = 0; y < state->r_spec.resolution; y++){
    int row = y * state->r_spec.resolution;
    for (int x = 0; x < state->r_spec.resolution; x++){
      state->origin_rays[row + x] = origin_to_pixel(state, x, y);
    }
  }

  int resolution = state->r_spec.resolution;
  int slice[5] = {0, resolution / 4, resolution / 4 * 2, resolution / 4 * 3, resolution};
  cilk_scope{
    for (int i = 0; i < 4; i++){
      for (int j = 0; j < 4; j++){
        cilk_spawn render_slice(state, bounding_region, marks, sorted_spheres, n_spheres, slice[i], slice[i+1], slice[j], slice[j+1]);
      }
    }
  }
  free(bounding_region);
  free(marks);
  return state->img;
}