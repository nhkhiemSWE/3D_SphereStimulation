/**
 * Author: Isabel Rosa, isrosa@mit.edu
 **/

#include <assert.h>
#include <cilk/cilk.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "../../common/simulate.h"
#include "../include/misc_utils.h"

typedef struct simulator_state {
  simulator_spec_t s_spec;
  sphere_t *spheres;
} simulator_state_t;

simulator_state_t* init_simulator(const simulator_spec_t *spec) {
  simulator_state_t *state = (simulator_state_t*)malloc(sizeof(simulator_state_t));
  state->s_spec = *spec;
  state->spheres = malloc(2 * spec->n_spheres * sizeof(sphere_t));
  assert(state->spheres != NULL);
  memcpy(state->spheres, spec->spheres, sizeof(sphere_t) * spec->n_spheres);
  memcpy(state->spheres + state->s_spec.n_spheres, spec->spheres, sizeof(sphere_t) * state->s_spec.n_spheres);
  return state;
}

void destroy_simulator(simulator_state_t* state) {
  free(state->spheres);
  free(state);
}

typedef struct {
  double x, y, z;
} double_vector_t;

void update_accelerations(sphere_t *spheres, int n_spheres, double g) {
  double* buffer = calloc(3ull * (size_t) n_spheres * (size_t) n_spheres, sizeof(double));
  cilk_for (int i = 0; i < n_spheres; i++){
    for (int j = i + 1; j < n_spheres; j++){
        vector_t j_minus_i = qsubtract(spheres[j].pos, spheres[i].pos);
        double mag = qsize(j_minus_i);
        double mag3 = mag * mag * mag;
        float i_term = g * spheres[j].mass / mag3;
        float j_term = g * spheres[i].mass / mag3;
        int i_index = i * n_spheres * 3 + j * 3;
        int j_index = j * n_spheres * 3 + i * 3;
        buffer[i_index] += i_term * j_minus_i.x;
        buffer[i_index + 1] += i_term * j_minus_i.y;
        buffer[i_index + 2] += i_term * j_minus_i.z;
        buffer[j_index] -= j_term * j_minus_i.x;
        buffer[j_index + 1] -= j_term * j_minus_i.y;
        buffer[j_index + 2] -= j_term * j_minus_i.z;
    }
  }
  cilk_for (int i = 0; i < n_spheres; i++){
    double_vector_t tmp_vec = {0, 0, 0};
    for (int j = 0; j < n_spheres; j++){
      if (i == j) continue;
      tmp_vec.x += buffer[i * n_spheres * 3 + j * 3];
      tmp_vec.y += buffer[i * n_spheres * 3 + j * 3 + 1];
      tmp_vec.z += buffer[i * n_spheres * 3 + j * 3 + 2];
    }
    spheres[i + n_spheres].accel.x = tmp_vec.x;
    spheres[i + n_spheres].accel.y = tmp_vec.y;
    spheres[i + n_spheres].accel.z = tmp_vec.z;
  }
  free(buffer);
}

void update_velocities_and_positions(sphere_t *spheres, int n_spheres, float t) {
  cilk_for (int i = 0; i < n_spheres; i++) {
    spheres[i + n_spheres].vel = qadd(spheres[i].vel, scale(t, spheres[i].accel));
    spheres[i + n_spheres].pos = qadd(spheres[i].pos, scale(t, spheres[i].vel));
  }
}

// runs simulation for minCollisionTime timesteps
// perform collision between spheres at indices i and j
void do_ministep(sphere_t *spheres, int n_spheres, double g, float minCollisionTime, int i, int j) {
  update_accelerations(spheres, n_spheres, g);
  update_velocities_and_positions(spheres, n_spheres, minCollisionTime);

  cilk_for (int k = 0; k < n_spheres; k++) {
    spheres[k] = spheres[k + n_spheres];
  }

  if (i == -1 || j == -1) {
    return;
  }

  vector_t distVec = qsubtract(spheres[i].pos, spheres[j].pos);
  float scale1 = 2 * spheres[j].mass /
                 (float)((double)spheres[i].mass + (double)spheres[j].mass);
  float scale2 = 2 * spheres[i].mass /
                 (float)((double)spheres[i].mass + (double)spheres[j].mass);
  float distNorm = qdot(distVec, distVec);
  vector_t velDiff = qsubtract(spheres[i].vel, spheres[j].vel);
  vector_t scaledDist = scale(qdot(velDiff, distVec) / distNorm, distVec);
  spheres[i].vel = qsubtract(spheres[i].vel, scale(scale1, scaledDist));
  spheres[j].vel = qsubtract(spheres[j].vel, scale(-1 * scale2, scaledDist));
}

// Check if the spheres at indices i and j collide in the next
// timeToCollision timesteps
// 
// If so, modifies timeToCollision to be the time until spheres i and j collide.
int check_for_collision(sphere_t *spheres, int i, int j, float *timeToCollision) {
  vector_t distVec = qsubtract(spheres[i].pos, spheres[j].pos);
  float dist = qsize(distVec);
  float sumRadii = (float)((double)spheres[i].r + (double)spheres[j].r);

  // Shift frame of reference to act like sphere i is stationary
  // Not adjusting for acceleration because our simulation does not adjust for acceleration
  vector_t movevec = qsubtract(spheres[j].vel, spheres[i].vel);

  // Distance that sphere j moves in timeToCollision time
  float moveDist = (float)((double)qsize(movevec) * (double)*timeToCollision);

  // Break if the length the sphere moves in timeToCollision time is less than
  // distance between the centers of these spheres minus their radii
  if ((double)moveDist < (double)dist - (double)sumRadii ||
      (movevec.x == 0 && movevec.y == 0 && movevec.z == 0)) {
    return 0;
  }

  vector_t unitMovevec = scale(1 / qsize(movevec), movevec);

  // distAlongMovevec = ||distVec|| * cos(angle between unitMovevec and distVec)
  float distAlongMovevec = qdot(unitMovevec, distVec);

  // Check that sphere j is moving towards sphere i
  if (distAlongMovevec <= 0) {
    return 0;
  }

  float jToMovevecDistSq =
      (float)((double)(dist * dist) -
              (double)(distAlongMovevec * distAlongMovevec));

  // Break if the closest that sphere j will get to sphere i is more than
  // the sum of their radii
  float sumRadiiSquared = sumRadii * sumRadii;
  if (jToMovevecDistSq >= sumRadiiSquared) {
    return 0;
  }

  // We now have jToMovevecDistSq and sumRadii, two sides of a right triangle.
  // Use these to find the third side, sqrt(T)
  float extraDist = (float)((double)sumRadiiSquared - (double)jToMovevecDistSq);

  // Draw out the spheres to check why this is the distance sphere j moves
  // before hitting sphere i;)
  float distance = (float)((double)distAlongMovevec - (double)sqrt(extraDist));

  // Break if the distance sphere j has to move to touch sphere i is too big
  if (distance < 0 || moveDist < distance) {
    return 0;
  }

  *timeToCollision = distance / qsize(movevec);
  return 1;
}

void do_timestep(simulator_state_t* state, float timeStep, float* collisionTimes, int* collideWith) {
  float timeLeft = timeStep;
  
  // If collisions are getting too frequent, we cut time step early
  // This allows for smoother rendering without losing accuracy
  
  while (timeLeft > 0.000001) {
    float minCollisionTime = timeLeft;
    int indexCollider1 = -1;
    int indexCollider2 = -1;
    
    for (int i = 0; i < state->s_spec.n_spheres; i++) {
      if (collisionTimes[i] < minCollisionTime && collisionTimes[i] > 0){
        minCollisionTime = collisionTimes[i];
        indexCollider1 = i;
        indexCollider2 = collideWith[i];
      }
    }
    
    if (indexCollider1 != -1){
      minCollisionTime = timeLeft;
      check_for_collision(state->spheres, indexCollider1, indexCollider2, &minCollisionTime);
    }
    for (int i = 0; i < state->s_spec.n_spheres; i++){
      collisionTimes[i] -= minCollisionTime;
    }

    do_ministep(state->spheres, state->s_spec.n_spheres, state->s_spec.g, minCollisionTime, indexCollider1, indexCollider2);

    timeLeft = timeLeft - minCollisionTime;

    if (indexCollider1 != -1){
      collisionTimes[indexCollider1] = timeLeft;
      for (int j = 0; j < state->s_spec.n_spheres; j++) {
        if (j == indexCollider1) continue;
        if (check_for_collision(state->spheres, indexCollider1, j, &collisionTimes[indexCollider1])){
          collideWith[indexCollider1] = j;
        }
      }
      collisionTimes[indexCollider2] = timeLeft;
      for (int j = 0; j < state->s_spec.n_spheres; j++) {
        if (j == indexCollider2) continue;
        if (check_for_collision(state->spheres, indexCollider2, j, &collisionTimes[indexCollider2])){
          collideWith[indexCollider2] = j;
        }
      }
    }
    
  }
}

sphere_t* simulate(simulator_state_t* state) {
  int n_spheres = state->s_spec.n_spheres;
  float timeStep = n_spheres > 1 ? (1 / log(n_spheres)) : 1;
  float* collisionTimes = calloc((size_t) n_spheres, sizeof(float));
  int* collideWith = calloc((size_t) n_spheres, sizeof(int));
  cilk_for (int i = 0; i < state->s_spec.n_spheres; i++) {
      collisionTimes[i] = timeStep;
      for (int j = i+1; j < state->s_spec.n_spheres; j++) {
        if (check_for_collision(state->spheres, i, j, &collisionTimes[i])){
          collideWith[i] = j;
        }
      }
    }
  do_timestep(state, timeStep, collisionTimes, collideWith);
  free(collisionTimes);
  free(collideWith);
  return state->spheres;
}
