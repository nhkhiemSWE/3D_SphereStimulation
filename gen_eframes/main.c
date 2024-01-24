#include "../common/types.h"
#include "../serde.h"
#include "../vtable.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h> 

struct opts {
  vtable_t vtable;
  const char *s_filename;
  const char *r_filename;
  const char *out_filename;
  size_t n_frames;
};


static void usage() { fprintf(stderr, "./gen_eframes [-r -s] sim_spec renderer_spec output_file\nBy default uses staff simulator and renderer, -r and -s switch to student versions.\n"); }

static int argparse(int argc, char *const argv[], struct opts *const o) {
  bool staff_renderer = true;
  bool staff_simulator = true;
  o->n_frames = 12;
  int ch;

  while ((ch = getopt(argc, argv, "n:rs")) != -1) {
    switch (ch) {
      case 'n':
      if (1 != sscanf(optarg, "%zu", &o->n_frames))
        goto error;
      break;
    case 'r':
      staff_renderer = false;
      break;
    case 's':
      staff_simulator = false;
      break;
    default:
      goto error;
    }
  }

  argc -= optind;
  argv += optind;

  if (argc != 3) {
    goto error;
  }
  o->s_filename = argv[0];
  o->r_filename = argv[1];
  o->out_filename = argv[2];
  if (staff_renderer && staff_simulator) {
    o->vtable = staff_all();
  } else if (!staff_renderer && staff_simulator) {
    o->vtable = student_renderer();
  } else if (staff_renderer && !staff_simulator) {
    o->vtable = student_simulator();
  } else {
    o->vtable = student_all();
  }
  return 0;

error:
  usage();
  return -1;
}

int main(int argc, char *const argv[]) {

  struct opts o;
  if (argparse(argc, argv, &o)) {
    return -1;
  }


  FILE *s_f = fopen(o.s_filename, "rb");
  FILE *r_f = fopen(o.r_filename, "rb");

  if (s_f == NULL | r_f == NULL) {
    fprintf(stderr, "gen_eframes: could not open %s or %s\n", o.s_filename, o.r_filename);
    exit(1);
  }

  simulator_spec_t s_spec;
  renderer_spec_t r_spec;

  if (deser_renderer_spec(&r_spec, r_f)) {
    fprintf(stderr, "gen_eframes: could not read render spec from %s\n", o.r_filename);
    exit(1);
  }

  if (deser_simulator_spec(&s_spec, s_f)) {
    fprintf(stderr, "gen_eframes: could not read simulate spec from %s\n",
            o.s_filename);
    exit(1);
  }

  // Run for 24 frames and output the serialized results to out.

  init_impl(&o.vtable, &r_spec, &s_spec);

  frames_t out = {
      .height = r_spec.resolution,
      .width = r_spec.resolution,
      .is_diff = false,
      .n_frames = 12,
  };

  const size_t floats_per_frame = 3 * r_spec.resolution * r_spec.resolution;

  out.buf = malloc(sizeof(float) * floats_per_frame * out.n_frames);
  if (out.buf == NULL) {
    fprintf(stderr, "gen_eframes: oom\n");
    exit(1);
  }

  for (size_t f = 0; f < out.n_frames; f++) {
    sphere_t* spheres = o.vtable.simulate(o.vtable.simulate_this);
    const float *frame = o.vtable.render(o.vtable.renderer_this, spheres, s_spec.n_spheres);

    float *buf = out.buf + (f * floats_per_frame);
    memcpy(buf, frame, floats_per_frame * sizeof(float));
  }

  FILE *out_f = fopen(o.out_filename, "wb");
  if (out_f == NULL) {
    fprintf(stderr, "gen_eframes: could not open %s as writeable\n", o.out_filename);
    exit(1);
  }

  if (ser_frames(out_f, &out)) {
    fprintf(stderr, "gen_eframes: could not serialize frames\n");
    exit(1);
  }

  destroy_simulator_spec(&s_spec);
  destroy_renderer_spec(&r_spec);
  destroy_impl(&o.vtable);
  free(out.buf);

  fclose(s_f);
  fclose(r_f);
  fclose(out_f);

  return 0;
}
