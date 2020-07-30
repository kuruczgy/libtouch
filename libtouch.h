#ifndef LIBTOUCH_H
#define LIBTOUCH_H
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* Represents the affine transformation:
 * [ s -r t1 ]
 * [ r  s t2 ]
 * [ 0  0  1 ]
 *
 * ('rt' abbreviates 'rigid-body transformation') */
struct libtouch_rt {
	float s, r;
	union {
		struct { float t1, t2; };
		float t[2];
	};
};

/* Represents an axis-aligned bounding box. */
struct aabb {
	union {
		struct { float x, y, w, h; };
		struct { float pos[2]; float size[2]; };
	};
};

/* Utilities for dealing with rigid-body transformations. */
extern const struct libtouch_rt libtouch_rt_identity;
struct libtouch_rt libtouch_rt_mul(const struct libtouch_rt *a,
	const struct libtouch_rt *b);
float libtouch_rt_rotation(const struct libtouch_rt *rt);
float libtouch_rt_scaling(const struct libtouch_rt *rt);

struct libtouch_surface;
struct libtouch_area;

struct libtouch_surface *libtouch_surface_create();
struct libtouch_area *libtouch_surface_add_area(
	struct libtouch_surface *surf, struct aabb aabb);
void libtouch_surface_destroy(struct libtouch_surface *surf);

/* Use these functions to inform a surface of touch events. */
void libtouch_surface_down(struct libtouch_surface *surf, uint32_t time,
	int id, const float pos[2]);
void libtouch_surface_up(struct libtouch_surface *surf, uint32_t time, int id);
void libtouch_surface_motion(struct libtouch_surface *surf, uint32_t time,
	int id, const float pos[2]);

struct libtouch_rt libtouch_area_get_transform(struct libtouch_area *area);

#endif
