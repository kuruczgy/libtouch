#ifndef LIBTOUCH_H
#define LIBTOUCH_H
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

enum libtouch_groups {
	LIBTOUCH_T,
	LIBTOUCH_TSR,
};

enum libtouch_area_flags {
	LIBTOUCH_V = 1 << 0,
};

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

/* We represent axis-aligned bounding boxes with a `float[4]`, where the first
 * two items give the position of the upper left corner, and the last two items
 * give the sizes.
 */

/* Utilities for dealing with rigid-body transformations. */
extern const struct libtouch_rt libtouch_rt_identity;
struct libtouch_rt libtouch_rt_mul(const struct libtouch_rt *a,
	const struct libtouch_rt *b);
float libtouch_rt_rotation(const struct libtouch_rt *rt);
float libtouch_rt_scaling(const struct libtouch_rt *rt);

struct libtouch_surface;
struct libtouch_area;

struct libtouch_gesture_data {
	uint32_t t;
	struct libtouch_rt rt;
	float V[2];
};

struct libtouch_area_opts {
	/* all callbacks can be set to NULL to disable them */

	/* environment for all callbacks */
	void *env;

	/* gesture start callback */
	void (*start)(void *env, struct libtouch_gesture_data data);

	/* callback during the gesture, when the transform changes */
	void (*move)(void *env, struct libtouch_gesture_data data);

	/* gesture end callback */
	void (*end)(void *env, struct libtouch_gesture_data data);

	/* various options */
	enum libtouch_groups g;
	enum libtouch_area_flags flags;
};

struct libtouch_surface *libtouch_surface_create();
struct libtouch_area *libtouch_surface_add_area(struct libtouch_surface *surf,
	const float *aabb, struct libtouch_area_opts opts);
void libtouch_surface_remove_area(struct libtouch_surface *surf,
	struct libtouch_area *area);
void libtouch_surface_destroy(struct libtouch_surface *surf);

/* Use these functions to inform a surface of touch events. */
void libtouch_surface_down(struct libtouch_surface *surf, uint32_t time,
	int id, const float pos[2]);
void libtouch_surface_up(struct libtouch_surface *surf, uint32_t time, int id);
void libtouch_surface_motion(struct libtouch_surface *surf, uint32_t time,
	int id, const float pos[2]);

struct libtouch_rt libtouch_area_get_transform(struct libtouch_area *area);

#endif
