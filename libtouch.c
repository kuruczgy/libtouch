#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "libtouch.h"

/* Represents an axis-aligned bounding box. */
struct aabb {
	union {
		struct { float x, y, w, h; };
		struct { float pos[2]; float size[2]; };
	};
};

struct point_pair { float a[2], b[2]; };
typedef bool(*point_pair_iterator)(void *cl, struct point_pair *res);

/*
 * This function is basically the core of the whole library.
 * See "Advanced algorithms for manipulating 2D objects on touch screens":
 * https://trepo.tuni.fi/bitstream/handle/123456789/24173/palen.pdf
 */
struct libtouch_rt estimate_translation_scaling_rotation(
		void *cl, point_pair_iterator iter)
{
	float a1 = 0, b1 = 0, c1 = 0, d1 = 0;
	float a2 = 0, b2 = 0, ac = 0, ad = 0, bc = 0, bd = 0;

	int N = 0;
	struct point_pair pair;
	while (iter(cl, &pair)) {
		++N;

		float a = pair.a[0], b = pair.a[1];
		float c = pair.b[0], d = pair.b[1];

		a1 += a;
		b1 += b;
		c1 += c;
		d1 += d;

		a2 += a * a;
		b2 += b * b;

		ac += a * c;
		ad += a * d;
		bc += b * c;
		bd += b * d;
	}

	float g = N * a2 + N * b2 - a1 * a1 - b1 * b1;

	if (g < 1e-1) { // TODO: epsilon
		if (N == 0) {
			return (struct libtouch_rt) {
				.s = 1, .r = 0, .t1 = 0, .t2 = 0
			};
		}
		return (struct libtouch_rt) {
			.s = 1, .r = 0, .t1 = (c1 - a1) / N, .t2 = (d1 - b1) / N
		};
	}

	float acbd = ac + bd, adbc = ad - bc;

	return (struct libtouch_rt) {
		.s = (N * acbd - a1 * c1 - b1 * d1) / g,
		.r = (N * adbc + b1 * c1 - a1 * d1) / g,
		.t1 = (-a1 * acbd + b1 * adbc + a2 * c1 + b2 * c1) / g,
		.t2 = (-b1 * acbd - a1 * adbc + a2 * d1 + b2 * d1) / g
	};
}
struct libtouch_rt estimate_translation(void *cl, point_pair_iterator iter)
{
	float a1 = 0, b1 = 0, c1 = 0, d1 = 0;

	int N = 0;
	struct point_pair pair;
	while (iter(cl, &pair)) {
		++N;

		float a = pair.a[0], b = pair.a[1];
		float c = pair.b[0], d = pair.b[1];

		a1 += a;
		b1 += b;
		c1 += c;
		d1 += d;
	}

	if (N < 1) {
		return libtouch_rt_identity;
	}

	return (struct libtouch_rt){
		.s = 1,
		.r = 0,
		.t1 = (c1 - a1) / N,
		.t2 = (d1 - b1) / N
	};
}

static struct libtouch_rt (*est_fn[])(void *, point_pair_iterator) = {
	[LIBTOUCH_T] = estimate_translation,
	[LIBTOUCH_TSR] = estimate_translation_scaling_rotation
};

static bool aabb_contains(const float *aabb, const float p[static 2])
{
	return p[0] >= aabb[0]
		&& p[0] < aabb[0] + aabb[2]
		&& p[1] >= aabb[1]
		&& p[1] < aabb[1] + aabb[3];
}

struct libtouch_rt libtouch_rt_mul(const struct libtouch_rt *a,
		const struct libtouch_rt *b)
{
	return (struct libtouch_rt){
		.s = a->s * b->s - a->r * b->r,
		.r = a->s * b->r + a->r * b->s,
		.t1 = a->t1 + a->s * b->t1 - a->r * b->t2,
		.t2 = a->t2 + a->r * b->t1 + a->s * b->t2
	};
}

float libtouch_rt_rotation(const struct libtouch_rt *rt)
{
	return atan2(rt->r, rt->s);
}

float libtouch_rt_scaling(const struct libtouch_rt *rt)
{
	return hypot(rt->r, rt->s);
}

const struct libtouch_rt libtouch_rt_identity =
	{ .s = 1, .r = 0, .t1 = 0, .t2 = 0 };

/* a minimal logarithmic growth vector implementation */
struct vec {
	void *d;
	size_t len, cap;
	size_t itemsize;
};
static void vec_realloc(struct vec *v)
{
	assert(v->cap > v->len);
	v->d = realloc(v->d, v->cap * v->itemsize);
}
static void vec_append(struct vec *v, const void *n)
{
	if (v->len == v->cap) {
		v->cap *= 2;
		if (v->cap < 1) {
			v->cap = 1;
		}
		vec_realloc(v);
	}
	memcpy(v->d + v->len * v->itemsize, n, v->itemsize);
	++v->len;
}
static void *vec_get(struct vec *v, size_t i)
{
	assert(i >= 0 && i < v->len);
	return v->d + i * v->itemsize;
}
static const void *vec_get_c(const struct vec *v, size_t i)
{
	assert(i >= 0 && i < v->len);
	return v->d + i * v->itemsize;
}
static void vec_remove(struct vec *v, size_t i)
{
	if (v->len > i + 1) {
		memmove(v->d + i * v->itemsize, v->d + (i + 1) * v->itemsize,
			v->itemsize * (v->len - i - 1));
	}
	--v->len;
}
static void vec_init_empty(struct vec *v, size_t itemsize)
{
	*v = (struct vec){ .d = 0, .len = 0, .cap = 0, .itemsize = itemsize };
}
static void vec_finish(struct vec *v)
{
	free(v->d);
}

struct tp {
	int id; /* identifies the touchpoint */

	uint32_t start_t;
	float start[2];

	uint32_t last_t;
	float last[2];

	/* the touchpoint is only processed by the areas it starts in */
	struct vec areas; /* vec<struct libtouch_area *> */
};

struct libtouch_surface {
	struct vec tps; /* vec<struct tp> */
	struct vec areas; /* vec<struct libtouch_area *> */
};

struct libtouch_area {
	/* retain a reference to the surface this area belongs to */
	struct libtouch_surface *surf;

	/* the area, defined by an AABB */
	const float *aabb;

	/* number of touchpoints touching this area */
	int n;

	/* maximum number of touchpoints */
	int max_n;

	/* transformation & derived values */
	struct libtouch_rt F_accum, F_curr;
	bool F_dirty;

	/* velocity calculation logic */
	struct {
		bool started;
		uint32_t last_t;
		float last_p[2];
		float val[2];
	} V;

	/* options */
	struct libtouch_area_opts opts;
};

struct libtouch_surface *libtouch_surface_create()
{
	struct libtouch_surface *surf =
		calloc(1, sizeof(struct libtouch_surface));
	vec_init_empty(&surf->tps, sizeof(struct tp));
	vec_init_empty(&surf->areas, sizeof(struct libtouch_area *));
	return surf;
}
struct libtouch_area *libtouch_surface_add_area(struct libtouch_surface *surf,
		const float *aabb, struct libtouch_area_opts opts)
{
	struct libtouch_area *area = calloc(1, sizeof(struct libtouch_area));
	*area = (struct libtouch_area){
		.surf = surf,
		.aabb = aabb,
		.F_accum = libtouch_rt_identity,
		.F_curr = libtouch_rt_identity,
		.opts = opts,
	};

	vec_append(&surf->areas, &area);

	return area;
}
void libtouch_surface_remove_area(struct libtouch_surface *surf,
		struct libtouch_area *area) {
	for (size_t i = 0; i < area->surf->tps.len; ++i) {
		struct tp *tp = vec_get(&area->surf->tps, i);
		for (int k = 0; k < tp->areas.len; ++k) {
			struct libtouch_area **a = vec_get(&tp->areas, k);
			if (*a == area) {
				vec_remove(&tp->areas, k);
				break;
			}
		}
	}
	for (int i = 0; i < surf->areas.len; ++i) {
		struct libtouch_area **a = vec_get(&surf->areas, i);
		if (*a == area) {
			vec_remove(&surf->areas, i);
			free(area);
			break;
		}
	}
}
void libtouch_surface_destroy(struct libtouch_surface *surf)
{
	for (int i = 0; i < surf->areas.len; ++i) {
		struct libtouch_area **area = vec_get(&surf->areas, i);
		free(*area);
	}
	vec_finish(&surf->tps);
	vec_finish(&surf->areas);
	free(surf);
}

struct area_point_iter_cl {
	struct libtouch_area *area;
	size_t i;
};
static bool area_point_iter(void *_cl, struct point_pair *pair)
{
	struct area_point_iter_cl *cl = _cl;
	for (; cl->i < cl->area->surf->tps.len; ++cl->i) {
		const struct tp *tp = vec_get(&cl->area->surf->tps, cl->i);
		for (int k = 0; k < tp->areas.len; ++k) {
			struct libtouch_area * const *a =
				vec_get_c(&tp->areas, k);
			if (*a == cl->area) goto ok;
		}
		continue;

ok:
		pair->a[0] = tp->start[0];
		pair->a[1] = tp->start[1];
		pair->b[0] = tp->last[0];
		pair->b[1] = tp->last[1];

		++cl->i;
		return true;
	}
	return false;
}


struct libtouch_rt libtouch_area_get_transform(struct libtouch_area *area)
{
	if (area->F_dirty) {
		struct area_point_iter_cl cl = {
			.area = area,
			.i = 0
		};
		struct libtouch_rt F =
			est_fn[area->opts.g](&cl, &area_point_iter);
		area->F_curr = libtouch_rt_mul(&F, &area->F_accum);
		area->F_dirty = false;
	}

	return area->F_curr;
}

static void area_accumulate(struct libtouch_area *area)
{
	area->F_accum = libtouch_area_get_transform(area);
	area->F_dirty = true;
	for (size_t i = 0; i < area->surf->tps.len; ++i) {
		struct tp *tp = vec_get(&area->surf->tps, i);
		for (int k = 0; k < tp->areas.len; ++k) {
			struct libtouch_area * const *a =
				vec_get_c(&tp->areas, k);
			if (*a == area) goto ok;
		}
		continue;

ok:
		memcpy(tp->start, tp->last, sizeof(float) * 2);
	}
}

static bool area_down(struct libtouch_area *area, const struct tp *tp,
	uint32_t time)
{
	if (aabb_contains(area->aabb, tp->start)) {
		area_accumulate(area);

		if (area->n == 0) {
			// first touchpoint, send start event
			if (area->opts.start) {
				struct libtouch_gesture_data data = {
					.t = time,
					.rt = area->F_accum,
					.V[0] = 0, .V[1] = 0,
				};
				area->opts.start(area->opts.env, data);
			}

			if (area->opts.flags & LIBTOUCH_V) {
				area->V.started = false;
				area->V.last_t = time;
				area->V.last_p[0] = area->F_accum.t[0];
				area->V.last_p[1] = area->F_accum.t[1];
				area->V.val[0] = area->V.val[1] = 0;
			}
		}
		++area->n;
		if (area->n > area->max_n) area->max_n = area->n;

		return true;
	}
	return false;
}

static void area_up(struct libtouch_area *area, const struct tp *tp,
	uint32_t time)
{
	area_accumulate(area);
	if (--area->n == 0) {
		// last touchpoint left, we can dispatch gesture events
		if (area->opts.end) {
			struct libtouch_gesture_data data = {
				.t = time,
				.rt = area->F_accum,
				.V[0] = area->V.val[0], .V[1] = area->V.val[1],
			};
			area->opts.end(area->opts.env, data);
			area->F_accum = libtouch_rt_identity;
		}
	}
}

static void area_motion(struct libtouch_area *area, const struct tp *tp,
		uint32_t time, const float pos[static 2])
{
	area->F_dirty = true;

	if (area->opts.flags & LIBTOUCH_V && time > area->V.last_t) {
		struct libtouch_rt rt = libtouch_area_get_transform(area);
		uint32_t dt = time - area->V.last_t;
		float val_new[2] = {
			(rt.t[0] - area->V.last_p[0]) / dt,
			(rt.t[1] - area->V.last_p[1]) / dt,
		};
		if (!area->V.started) {
			area->V.val[0] = val_new[0];
			area->V.val[1] = val_new[1];
		} else {
			/* for low pass filter idea, see:
			 * https://mortoray.com/2015/04/08/measuring-finger-mouse-velocity-at-release-time */
			static const float tau = 10; // TODO: make tau configurable
			float alpha = 1 - expf(-(dt/tau));
			area->V.val[0] += alpha * (val_new[0] - area->V.val[0]);
			area->V.val[1] += alpha * (val_new[1] - area->V.val[1]);
		}

		area->V.started = true;
		area->V.last_t = time;
		area->V.last_p[0] = rt.t[0];
		area->V.last_p[1] = rt.t[1];
	}

	if (area->opts.move) {
		struct libtouch_gesture_data data = {
			.t = time,
			.rt = libtouch_area_get_transform(area),
			.V[0] = area->V.val[0], .V[1] = area->V.val[1],
		};
		area->opts.move(area->opts.env, data);
	}
}

void libtouch_surface_down(struct libtouch_surface *surf, uint32_t time,
		int id, const float pos[2])
{
	struct tp tp = {
		.id = id,
		.start_t = time,
		.start = { pos[0], pos[1] },
		.last_t = time,
		.last = { pos[0], pos[1] },
	};
	vec_init_empty(&tp.areas, sizeof(struct libtouch_area *));

	/* find the area that contains this point */
	for (size_t i = 0; i < surf->areas.len; ++i) {
		struct libtouch_area **area = vec_get(&surf->areas, i);
		if (area_down(*area, &tp, time)) {
			vec_append(&tp.areas, area);
		}
	}

	/* store the touchpoint */
	vec_append(&surf->tps, &tp);
}

static size_t find_tp(struct libtouch_surface *surf, int id)
{
	for (size_t i = 0; i < surf->tps.len; ++i) {
		const struct tp *tp = vec_get(&surf->tps, i);
		if (tp->id == id) {
			return i;
		}
	}
	assert(false);
}

void libtouch_surface_up(struct libtouch_surface *surf, uint32_t time, int id)
{
	size_t tp_i = find_tp(surf, id);
	struct tp *tp = vec_get(&surf->tps, tp_i);
	for (int i = 0; i < tp->areas.len; ++i) {
		struct libtouch_area *area =
			*(struct libtouch_area **)vec_get(&tp->areas, i);
		area_up(area, tp, time);
	}

	/* remove this tp from the list */
	vec_finish(&tp->areas);
	vec_remove(&surf->tps, tp_i);
}
void libtouch_surface_motion(struct libtouch_surface *surf, uint32_t time,
		int id, const float pos[2])
{
	size_t tp_i = find_tp(surf, id);
	struct tp *tp = vec_get(&surf->tps, tp_i);

	for (int i = 0; i < tp->areas.len; ++i) {
		struct libtouch_area *area =
			*(struct libtouch_area **)vec_get(&tp->areas, i);
		area_motion(area, tp, time, pos);
	}
	tp->last_t = time;
	memcpy(tp->last, pos, sizeof(float) * 2);
}
