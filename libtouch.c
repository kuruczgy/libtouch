#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "libtouch.h"

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

bool aabb_contains(const struct aabb *aabb, const float p[static 2])
{
	return p[0] >= aabb->x
		&& p[0] < aabb->x + aabb->w
		&& p[1] >= aabb->y
		&& p[1] < aabb->y + aabb->h;
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
void vec_append(struct vec *v, const void *n)
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
void *vec_get(struct vec *v, size_t i)
{
	assert(i >= 0 && i < v->len);
	return v->d + i * v->itemsize;
}
void vec_remove(struct vec *v, size_t i)
{
	if (v->len > i + 1) {
		memmove(v->d + i * v->itemsize, v->d + (i + 1) * v->itemsize,
			v->itemsize * (v->len - i - 1));
	}
	--v->len;
}
void vec_init_empty(struct vec *v, size_t itemsize)
{
	*v = (struct vec){ .d = 0, .len = 0, .cap = 0, .itemsize = itemsize };
}
void vec_finish(struct vec *v)
{
	free(v->d);
}

struct tp {
	int id; /* identifies the touchpoint */

	uint32_t start_t;
	float start[2];
	
	uint32_t last_t;
	float last[2];

	/* the touchpoint is only processed by the area it starts in */
	struct libtouch_area *area;
};

struct libtouch_surface {
	struct vec tps; /* vec<struct tp> */
	struct vec areas; /* vec<struct libtouch_area *> */
};

struct libtouch_area {
	/* retain a reference to the surface this area belongs to */
	struct libtouch_surface *surf;

	/* the area, defined by an AABB */
	struct aabb aabb;

	/* number of touchpoints touching this area */
	int n;

	/* maximum number of touchpoints */
	int max_n;

	struct libtouch_rt F_accum, F_curr;
	bool F_dirty;
};

struct libtouch_surface *libtouch_surface_create()
{
	struct libtouch_surface *surf =
		calloc(1, sizeof(struct libtouch_surface));
	vec_init_empty(&surf->tps, sizeof(struct tp));
	vec_init_empty(&surf->areas, sizeof(struct libtouch_area *));
	return surf;
}
struct libtouch_area *libtouch_surface_add_area(
		struct libtouch_surface *surf, struct aabb aabb)
{
	struct libtouch_area *area = calloc(1, sizeof(struct libtouch_area));
	area->surf = surf;
	area->aabb = aabb;
	area->F_accum = area->F_curr = libtouch_rt_identity;

	vec_append(&surf->areas, &area);

	return area;
}
void libtouch_surface_destroy(struct libtouch_surface *surf)
{
	for (int i = 0; i < surf->areas.len; ++i) {
		struct libtouch_area **area = vec_get(&surf->areas, i);
		free(*area);
	}
	vec_finish(&surf->tps);
	vec_finish(&surf->areas);
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
		if (tp->area != cl->area) continue;

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
		struct libtouch_rt F = estimate_translation_scaling_rotation(
			&cl, &area_point_iter);
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
		if (tp->area != area) continue;

		memcpy(tp->start, tp->last, sizeof(float) * 2);
	}
}

static bool area_down(struct libtouch_area *area, const struct tp *tp)
{
	if (aabb_contains(&area->aabb, tp->start)) {
		area_accumulate(area);

		++area->n;
		if (area->n > area->max_n) area->max_n = area->n;

		return true;
	}
	return false;
}

static void area_up(struct libtouch_area *area, const struct tp *tp)
{
	area_accumulate(area);
	if (--area->n == 0) {
		// last touchpoint left, we can dispatch gesture events
	}
}

static void area_motion(struct libtouch_area *area, const struct tp *tp,
		uint32_t time, const float pos[static 2])
{
	area->F_dirty = true;
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
		.area = NULL
	};

	/* find the area that contains this point */
	for (size_t i = 0; i < surf->areas.len; ++i) {
		struct libtouch_area **area = vec_get(&surf->areas, i);
		if (area_down(*area, &tp)) {
			tp.area = *area;
			break;
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
	if (tp->area) {
		area_up(tp->area, tp);
	}

	/* remove this tp from the list */
	vec_remove(&surf->tps, tp_i);
}
void libtouch_surface_motion(struct libtouch_surface *surf, uint32_t time,
		int id, const float pos[2])
{
	size_t tp_i = find_tp(surf, id);
	struct tp *tp = vec_get(&surf->tps, tp_i);

	if (tp->area) {
		area_motion(tp->area, tp, time, pos);
	}
	tp->last_t = time;
	memcpy(tp->last, pos, sizeof(float) * 2);
}