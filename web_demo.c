#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define B3W_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define B3W_EXPORT
#endif

#define B3W_MAX_BODIES 4096
#define B3W_FLOATS_PER_BODY 8
#define B3W_STATS_FLOATS 8

static b3WorldId g_world = {0};
static b3BodyId g_bodies[B3W_MAX_BODIES];
static uint8_t g_body_types[B3W_MAX_BODIES];
static int g_body_count = 0;
static float g_transforms[B3W_MAX_BODIES * B3W_FLOATS_PER_BODY];
static float g_stats[B3W_STATS_FLOATS];
static uint32_t g_rng = 1;
static int g_shots = 0;
static int g_body_target = 512;
static float g_player_position[4] = {0.0f, 2.4f, 34.0f, 0.0f};
static b3Pos g_mover_origin = {0.0f, 1.25f, 34.0f};
static b3Vec3 g_mover_velocity = {0.0f, 0.0f, 0.0f};
static b3Vec3 g_mover_wish_velocity = {0.0f, 0.0f, 0.0f};
static b3Capsule g_mover_capsule = {{0.0f, -0.55f, 0.0f}, {0.0f, 0.55f, 0.0f}, 0.34f};
static int g_mover_on_ground = 0;

typedef struct B3WMoverContext {
    b3CollisionPlane planes[8];
    b3ShapeId shape_ids[8];
    b3Vec3 points[8];
    int plane_count;
    int on_ground;
} B3WMoverContext;

static uint32_t b3w_rand_u32(void) {
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng;
}

static float b3w_rand01(void) {
    return (float)(b3w_rand_u32() >> 8) * (1.0f / 16777216.0f);
}

static float b3w_rand_range(float lo, float hi) {
    return lo + (hi - lo) * b3w_rand01();
}

static void b3w_destroy_world(void) {
    if (b3World_IsValid(g_world)) {
        b3DestroyWorld(g_world);
    }

    memset(&g_world, 0, sizeof(g_world));
    g_body_count = 0;
}

static b3BodyId b3w_create_body(b3BodyType type, b3Vec3 position, b3Quat rotation, uint8_t draw_type) {
    if (g_body_count >= B3W_MAX_BODIES) {
        return (b3BodyId){0};
    }

    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = type;
    body_def.position = position;
    body_def.rotation = rotation;
    body_def.enableSleep = true;

    b3BodyId body = b3CreateBody(g_world, &body_def);
    g_bodies[g_body_count] = body;
    g_body_types[g_body_count] = draw_type;
    g_body_count++;
    return body;
}

static b3BodyId b3w_create_dynamic_body(b3Vec3 position, b3Quat rotation, b3Vec3 velocity, uint8_t draw_type) {
    if (g_body_count >= B3W_MAX_BODIES) {
        return (b3BodyId){0};
    }

    b3BodyDef body_def = b3DefaultBodyDef();
    body_def.type = b3_dynamicBody;
    body_def.position = position;
    body_def.rotation = rotation;
    body_def.linearVelocity = velocity;
    body_def.angularVelocity = (b3Vec3){b3w_rand_range(-1.5f, 1.5f), b3w_rand_range(-1.5f, 1.5f), b3w_rand_range(-1.5f, 1.5f)};
    body_def.enableSleep = draw_type == 6 ? false : true;
    body_def.isBullet = draw_type == 6;

    b3BodyId body = b3CreateBody(g_world, &body_def);
    g_bodies[g_body_count] = body;
    g_body_types[g_body_count] = draw_type;
    g_body_count++;
    return body;
}

static void b3w_add_box(b3BodyId body, float hx, float hy, float hz, float friction, float restitution) {
    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.density = 1.0f;
    shape_def.baseMaterial.friction = friction;
    shape_def.baseMaterial.restitution = restitution;
    b3BoxHull hull = b3MakeBoxHull(hx, hy, hz);
    b3CreateHullShape(body, &shape_def, &hull.base);
}

static void b3w_add_sphere_ex(
    b3BodyId body,
    float radius,
    float friction,
    float restitution,
    float density,
    float rolling_resistance
) {
    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.density = density;
    shape_def.baseMaterial.friction = friction;
    shape_def.baseMaterial.restitution = restitution;
    shape_def.baseMaterial.rollingResistance = rolling_resistance;
    b3Sphere sphere = {b3Vec3_zero, radius};
    b3CreateSphereShape(body, &shape_def, &sphere);
}

static void b3w_add_sphere(b3BodyId body, float radius, float friction, float restitution) {
    b3w_add_sphere_ex(body, radius, friction, restitution, 1.0f, 0.03f);
}

static void b3w_add_capsule(b3BodyId body, float half_height, float radius, float friction, float restitution) {
    b3ShapeDef shape_def = b3DefaultShapeDef();
    shape_def.density = 0.9f;
    shape_def.baseMaterial.friction = friction;
    shape_def.baseMaterial.restitution = restitution;
    shape_def.baseMaterial.rollingResistance = 0.04f;
    b3Capsule capsule = {{0.0f, -half_height, 0.0f}, {0.0f, half_height, 0.0f}, radius};
    b3CreateCapsuleShape(body, &shape_def, &capsule);
}

static bool b3w_mover_filter(b3ShapeId shape_id, void* context) {
    (void)shape_id;
    (void)context;
    return true;
}

static bool b3w_plane_result(b3ShapeId shape_id, const b3PlaneResult* plane_results, int plane_count, void* context) {
    (void)shape_id;
    B3WMoverContext* ctx = (B3WMoverContext*)context;

    for (int i = 0; i < plane_count && ctx->plane_count < 8; i++) {
        b3Plane plane = plane_results[i].plane;
        ctx->planes[ctx->plane_count] = (b3CollisionPlane){
            .plane = plane,
            .pushLimit = FLT_MAX,
            .push = 0.0f,
            .clipVelocity = true,
        };
        ctx->shape_ids[ctx->plane_count] = shape_id;
        ctx->points[ctx->plane_count] = plane_results[i].point;
        if (plane.normal.y > 0.45f) {
            ctx->on_ground = 1;
        }
        ctx->plane_count++;
    }

    return true;
}

static void b3w_reset_mover(void) {
    g_mover_origin = (b3Pos){0.0f, 1.25f, 34.0f};
    g_mover_velocity = (b3Vec3){0.0f, 0.0f, 0.0f};
    g_mover_wish_velocity = (b3Vec3){0.0f, 0.0f, 0.0f};
    g_mover_on_ground = 0;
    g_player_position[0] = (float)g_mover_origin.x;
    g_player_position[1] = (float)g_mover_origin.y + 0.9f;
    g_player_position[2] = (float)g_mover_origin.z;
    g_player_position[3] = 1.0f;
}

static int b3w_probe_ground(void) {
    b3QueryFilter filter = b3DefaultQueryFilter();
    b3Pos ray_origin = b3OffsetPos(g_mover_origin, g_mover_capsule.center1);
    b3RayResult ray = b3World_CastRayClosest(g_world, ray_origin, (b3Vec3){0.0f, -0.24f, 0.0f}, filter);
    return ray.hit && ray.normal.y > 0.45f;
}

static void b3w_push_dynamic_contacts(const B3WMoverContext* ctx) {
    for (int i = 0; i < ctx->plane_count; i++) {
        b3BodyId body_id = b3Shape_GetBody(ctx->shape_ids[i]);
        if (!b3Body_IsValid(body_id) || b3Body_GetType(body_id) != b3_dynamicBody) {
            continue;
        }

        b3Vec3 normal = {
            -ctx->planes[i].plane.normal.x,
            -ctx->planes[i].plane.normal.y,
            -ctx->planes[i].plane.normal.z,
        };

        float inv_mass = b3Body_GetInverseMass(body_id);
        b3Matrix3 inv_inertia = b3Body_GetWorldInverseRotationalInertia(body_id);
        b3Pos center = b3Body_GetWorldCenterOfMass(body_id);
        b3Vec3 r = b3SubPos(ctx->points[i], center);
        b3Vec3 rn = b3Cross(r, normal);
        float k = inv_mass + b3Dot(rn, b3MulMV(inv_inertia, rn));
        float normal_mass = k > 0.0f ? 1.0f / k : 0.0f;

        b3Vec3 body_velocity = b3Body_GetLinearVelocity(body_id);
        b3Vec3 body_omega = b3Body_GetAngularVelocity(body_id);
        b3Vec3 contact_velocity = b3Add(body_velocity, b3Cross(body_omega, r));
        float vn = b3Dot(b3Sub(contact_velocity, g_mover_velocity), normal);
        float impulse = fmaxf(-normal_mass * vn, 0.0f);
        if (impulse <= 0.0f) {
            continue;
        }

        b3Body_ApplyLinearImpulse(body_id, b3MulSV(impulse, normal), ctx->points[i], true);
    }
}

static void b3w_update_mover(float dt) {
    const float max_speed = 10.0f;
    const float accel = 35.0f;
    const float friction = 7.0f;
    const float gravity = 24.0f;
    const float min_speed = 0.01f;

    g_mover_on_ground = g_mover_on_ground || b3w_probe_ground();

    b3Vec3 wish = g_mover_wish_velocity;
    wish.y = 0.0f;
    float wish_len2 = b3LengthSquared(wish);
    if (wish_len2 > max_speed * max_speed) {
        wish = b3MulSV(max_speed / sqrtf(wish_len2), wish);
    }

    b3Vec3 horizontal = {g_mover_velocity.x, 0.0f, g_mover_velocity.z};
    float speed = sqrtf(b3LengthSquared(horizontal));
    if (speed > min_speed) {
        float drop = speed * friction * dt;
        float new_speed = fmaxf(0.0f, speed - drop);
        float scale = new_speed / speed;
        g_mover_velocity.x *= scale;
        g_mover_velocity.z *= scale;
    } else {
        g_mover_velocity.x = 0.0f;
        g_mover_velocity.z = 0.0f;
    }

    b3Vec3 current_horizontal = {g_mover_velocity.x, 0.0f, g_mover_velocity.z};
    b3Vec3 delta_v = b3Sub(wish, current_horizontal);
    float delta_len2 = b3LengthSquared(delta_v);
    if (delta_len2 > 0.000001f) {
        float delta_len = sqrtf(delta_len2);
        float add = fminf(delta_len, accel * dt);
        b3Vec3 add_v = b3MulSV(add / delta_len, delta_v);
        g_mover_velocity = b3Add(g_mover_velocity, add_v);
    }

    if (g_mover_on_ground && g_mover_velocity.y < 0.0f) {
        g_mover_velocity.y = 0.0f;
    }
    g_mover_velocity.y -= gravity * dt;

    B3WMoverContext last_ctx = {0};
    b3QueryFilter filter = b3DefaultQueryFilter();
    b3Pos target = b3OffsetPos(g_mover_origin, b3MulSV(dt, g_mover_velocity));

    for (int iter = 0; iter < 6; iter++) {
        b3Vec3 target_delta = b3SubPos(target, g_mover_origin);
        if (b3LengthSquared(target_delta) < 0.000001f) {
            break;
        }

        float fraction = b3World_CastMover(g_world, g_mover_origin, &g_mover_capsule, target_delta, filter, b3w_mover_filter, NULL);
        b3Vec3 safe_delta = b3MulSV(fraction, target_delta);
        g_mover_origin = b3OffsetPos(g_mover_origin, safe_delta);

        B3WMoverContext ctx = {0};
        b3World_CollideMover(g_world, g_mover_origin, &g_mover_capsule, filter, b3w_plane_result, &ctx);

        b3PlaneSolverResult result = b3SolvePlanes(b3Vec3_zero, ctx.planes, ctx.plane_count);
        b3Vec3 delta = result.delta;
        g_mover_origin = b3OffsetPos(g_mover_origin, delta);
        last_ctx = ctx;

        if (fraction >= 0.999f && b3LengthSquared(delta) < 0.0001f) {
            break;
        }
    }

    b3w_push_dynamic_contacts(&last_ctx);
    g_mover_velocity = b3ClipVector(g_mover_velocity, last_ctx.planes, last_ctx.plane_count);
    g_mover_on_ground = last_ctx.on_ground || b3w_probe_ground();

    g_player_position[0] = (float)g_mover_origin.x;
    g_player_position[1] = (float)g_mover_origin.y + 0.9f;
    g_player_position[2] = (float)g_mover_origin.z;
    g_player_position[3] = (float)g_mover_on_ground;
}

B3W_EXPORT void b3demo_init(int body_target) {
    if (body_target < 64) body_target = 64;
    if (body_target > 3200) body_target = 3200;

    b3w_destroy_world();
    g_rng = 1;
    g_shots = 0;
    g_body_target = body_target;

    b3WorldDef world_def = b3DefaultWorldDef();
    world_def.gravity = (b3Vec3){0.0f, -18.0f, 0.0f};
    world_def.enableContinuous = true;
    world_def.enableSleep = true;
    world_def.workerCount = 1;
    world_def.capacity.staticBodyCount = 32;
    world_def.capacity.dynamicBodyCount = body_target + 768;
    world_def.capacity.staticShapeCount = 64;
    world_def.capacity.dynamicShapeCount = body_target + 768;
    world_def.capacity.contactCount = (body_target + 768) * 16;
    g_world = b3CreateWorld(&world_def);

    b3BodyId floor = b3w_create_body(b3_staticBody, (b3Vec3){0.0f, -0.75f, 0.0f}, b3Quat_identity, 3);
    b3w_add_box(floor, 58.0f, 0.5f, 58.0f, 0.92f, 0.04f);

    b3w_reset_mover();

    b3Quat ramp_rot = b3MakeQuatFromAxisAngle((b3Vec3){1.0f, 0.0f, 0.0f}, 0.18f);
    b3BodyId ramp = b3w_create_body(b3_staticBody, (b3Vec3){26.0f, 0.45f, 19.0f}, ramp_rot, 4);
    b3w_add_box(ramp, 18.0f, 0.35f, 6.0f, 0.7f, 0.08f);

    for (int i = 0; i < 5; i++) {
        float a = (float)i * 1.25663706f;
        b3Vec3 p = {cosf(a) * 18.0f, 0.35f, sinf(a) * 18.0f};
        b3BodyId bumper = b3w_create_body(b3_staticBody, p, b3Quat_identity, 5);
        b3w_add_sphere(bumper, 0.9f, 0.1f, 1.15f);
    }

    int made = 0;

    const float hx = 0.48f;
    const float hy = 0.32f;
    const float hz = 0.48f;
    const float sx = 1.02f;
    const float sy = 0.66f;
    const float sz = 1.02f;

    for (int layer = 0; layer < 36 && made < body_target; layer++) {
        int width = 28 - layer / 3;
        if (width < 8) width = 8;
        float y = 0.05f + hy + (float)layer * sy;

        for (int side = -1; side <= 1 && made < body_target; side += 2) {
            float z = -2.0f + side * 5.2f;
            for (int x = 0; x < width && made < body_target; x++) {
                float px = ((float)x - 0.5f * (float)(width - 1)) * sx + ((layer & 1) ? 0.5f * sx : 0.0f);
                b3Vec3 p = {px, y, z};
                b3BodyId body = b3w_create_body(b3_dynamicBody, p, b3Quat_identity, 1);
                b3w_add_box(body, hx, hy, hz, 0.72f, 0.06f);
                made++;
            }
        }

        int tower = 7;
        for (int side = -1; side <= 1 && made < body_target; side += 2) {
            for (int x = -tower; x <= tower && made < body_target; x += tower * 2) {
                for (int z = -1; z <= 1 && made < body_target; z++) {
                    b3Vec3 p = {(float)x * sx, y, -2.0f + side * (7.0f + (float)z * sz)};
                    b3BodyId body = b3w_create_body(b3_dynamicBody, p, b3Quat_identity, 1);
                    b3w_add_box(body, hx, hy, hz, 0.75f, 0.05f);
                    made++;
                }
            }
        }
    }

    while (made < body_target) {
        float px = b3w_rand_range(-15.0f, 15.0f);
        float py = 1.0f + b3w_rand_range(0.0f, 12.0f);
        float pz = b3w_rand_range(-10.0f, 3.0f);
        b3BodyId body = b3w_create_body(b3_dynamicBody, (b3Vec3){px, py, pz}, b3Quat_identity, 1);
        b3w_add_box(body, hx, hy, hz, 0.72f, 0.06f);
        made++;
    }
}

B3W_EXPORT int b3demo_fire_cannonball(float x, float y, float z, float vx, float vy, float vz, float radius) {
    if (!b3World_IsValid(g_world)) {
        b3demo_init(512);
    }

    if (radius < 0.18f) radius = 0.18f;
    if (radius > 1.2f) radius = 1.2f;

    b3BodyId ball = b3w_create_dynamic_body((b3Vec3){x, y, z}, b3Quat_identity, (b3Vec3){vx, vy, vz}, 6);
    if (!b3Body_IsValid(ball)) {
        return 0;
    }

    b3w_add_sphere_ex(ball, radius, 0.18f, 0.04f, 14.0f, 0.005f);
    g_shots++;
    return 1;
}

B3W_EXPORT void b3demo_barrage(int count) {
    if (count < 1) count = 1;
    if (count > 32) count = 32;

    for (int i = 0; i < count; i++) {
        float lane = ((float)(i % 9) - 4.0f) * 1.6f + b3w_rand_range(-0.35f, 0.35f);
        float y = 2.8f + b3w_rand_range(0.0f, 4.0f);
        float z = 30.0f + b3w_rand_range(0.0f, 4.0f);
        float vx = b3w_rand_range(-1.2f, 1.2f);
        float vy = b3w_rand_range(4.0f, 8.0f);
        float vz = -42.0f + b3w_rand_range(-8.0f, 2.0f);
        if (!b3demo_fire_cannonball(lane, y, z, vx, vy, vz, 0.75f)) {
            break;
        }
    }
}

B3W_EXPORT int b3demo_shot_count(void) {
    return g_shots;
}

B3W_EXPORT void b3demo_set_player_velocity(float vx, float vz) {
    g_mover_wish_velocity = (b3Vec3){vx, 0.0f, vz};
}

B3W_EXPORT void b3demo_jump(void) {
    if (g_mover_on_ground) {
        g_mover_velocity.y = 8.5f;
        g_mover_on_ground = 0;
    }
}

B3W_EXPORT float* b3demo_player_position_ptr(void) {
    return g_player_position;
}

static void b3w_destroy_body_at(int i) {
    if (i < 0 || i >= g_body_count) {
        return;
    }

    if (b3Body_IsValid(g_bodies[i])) {
        b3DestroyBody(g_bodies[i]);
    }

    int last = g_body_count - 1;
    if (i != last) {
        g_bodies[i] = g_bodies[last];
        g_body_types[i] = g_body_types[last];
    }

    g_body_count--;
}

static void b3w_prune_cannonballs(void) {
    for (int i = 0; i < g_body_count;) {
        if (g_body_types[i] != 6) {
            i++;
            continue;
        }

        b3Pos p = b3Body_GetPosition(g_bodies[i]);
        if (
            p.y < -8.0f ||
            p.z < -70.0f ||
            p.z > 80.0f ||
            p.x < -80.0f ||
            p.x > 80.0f ||
            g_body_count > g_body_target + 520
        ) {
            b3w_destroy_body_at(i);
            continue;
        }

        i++;
    }
}

B3W_EXPORT void b3demo_reset(void) {
    b3demo_init(g_body_target);
}

B3W_EXPORT void b3demo_step(int steps) {
    if (!b3World_IsValid(g_world)) {
        b3demo_init(512);
    }

    if (steps < 1) steps = 1;
    if (steps > 16) steps = 16;

    for (int i = 0; i < steps; i++) {
        b3w_update_mover(1.0f / 60.0f);
        b3World_Step(g_world, 1.0f / 60.0f, 8);
    }

    b3w_prune_cannonballs();

    b3Counters counters = b3World_GetCounters(g_world);
    b3Profile profile = b3World_GetProfile(g_world);
    g_stats[0] = (float)counters.bodyCount;
    g_stats[1] = (float)counters.contactCount;
    g_stats[2] = profile.step;
    g_stats[3] = profile.solve;
    g_stats[4] = profile.pairs;
    g_stats[5] = profile.collide;
    g_stats[6] = profile.solverSetup;
    g_stats[7] = profile.integrateVelocities;

    for (int i = 0; i < g_body_count; i++) {
        b3WorldTransform xf = b3Body_GetTransform(g_bodies[i]);
        int o = i * B3W_FLOATS_PER_BODY;
        g_transforms[o + 0] = (float)g_body_types[i];
        g_transforms[o + 1] = (float)xf.p.x;
        g_transforms[o + 2] = (float)xf.p.y;
        g_transforms[o + 3] = (float)xf.p.z;
        g_transforms[o + 4] = xf.q.v.x;
        g_transforms[o + 5] = xf.q.v.y;
        g_transforms[o + 6] = xf.q.v.z;
        g_transforms[o + 7] = xf.q.s;
    }

    if (g_body_count < B3W_MAX_BODIES) {
        int o = g_body_count * B3W_FLOATS_PER_BODY;
        g_transforms[o + 0] = 7.0f;
        g_transforms[o + 1] = (float)g_mover_origin.x;
        g_transforms[o + 2] = (float)g_mover_origin.y;
        g_transforms[o + 3] = (float)g_mover_origin.z;
        g_transforms[o + 4] = 0.0f;
        g_transforms[o + 5] = 0.0f;
        g_transforms[o + 6] = 0.0f;
        g_transforms[o + 7] = 1.0f;
    }
}

B3W_EXPORT int b3demo_body_count(void) {
    return g_body_count < B3W_MAX_BODIES ? g_body_count + 1 : g_body_count;
}

B3W_EXPORT int b3demo_floats_per_body(void) {
    return B3W_FLOATS_PER_BODY;
}

B3W_EXPORT float* b3demo_transforms_ptr(void) {
    return g_transforms;
}

B3W_EXPORT float* b3demo_stats_ptr(void) {
    return g_stats;
}

B3W_EXPORT uint32_t b3demo_hash(void) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < g_body_count * B3W_FLOATS_PER_BODY; i++) {
        union {
            float f;
            uint32_t u;
        } v;
        v.f = g_transforms[i];
        h ^= v.u;
        h *= 16777619u;
    }
    return h;
}
