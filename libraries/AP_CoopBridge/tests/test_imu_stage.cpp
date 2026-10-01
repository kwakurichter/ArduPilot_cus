/*
  Unit tests for AP_CoopBridge_ImuStage, the FC first stage of the two-stage IMU
  preintegration (COOP_IMU_DELTA).

  The reference is the same recurrence in double precision with an exact SO(3) exponential,
  integrated sample by sample with rates held constant within each tick (the zero-order hold
  of the thesis repo's scripts/coop/frontends/two_stage.py FirstStage). The float32 stage must
  agree to float precision, put its boundaries on the absolute 10 ms grid, and split ticks
  that straddle a boundary in proportion to time.
 */
#include <AP_gtest.h>

#include <AP_CoopBridge/AP_CoopBridge_ImuStage.h>

#if AP_COOPBRIDGE_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

using Sub = AP_CoopBridge_ImuStage::Sub;
constexpr uint32_t SUB = AP_CoopBridge_ImuStage::SUB_US;

// Exact rotation matrix of a rotation vector (Rodrigues), double precision.
Matrix3d exp_so3(const Vector3d &phi)
{
    const double angle = phi.length();
    Matrix3d r;
    r.identity();
    if (angle < 1e-12) {
        return r;
    }
    const Vector3d k = phi / angle;
    Matrix3d kx(Vector3d(0, -k.z, k.y), Vector3d(k.z, 0, -k.x), Vector3d(-k.y, k.x, 0));
    return r + kx * sin(angle) + (kx * kx) * (1 - cos(angle));
}

// Rotation vector of a rotation matrix (angle below pi), double precision.
Vector3d log_so3(const Matrix3d &r)
{
    const double c = constrain_float((r.a.x + r.b.y + r.c.z - 1) * 0.5, -1.0, 1.0);
    const double angle = acos(c);
    const Vector3d w(r.c.y - r.b.z, r.a.z - r.c.x, r.b.x - r.a.y);
    if (angle < 1e-12) {
        return w * 0.5;
    }
    return w * (angle / (2 * sin(angle)));
}

// Double-precision reference for one sub-interval: pieces of (dtheta, dv, dt).
struct Reference {
    Matrix3d rot;
    Vector3d vel, pos;
    Reference() { rot.identity(); }
    void add(const Vector3d &dtheta, const Vector3d &dv, double dt) {
        const Vector3d dv_start = rot * dv;
        pos += vel * dt + dv_start * (0.5 * dt);
        vel += dv_start;
        rot = rot * exp_so3(dtheta);
    }
};

// Rates of a smooth test motion (body frame), constant within a tick by construction.
Vector3d omega_at(double t) { return Vector3d(0.8 * sin(3 * t), -0.5 + 0.3 * cos(2 * t), 1.2 * sin(t)); }
Vector3d force_at(double t) { return Vector3d(0.5 * cos(5 * t), 0.2, 9.81 + 0.3 * sin(4 * t)); }

void expect_close(const Vector3f &got, const Vector3d &want, double tol, const char *what, int i)
{
    for (uint8_t k = 0; k < 3; k++) {
        EXPECT_NEAR(got[k], want[k], tol) << what << " component " << int(k) << " of sub " << i;
    }
}

}  // namespace

// Ticks of an irregular length (as a main loop with jitter): every completed sub-interval
// lies on the grid, follows the previous one without a gap, and matches the reference.
TEST(CoopImuStage, MatchesReferenceOnGrid)
{
    AP_CoopBridge_ImuStage stage;
    const uint64_t t_begin = 123456789ULL;   // not on the grid
    uint64_t t = t_begin;
    uint32_t tick_index = 0;

    // reference: integrate the same pieces, split at the grid, in double precision
    Reference ref;
    bool ref_running = false;
    uint64_t ref_start = 0;
    std::vector<std::pair<uint64_t, Reference>> expected;

    std::vector<Sub> got;
    while (t < t_begin + 2000000ULL) {   // 2 s
        const uint32_t tick_us = 2500 + (tick_index * 37) % 400;  // 2.5 ms +- jitter
        tick_index++;
        const uint64_t t_end = t + tick_us;
        const double mid = (t + t_end) * 0.5e-6;
        const double dt = tick_us * 1e-6;
        const Vector3d dtheta = omega_at(mid) * dt;
        const Vector3d dv = force_at(mid) * dt;

        // reference split
        uint64_t a = t;
        while (a < t_end) {
            if (!ref_running) {
                const uint64_t grid = ((a + SUB - 1) / SUB) * SUB;
                if (grid >= t_end) {
                    break;
                }
                a = grid;
                ref_start = grid;
                ref = Reference();
                ref_running = true;
            }
            const uint64_t b = MIN(t_end, ref_start + SUB);
            const double share = double(b - a) / tick_us;
            ref.add(dtheta * share, dv * share, (b - a) * 1e-6);
            a = b;
            if (b == ref_start + SUB) {
                expected.emplace_back(ref_start, ref);
                ref_start += SUB;
                ref = Reference();
            }
        }

        Sub out[4];
        const uint16_t n = stage.push(t, t_end, Vector3f(dtheta.x, dtheta.y, dtheta.z),
                                      Vector3f(dv.x, dv.y, dv.z), 2.0f * tick_us * 1e-3f, 1, out, 4);
        ASSERT_LE(n, 4);
        for (uint16_t i = 0; i < n; i++) {
            got.push_back(out[i]);
        }
        t = t_end;
    }

    ASSERT_EQ(got.size(), expected.size());
    ASSERT_GE(got.size(), 190U);
    for (size_t i = 0; i < got.size(); i++) {
        const Sub &s = got[i];
        EXPECT_EQ(s.t0_us % SUB, 0U);
        EXPECT_EQ(s.duration_us, SUB);
        EXPECT_EQ(s.t0_us, expected[i].first);
        if (i > 0) {
            EXPECT_EQ(s.t0_us, got[i - 1].t0_us + SUB);  // contiguous
        }
        const Reference &r = expected[i].second;
        expect_close(s.theta, log_so3(r.rot), 2e-6, "theta", i);
        expect_close(s.velocity, r.vel, 2e-6, "velocity", i);
        expect_close(s.position, r.pos, 2e-8, "position", i);
        // 2 samples per ms of tick: 20 per sub-interval
        EXPECT_EQ(s.samples, 20);
        EXPECT_EQ(s.flags, 1);
    }
}

// Splitting a tick in two keeps the rotation exactly (same axis) and changes velocity and
// position only at second order: the second part of the velocity is rotated by the first
// part's rotation, about |dtheta| |dv| / 4 here.
TEST(CoopImuStage, SplitSecondOrder)
{
    const Vector3f dtheta(0.01f, -0.02f, 0.015f);
    const Vector3f dv(0.03f, 0.01f, 0.0981f);
    Sub whole[2], parts[2];

    AP_CoopBridge_ImuStage a;
    a.push(SUB * 100 - 1000, SUB * 100, Vector3f(), Vector3f(), 0, 0, whole, 2);  // reach the grid
    ASSERT_EQ(a.push(SUB * 100, SUB * 101, dtheta, dv, 0, 0, whole, 2), 1);

    AP_CoopBridge_ImuStage b;
    b.push(SUB * 100 - 1000, SUB * 100, Vector3f(), Vector3f(), 0, 0, parts, 2);
    ASSERT_EQ(b.push(SUB * 100, SUB * 100 + 3000, dtheta * 0.3f, dv * 0.3f, 0, 0, parts, 2), 0);
    ASSERT_EQ(b.push(SUB * 100 + 3000, SUB * 101, dtheta * 0.7f, dv * 0.7f, 0, 0, parts, 2), 1);

    for (uint8_t k = 0; k < 3; k++) {
        EXPECT_NEAR(whole[0].theta[k], parts[0].theta[k], 1e-7f);
        EXPECT_NEAR(whole[0].velocity[k], parts[0].velocity[k], 1e-3f);
        EXPECT_NEAR(whole[0].position[k], parts[0].position[k], 1e-5f);
    }
}

// A tick straddling a boundary is shared out in proportion to time, flags included.
TEST(CoopImuStage, StraddlingTick)
{
    AP_CoopBridge_ImuStage stage;
    Sub out[2];
    // reach the grid at 10 ms, then one tick from 18 ms to 22 ms crossing the 20 ms boundary
    EXPECT_EQ(stage.push(9000, 10000, Vector3f(), Vector3f(), 0, 0, out, 2), 0);
    EXPECT_EQ(stage.push(10000, 18000, Vector3f(), Vector3f(0.8f, 0, 0), 8, 1, out, 2), 0);
    ASSERT_EQ(stage.push(18000, 22000, Vector3f(), Vector3f(0.4f, 0, 0), 4, 4, out, 2), 1);
    EXPECT_EQ(out[0].t0_us, 10000U);
    EXPECT_NEAR(out[0].velocity.x, 0.8f + 0.2f, 1e-6f);  // half of the straddling tick
    EXPECT_EQ(out[0].samples, 10);
    EXPECT_EQ(out[0].flags, 1 | 4);
    // the other half starts the next sub-interval
    ASSERT_EQ(stage.push(22000, 30000, Vector3f(), Vector3f(0.8f, 0, 0), 8, 1, out, 2), 1);
    EXPECT_EQ(out[0].t0_us, 20000U);
    EXPECT_NEAR(out[0].velocity.x, 0.2f + 0.8f, 1e-6f);
    EXPECT_EQ(out[0].flags, 1 | 4);
}

// A discontinuity drops the partial sub-interval and restarts on the next grid point; a long
// tick completes several sub-intervals, more than the caller's capacity are counted.
TEST(CoopImuStage, DiscontinuityAndLongTick)
{
    AP_CoopBridge_ImuStage stage;
    Sub out[2];
    stage.push(5000, 10000, Vector3f(), Vector3f(), 0, 0, out, 2);
    stage.push(10000, 15000, Vector3f(), Vector3f(1, 0, 0), 0, 0, out, 2);
    // jump: 15 ms -> 16 ms is missing; nothing of [10, 15) ms may be sent
    EXPECT_EQ(stage.push(16000, 21000, Vector3f(), Vector3f(1, 0, 0), 0, 0, out, 2), 0);
    EXPECT_TRUE(stage.running());   // restarted at 20 ms
    ASSERT_EQ(stage.push(21000, 30000, Vector3f(), Vector3f(0.9f, 0, 0), 0, 0, out, 2), 1);
    EXPECT_EQ(out[0].t0_us, 20000U);
    EXPECT_NEAR(out[0].velocity.x, 0.2f + 0.9f, 1e-6f);
    // a 35 ms tick completes three sub-intervals (30-40, 40-50, 50-60); two fit
    EXPECT_EQ(stage.push(30000, 65000, Vector3f(), Vector3f(3.5f, 0, 0), 0, 0, out, 2), 3);
    EXPECT_EQ(out[0].t0_us, 30000U);
    EXPECT_EQ(out[1].t0_us, 40000U);
    EXPECT_NEAR(out[1].velocity.x, 1.0f, 1e-6f);
}

#endif  // AP_COOPBRIDGE_ENABLED

AP_GTEST_MAIN()
