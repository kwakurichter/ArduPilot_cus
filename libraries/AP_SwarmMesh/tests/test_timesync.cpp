/*
  Unit tests for AP_SwarmMesh_TimeSync, the follower's mesh clock offset estimator.

  A reference clock and a follower clock with a known offset (and drift) exchange SYNC
  samples with random non-negative delays; the estimate must recover the offset to within
  the smallest delay in its window, follow a new target at the slew rate when stepping is not
  allowed, and step large errors on the ground.
 */
#include <AP_gtest.h>

#include <AP_SwarmMesh/AP_SwarmMesh_TimeSync.h>
#include <AP_SwarmMesh/AP_SwarmMesh_config.h>

#if AP_SWARMMESH_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

// deterministic delays in [lo, lo + span) us
struct Lcg {
    uint32_t s = 12345;
    uint32_t next(uint32_t span) { s = s * 1664525U + 1013904223U; return (s >> 8) % span; }
};

}  // namespace

// Constant offset, delays 200..10200 us with at least one short one per window: the estimate
// is within the window's smallest delay of the truth.
TEST(SwarmMeshTimeSync, RecoversOffset)
{
    AP_SwarmMesh_TimeSync sync;
    AP_SwarmMesh_TimeSync::Config cfg;
    const int64_t true_offset = 12345678;     // reference = local + true_offset
    Lcg rng;
    uint64_t local = 5000000;
    for (int i = 0; i < 40; i++) {
        const uint32_t delay = (i % 4 == 0) ? 200 : 200 + rng.next(10000);
        const uint64_t ref_tx = local + true_offset;
        sync.sample(cfg, 7, ref_tx, local + delay);
        sync.update(cfg, local + delay, true);
        local += 500000;
    }
    EXPECT_EQ(sync.reference(), 7);
    EXPECT_NEAR(double(sync.offset_us()), double(true_offset - 200), 1.0);
    EXPECT_TRUE(sync.locked(cfg, local));
    EXPECT_FALSE(sync.locked(cfg, local + cfg.timeout_us + 1));
}

// The fixed latency moves the estimate onto the truth.
TEST(SwarmMeshTimeSync, Latency)
{
    AP_SwarmMesh_TimeSync sync;
    AP_SwarmMesh_TimeSync::Config cfg;
    cfg.latency_us = 300;
    uint64_t local = 1000000;
    for (int i = 0; i < 10; i++) {
        sync.sample(cfg, 1, local - 4000, local + 300);
        sync.update(cfg, local + 300, true);
        local += 500000;
    }
    EXPECT_EQ(sync.offset_us(), -4000);
}

// In flight (no stepping) a 30 ms change of the target is followed at the slew rate only;
// on the ground an error above step_us (20 ms) is stepped.
TEST(SwarmMeshTimeSync, SlewsInFlight)
{
    AP_SwarmMesh_TimeSync sync;
    AP_SwarmMesh_TimeSync::Config cfg;
    cfg.window = 1;
    uint64_t local = 1000000;
    sync.sample(cfg, 1, local + 1000, local);
    sync.update(cfg, local, false);
    EXPECT_EQ(sync.offset_us(), 1000);          // the first target is taken at once
    sync.sample(cfg, 1, local + 31000, local);  // the reference now 30 ms further ahead
    uint64_t prev = local + sync.offset_us();
    for (int i = 1; i <= 100; i++) {            // 1 s in 10 ms ticks
        const uint64_t t = local + i * 10000;
        sync.update(cfg, t, false);
        const uint64_t mesh = t + sync.offset_us();
        EXPECT_GT(mesh, prev);                   // monotonic
        prev = mesh;
    }
    EXPECT_EQ(sync.offset_us(), 1000 + 500);    // 500 ppm over 1 s
    // on the ground it steps
    sync.update(cfg, local + 1010000, true);
    EXPECT_EQ(sync.offset_us(), 31000);
}

// Drifting clocks (50 ppm): the estimate stays within the drift over the window plus the
// shortest delay.
TEST(SwarmMeshTimeSync, Drift)
{
    AP_SwarmMesh_TimeSync sync;
    AP_SwarmMesh_TimeSync::Config cfg;
    Lcg rng;
    const double ppm = 50e-6;
    uint64_t local = 1000000;
    double worst = 0;
    for (int i = 0; i < 200; i++) {
        const int64_t truth = 777000 + int64_t(ppm * double(local));
        const uint32_t delay = 100 + rng.next(3000);
        sync.sample(cfg, 3, local + truth, local + delay);
        sync.update(cfg, local + delay, i < 20);
        if (i >= 20) {
            const double err = double(sync.offset_us() - truth);
            worst = fabs(err) > worst ? fabs(err) : worst;
        }
        local += 500000;
    }
    // 8 samples x 0.5 s x 50 ppm = 200 us, plus the window's shortest delay (< 3100 us)
    EXPECT_LT(worst, 3300.0);
}

// A new reference starts a new window.
TEST(SwarmMeshTimeSync, NewReference)
{
    AP_SwarmMesh_TimeSync sync;
    AP_SwarmMesh_TimeSync::Config cfg;
    sync.sample(cfg, 5, 2000000, 1000000);
    sync.update(cfg, 1000000, true);
    EXPECT_EQ(sync.offset_us(), 1000000);
    sync.sample(cfg, 2, 3500000, 1500000);
    sync.update(cfg, 1500000, true);
    EXPECT_EQ(sync.reference(), 2);
    EXPECT_EQ(sync.offset_us(), 2000000);
}

#endif  // AP_SWARMMESH_ENABLED

AP_GTEST_MAIN()
