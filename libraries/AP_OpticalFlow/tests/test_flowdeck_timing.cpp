#include <AP_gtest.h>

#include <AP_OpticalFlow/AP_OpticalFlow_FlowDeck_Timing.h>

TEST(FlowDeckTiming, ContiguousWindow)
{
    AP_OpticalFlow_FlowDeck_Timing timing;
    for (uint32_t end = 10000; end <= 100000; end += 10000) {
        timing.add_sample(end, 10000);
    }
    EXPECT_EQ(timing.accepted_us(), 100000U);
    EXPECT_EQ(timing.span_us(), 100000U);
    EXPECT_EQ(timing.mean_age_us(100000), 50000U);
    EXPECT_EQ(timing.last_age_us(100000), 0U);
}

TEST(FlowDeckTiming, RejectedSamplesExtendSpanAndAge)
{
    AP_OpticalFlow_FlowDeck_Timing timing;
    for (uint32_t end = 10000; end <= 190000; end += 20000) {
        timing.add_sample(end, 10000);
    }
    EXPECT_EQ(timing.accepted_us(), 100000U);
    EXPECT_EQ(timing.span_us(), 190000U);
    EXPECT_EQ(timing.mean_age_us(190000), 95000U);
    EXPECT_EQ(timing.last_age_us(190000), 0U);
    EXPECT_EQ(timing.mean_age_us(215000), 120000U);
    EXPECT_EQ(timing.last_age_us(215000), 25000U);
}

TEST(FlowDeckTiming, MidpointIsWeightedByIntervalDuration)
{
    AP_OpticalFlow_FlowDeck_Timing timing;
    timing.add_sample(10000, 10000);
    timing.add_sample(70000, 30000);
    EXPECT_EQ(timing.accepted_us(), 40000U);
    EXPECT_EQ(timing.span_us(), 70000U);
    EXPECT_EQ(timing.mean_age_us(80000), 37500U);
}

TEST(FlowDeckTiming, MicrosRollover)
{
    AP_OpticalFlow_FlowDeck_Timing timing;
    const uint32_t start = UINT32_MAX - 55000U;
    for (uint32_t dt = 10000; dt <= 100000; dt += 10000) {
        timing.add_sample(start + dt, 10000);
    }
    EXPECT_EQ(timing.accepted_us(), 100000U);
    EXPECT_EQ(timing.span_us(), 100000U);
    EXPECT_EQ(timing.mean_age_us(start + 120000U), 70000U);
    EXPECT_EQ(timing.last_age_us(start + 120000U), 20000U);
}

TEST(FlowDeckTiming, LongGapDoesNotOverflowWeightedSum)
{
    AP_OpticalFlow_FlowDeck_Timing timing;
    timing.add_sample(10000, 10000);
    timing.add_sample(1000000000, 10000);
    EXPECT_EQ(timing.accepted_us(), 20000U);
    EXPECT_EQ(timing.span_us(), 1000000000U);
    EXPECT_EQ(timing.mean_age_us(1000000000), 500000000U);
}

TEST(FlowDeckTiming, SnapshotAndReset)
{
    AP_OpticalFlow_FlowDeck_Timing timing;
    timing.add_sample(10000, 10000);
    const auto snapshot = timing;
    timing = {};
    timing.add_sample(90000, 20000);
    EXPECT_EQ(snapshot.mean_age_us(90000), 85000U);
    EXPECT_EQ(timing.accepted_us(), 20000U);
    EXPECT_EQ(timing.span_us(), 20000U);
    EXPECT_EQ(timing.mean_age_us(90000), 10000U);
}

TEST(FlowDeckTiming, EmptyAndZeroDuration)
{
    AP_OpticalFlow_FlowDeck_Timing timing;
    timing.add_sample(10000, 0);
    EXPECT_EQ(timing.accepted_us(), 0U);
    EXPECT_EQ(timing.span_us(), 0U);
    EXPECT_EQ(timing.mean_age_us(10000), 0U);
}

TEST(FlowDeckMotion, RawStatusAndNonzeroCounts)
{
    AP_OpticalFlow_FlowDeck_MotionStats stats;
    stats.add(0xB0, true);
    stats.add(0x30, false);
    stats.add(0x30, true);
    EXPECT_EQ(stats.entries[0].status, 0xB0);
    EXPECT_EQ(stats.entries[0].count, 1);
    EXPECT_EQ(stats.entries[0].nonzero, 1);
    EXPECT_EQ(stats.entries[1].status, 0x30);
    EXPECT_EQ(stats.entries[1].count, 2);
    EXPECT_EQ(stats.entries[1].nonzero, 1);
    EXPECT_EQ(stats.overflow_count, 0);
}

TEST(FlowDeckMotion, BoundedHistogramPreservesCounts)
{
    AP_OpticalFlow_FlowDeck_MotionStats stats;
    for (uint8_t status = 0; status < 10; status++) {
        stats.add(status, (status & 1) != 0);
    }
    stats.add(0, true);
    EXPECT_EQ(stats.entries[0].count, 2);
    EXPECT_EQ(stats.entries[0].nonzero, 1);
    EXPECT_EQ(stats.overflow_count, 2);
    EXPECT_EQ(stats.overflow_nonzero, 1);
    stats = {};
    EXPECT_EQ(stats.entries[0].count, 0);
    EXPECT_EQ(stats.overflow_count, 0);
}

AP_GTEST_MAIN()
