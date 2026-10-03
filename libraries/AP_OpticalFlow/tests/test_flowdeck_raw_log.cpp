#include <AP_gtest.h>

#include <AP_OpticalFlow/AP_OpticalFlow_FlowDeck_RawLog.h>

using RawLog = AP_OpticalFlow_FlowDeck_RawLog;

TEST(FlowDeckRawLog, EmptyQueue)
{
    RawLog queue;
    RawLog::Sample sample {};
    EXPECT_FALSE(queue.pop(sample));
}

TEST(FlowDeckRawLog, PreservesPollAndReadTimestamps)
{
    RawLog queue;
    RawLog::Sample input {};
    input.time_us = uint64_t(UINT32_MAX) + 10000;
    input.dt_us = 10003;
    input.read_start_us = 4;
    input.read_end_us = 207;
    input.gyro_us = 211;
    input.delta_x = -23;
    input.delta_y = 47;
    input.gyro_x = -0.75f;
    input.gyro_y = 1.5f;
    input.gyro_z = -2.5f;
    input.motion = 0xB0;
    input.quality = 127;
    input.reason = RawLog::Reason::Accepted;
    queue.push(input);
    RawLog::Sample output {};
    ASSERT_TRUE(queue.pop(output));
    EXPECT_EQ(output.time_us, input.time_us);
    EXPECT_EQ(output.dt_us, input.dt_us);
    EXPECT_EQ(output.read_start_us, input.read_start_us);
    EXPECT_EQ(output.read_end_us, input.read_end_us);
    EXPECT_EQ(output.gyro_us, input.gyro_us);
    EXPECT_EQ(output.delta_x, input.delta_x);
    EXPECT_EQ(output.delta_y, input.delta_y);
    EXPECT_FLOAT_EQ(output.gyro_x, input.gyro_x);
    EXPECT_FLOAT_EQ(output.gyro_y, input.gyro_y);
    EXPECT_FLOAT_EQ(output.gyro_z, input.gyro_z);
    EXPECT_EQ(output.motion, input.motion);
    EXPECT_EQ(output.quality, input.quality);
    EXPECT_EQ(output.reason, input.reason);
    EXPECT_EQ(output.sequence, 1U);
    EXPECT_EQ(output.dropped, 0U);
}

TEST(FlowDeckRawLog, OverflowPreservesQueuedRecordsAndReportsLoss)
{
    RawLog queue;
    RawLog::Sample input {};
    for (uint32_t i = 0; i < RawLog::CAPACITY + 3U; i++) {
        input.time_us = i * 10000;
        queue.push(input);
    }
    RawLog::Sample output {};
    for (uint32_t i = 0; i < RawLog::CAPACITY; i++) {
        ASSERT_TRUE(queue.pop(output));
        EXPECT_EQ(output.time_us, i * 10000);
        EXPECT_EQ(output.sequence, i + 1);
        EXPECT_EQ(output.dropped, 0U);
    }
    EXPECT_FALSE(queue.pop(output));
    queue.push(input);
    ASSERT_TRUE(queue.pop(output));
    EXPECT_EQ(output.sequence, RawLog::CAPACITY + 4U);
    EXPECT_EQ(output.dropped, 3U);
}

TEST(FlowDeckRawLog, FifoSurvivesRepeatedIndexWrap)
{
    RawLog queue;
    RawLog::Sample input {};
    RawLog::Sample output {};
    for (uint32_t batch = 0; batch < 100; batch++) {
        for (uint32_t j = 0; j < 7; j++) {
            input.dt_us = batch * 7 + j;
            queue.push(input);
        }
        for (uint32_t j = 0; j < 7; j++) {
            ASSERT_TRUE(queue.pop(output));
            EXPECT_EQ(output.dt_us, batch * 7 + j);
            EXPECT_EQ(output.sequence, batch * 7 + j + 1);
            EXPECT_EQ(output.dropped, 0U);
        }
    }
}

TEST(FlowDeckRawLog, ClearDiscardsRecordsAndPreservesCounters)
{
    RawLog queue;
    RawLog::Sample input {};
    RawLog::Sample output {};
    for (uint8_t i = 0; i < RawLog::CAPACITY + 1; i++) {
        queue.push(input);
    }
    queue.clear();
    EXPECT_FALSE(queue.pop(output));
    input.time_us = 123456;
    queue.push(input);
    ASSERT_TRUE(queue.pop(output));
    EXPECT_EQ(output.time_us, 123456U);
    EXPECT_EQ(output.sequence, RawLog::CAPACITY + 2U);
    EXPECT_EQ(output.dropped, 1U);
    EXPECT_FALSE(queue.pop(output));
}

AP_GTEST_MAIN()
