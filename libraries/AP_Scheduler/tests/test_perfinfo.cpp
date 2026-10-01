#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/PerfInfo.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(PerfInfo, SeparatesSlipsFromExecutionOverruns)
{
    AP::PerfInfo perf;
    perf.allocate_task_info(1);
    ASSERT_TRUE(perf.has_task_info());

    const auto *info = perf.get_task_info(0);
    EXPECT_EQ(info->tick_count, 0U);
    EXPECT_EQ(info->name, nullptr);

    perf.task_slipped(0);
    perf.update_task_info(0, 100, false, "test_task");
    perf.update_task_info(0, 300, true, "test_task");
    EXPECT_EQ(info->slip_count, 1U);
    EXPECT_EQ(info->overrun_count, 1U);
    EXPECT_EQ(info->tick_count, 2U);
    EXPECT_EQ(info->elapsed_time_us, 400U);
    EXPECT_EQ(info->min_time_us, 100U);
    EXPECT_EQ(info->max_time_us, 300U);
    EXPECT_STREQ(info->name, "test_task");

    perf.reset();
    EXPECT_EQ(info->tick_count, 0U);
    EXPECT_EQ(info->elapsed_time_us, 0U);
    EXPECT_EQ(info->slip_count, 0U);
    EXPECT_EQ(info->overrun_count, 0U);
    EXPECT_EQ(info->name, nullptr);
    perf.free_task_info();
    EXPECT_FALSE(perf.has_task_info());
    EXPECT_EQ(perf.get_task_info(0), nullptr);
    perf.update_task_info(0, 100, false, "disabled");
    perf.task_slipped(0);
}

TEST(PerfInfo, LoopTimingAccumulatesAndResets)
{
    AP::PerfInfo perf{};
    const auto stage = AP::PerfInfo::LoopStage::IMU_WAIT;
    const auto &timing = perf.get_loop_timing(stage);
    EXPECT_EQ(timing.count, 0U);
    perf.record_loop_stage(stage, 0);
    perf.record_loop_stage(stage, UINT32_MAX);
    perf.record_loop_stage(stage, 100);
    EXPECT_EQ(timing.count, 3U);
    EXPECT_EQ(timing.total_us, uint64_t(UINT32_MAX) + 100);
    EXPECT_EQ(timing.max_us, UINT32_MAX);
    EXPECT_EQ(perf.get_loop_timing(AP::PerfInfo::LoopStage::SCHED_RUN).count, 0U);
    perf.record_sample_wait(2, true);
    perf.record_sample_wait(3, false);
    EXPECT_EQ(perf.get_sample_wait_count(), 2U);
    EXPECT_EQ(perf.get_sample_poll_count(), 5U);
    EXPECT_EQ(perf.get_sample_rephase_count(), 1U);
    perf.reset();
    EXPECT_EQ(timing.count, 0U);
    EXPECT_EQ(timing.total_us, 0U);
    EXPECT_EQ(timing.max_us, 0U);
    EXPECT_EQ(perf.get_sample_wait_count(), 0U);
    EXPECT_EQ(perf.get_sample_poll_count(), 0U);
    EXPECT_EQ(perf.get_sample_rephase_count(), 0U);
}

AP_GTEST_MAIN()
