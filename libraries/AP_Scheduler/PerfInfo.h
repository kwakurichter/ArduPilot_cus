#pragma once

#include "AP_Scheduler_config.h"

#if AP_SCHEDULER_ENABLED

#include <stdint.h>
#include <AP_Common/ExpandingString.h>

namespace AP {

class PerfInfo {
public:
    PerfInfo() {}

    // per-task timing information
    struct TaskInfo {
        const char *name;
        uint16_t min_time_us;
        uint16_t max_time_us;
        uint32_t elapsed_time_us;
        uint32_t tick_count;
        uint16_t slip_count;
        uint16_t overrun_count;

        void update(uint16_t task_time_us, bool overrun);
        void print(const char* task_name, uint32_t total_time, ExpandingString& str) const;
    };

    enum class LoopStage : uint8_t {
        IMU_WAIT,
        SLEEP_REQUEST,
        SLEEP_ACTUAL,
        WAKE_LATE,
        SAMPLE_WAIT,
        ENTRY_LATE,
        SCHED_LOCK,
        SCHED_RUN,
        LOOP_GAP,
        COUNT
    };

    struct LoopTiming {
        uint64_t total_us;
        uint32_t count;
        uint32_t max_us;
    };

    void record_loop_stage(LoopStage stage, uint32_t time_us);
    const LoopTiming &get_loop_timing(LoopStage stage) const { return _loop_timing[uint8_t(stage)]; }
    void record_sample_wait(uint32_t polls, bool rephased) {
        _sample_wait_count++;
        _sample_poll_count += polls;
        _sample_rephase_count += rephased;
    }
    uint32_t get_sample_wait_count() const { return _sample_wait_count; }
    uint32_t get_sample_poll_count() const { return _sample_poll_count; }
    uint32_t get_sample_rephase_count() const { return _sample_rephase_count; }

    /* Do not allow copies */
    CLASS_NO_COPY(PerfInfo);

    void reset();
    void ignore_this_loop();
    void check_loop_time(uint32_t time_in_micros);
    uint16_t get_num_loops() const;
    uint32_t get_max_time() const;
    uint32_t get_min_time() const;
    uint16_t get_num_long_running() const;
    uint32_t get_avg_time() const;
    uint32_t get_stddev_time() const;
    float    get_filtered_time() const;
    float get_filtered_loop_rate_hz() const;
    void set_loop_rate(uint16_t rate_hz);

    void update_logging() const;

    // allocate the array of task statistics for use by @SYS/tasks.txt
    void allocate_task_info(uint8_t num_tasks);
    void free_task_info();
    // whether or not we have task info allocated
    bool has_task_info() { return _task_info != nullptr; }
    // return a task info
    const TaskInfo* get_task_info(uint8_t task_index) const {
        return (_task_info && task_index < _num_tasks) ? &_task_info[task_index] : nullptr;
    }
    // called after each run of a task to update its statistics based on measurements taken by the scheduler
    void update_task_info(uint8_t task_index, uint16_t task_time_us, bool overrun, const char *name);
    // record that a task slipped
    void task_slipped(uint8_t task_index) {
        if (_task_info && task_index < _num_tasks) {
            _task_info[task_index].slip_count++;
        }
    }

private:
    LoopTiming _loop_timing[uint8_t(LoopStage::COUNT)] {};
    uint32_t _sample_wait_count = 0;
    uint32_t _sample_poll_count = 0;
    uint32_t _sample_rephase_count = 0;
    uint16_t loop_rate_hz;
    uint16_t overtime_threshold_micros;
    uint16_t loop_count;
    uint32_t max_time; // in microseconds
    uint32_t min_time; // in microseconds
    uint64_t sigma_time;
    uint64_t sigmasquared_time;
    uint16_t long_running;
    uint32_t last_check_us;
    float filtered_loop_time;
    bool ignore_loop;
    // performance monitoring
    uint8_t _num_tasks = 0;
    TaskInfo* _task_info = nullptr;
};

};

#endif  // AP_SCHEDULER_ENABLED
