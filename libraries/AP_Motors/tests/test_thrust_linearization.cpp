#include <AP_gtest.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_Motors_Thrust_Cubic.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class AP_MotorsMulticopter_test : public AP_MotorsMulticopter {
public:
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override {}
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {}
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override {}
    const char* _get_frame_string() const override { return "TEST"; }
    void output_armed_stabilizing() override {}
    void output_to_motors() override {}

    void configure_expo(float expo, float voltage_ratio)
    {
#if AP_MOTORS_THRUST_CUBIC_ENABLED
        thr_lin.thrust_model.set(0);
#endif
        thr_lin.curve_expo.set(expo);
        thr_lin.spin_min.set(0.15f);
        thr_lin.spin_max.set(0.95f);
        thr_lin.batt_voltage_filt.reset(voltage_ratio);
        thr_lin.lift_max = voltage_ratio * (1 - expo) + expo * sq(voltage_ratio);
    }

    int16_t shutdown_pwm()
    {
        _spool_state = SpoolState::SHUT_DOWN;
        _disarm_disable_pwm.set(0);
        return output_to_pwm(1.0f);
    }

    float idle_actuator()
    {
        _spin_up_ratio = 0.4f;
        return actuator_spin_up_to_ground_idle();
    }

#if AP_MOTORS_THRUST_CUBIC_ENABLED
    void configure_cubic(float voltage, float spin_max = 0.95f)
    {
        thr_lin.thrust_model.set(1);
        thr_lin.spin_min.set(0.25f);
        thr_lin.spin_max.set(spin_max);
        thr_lin.batt_voltage_min.set(3.3f);
        thr_lin.batt_voltage_max.set(4.2f);
        thr_lin.cubic_voltage_ready = false;
        _pwm_type.set(PWMType::DSHOT300);
        set_dt_s(0.001f);
        sample(voltage, true);
    }

    void sample(float voltage, bool healthy) { thr_lin.update_cubic_voltage(voltage, healthy); }
    float voltage() const { return thr_lin.cubic_voltage_filt.get(); }
    void set_spin_min(float value) { thr_lin.spin_min.set(value); }
    void set_model(int8_t value) { thr_lin.thrust_model.set(value); }
#endif
};

static AP_MotorsMulticopter_test motors;

TEST(ThrustLinearization, ExpoRegression)
{
    for (float expo : {0.0f, 0.45f, 0.65f, 1.0f}) {
        for (float ratio : {0.8f, 0.9f, 1.0f}) {
            motors.configure_expo(expo, ratio);
            const float lift = ratio * (1 - expo) + expo * ratio * ratio;
            for (uint16_t i = 0; i <= 100; i++) {
                const float thrust = i * 0.01f;
                const float throttle = is_zero(expo) ? thrust :
                    (sqrtf(sq(1 - expo) + 4 * expo * lift * thrust) - (1 - expo)) / (2 * expo * ratio);
                const float actuator = motors.thr_lin.thrust_to_actuator(thrust);
                EXPECT_NEAR(actuator, 0.15f + 0.8f * throttle, 1.0e-6f);
                EXPECT_NEAR(motors.thr_lin.actuator_to_thrust(actuator), thrust, 2.0e-6f);
            }
        }
    }
}

#if AP_MOTORS_THRUST_CUBIC_ENABLED
static const AP_Motors_Thrust_Cubic model(AP_MOTORS_THRUST_CUBIC_PROFILE);

TEST(ThrustLinearization, CubicMixerCompensationAndInverse)
{
    ASSERT_TRUE(model.valid());
    // Lower spin_max exercises available-thrust compensation and early mixer saturation.
    for (float spin_max : {0.8f, 0.95f}) {
        for (float voltage : {3.3f, 3.7f, 4.2f}) {
            motors.configure_cubic(voltage, spin_max);
            const float available = model.available_thrust(voltage, spin_max);
            const float gain = motors.thr_lin.get_compensation_gain();
            EXPECT_NEAR(gain, model.maximum_thrust() / available, 1.0e-6f);
            for (uint16_t i = 25; i <= 100; i++) {
                const float demand = i * 0.01f;
                const float mixed = constrain_float(demand * gain, 0.0f, 1.0f);
                const float actuator = motors.thr_lin.thrust_to_actuator(mixed);
                EXPECT_NEAR(model.force(actuator, voltage), MIN(demand * model.maximum_thrust(), available), 4.0e-6f);
                EXPECT_NEAR(motors.thr_lin.actuator_to_thrust(actuator), mixed, 3.0e-5f);
            }
            EXPECT_FLOAT_EQ(motors.thr_lin.thrust_to_actuator(0), 0.25f);
            EXPECT_EQ(motors.shutdown_pwm(), 1000);
            EXPECT_NEAR(motors.idle_actuator(), 0.1f, 1.0e-6f);
        }
    }
}

TEST(ThrustLinearization, VoltageFilteringAndTelemetryLoss)
{
    motors.configure_cubic(4.2f);
    motors.sample(3.3f, true);
    EXPECT_GT(motors.voltage(), 3.3f);
    EXPECT_LT(motors.voltage(), 4.2f);
    for (uint16_t i = 0; i < 1000; i++) {
        motors.sample(3.3f, true);
    }
    EXPECT_NEAR(motors.voltage(), 3.3f, 0.0001f);
    const float previous = motors.voltage();
    for (float bad_voltage : {NAN, INFINITY, -1.0f, 0.0f, 1.0f, 10.0f}) {
        motors.sample(bad_voltage, true);
        EXPECT_FLOAT_EQ(motors.voltage(), previous);
    }
    motors.sample(4.2f, false);
    EXPECT_FLOAT_EQ(motors.voltage(), previous);
    motors.sample(3.0f, true);
    EXPECT_NEAR(motors.voltage(), 3.3f, 0.0001f);
}

TEST(ThrustLinearization, InvalidModelAndUncalibratedIdle)
{
    motors.configure_cubic(3.7f);
    motors.set_model(2);
    EXPECT_FALSE(motors.thr_lin.cubic_configuration_valid());
    motors.set_model(1);
    motors.set_spin_min(0.15f);
    EXPECT_FALSE(motors.thr_lin.cubic_configuration_valid());
    motors.set_model(0);
    EXPECT_TRUE(motors.thr_lin.cubic_configuration_valid());
}
#endif // AP_MOTORS_THRUST_CUBIC_ENABLED

AP_GTEST_MAIN()
