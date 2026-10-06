#include <AP_gtest.h>
#include <AP_Motors/AP_Motors_Thrust_Cubic.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Independent reference from Bitcraze's CF2.1 Brushless calibration.
static const AP_Motors_Thrust_Cubic model({-0.014058926705279723f, 0.04265273261724981f,
                                0.0018327760144017432f, 0.0020576974784587178f,
                                0.02136263065537499f, 0.2f, 4.2f, 1.024f, -0.024f});

static double reference_voltage(double force)
{
    const double c0 = -0.014058926705279723;
    const double c1 = 0.04265273261724981;
    const double c2 = 0.0018327760144017432;
    const double c3 = 0.0020576974784587178;
    const double p = -c2 / (3 * c3);
    const double q = p * p * p + (c2 * c1 - 3 * c3 * (c0 - force)) / (6 * c3 * c3);
    const double r = c1 / (3 * c3);
    const double root = sqrt(q * q + pow(r - p * p, 3));
    return cbrt(q + root) + cbrt(q - root) + p;
}

TEST(ThrustCubic, MatchesCrazyflieDshot)
{
    ASSERT_TRUE(model.valid());
    for (uint16_t millivolts = 3300; millivolts <= 4200; millivolts += 100) {
        const float battery = millivolts * 0.001f;
        for (uint16_t millinewtons = 40; millinewtons <= 200; millinewtons++) {
            const float force = millinewtons * 0.001f;
            const float actuator = model.actuator(force, battery, 0.25f, 0.95f);
            const uint16_t pwm = 1000 + 1000 * actuator;
            const int ap_dshot = MIN(2 * (pwm - 1000), 1999) + 48;
            const int cf_dshot = uint16_t(65535 * reference_voltage(force) / battery) >> 5;
            EXPECT_LE(abs(ap_dshot - cf_dshot), 2);
            EXPECT_NEAR(model.force(actuator, battery), force, 4.0e-6f);
        }
    }
}

TEST(ThrustCubic, MonotonicBoundedAndSaturated)
{
    for (float voltage : {3.0f, 3.3f, 3.7f, 4.2f}) {
        float previous = 0.25f;
        for (uint16_t step = 0; step <= 1000; step++) {
            const float actuator = model.actuator(step * 0.0004f, voltage, 0.25f, 0.95f);
            EXPECT_GE(actuator, previous);
            EXPECT_GE(actuator, 0.25f);
            EXPECT_LE(actuator, 0.95f);
            previous = actuator;
        }
        EXPECT_FLOAT_EQ(model.actuator(-1.0f, voltage, 0.25f, 0.95f), 0.25f);
        EXPECT_FLOAT_EQ(model.actuator(1.0f, voltage, 0.25f, 0.95f), 0.95f);
    }
    EXPECT_LT(model.available_thrust(3.0f, 0.95f), 0.2f);
    EXPECT_FLOAT_EQ(model.available_thrust(4.2f, 0.95f), 0.2f);
}

TEST(ThrustCubic, InvalidInputsAndMissingProfile)
{
    EXPECT_FALSE(AP_Motors_Thrust_Cubic({}).valid());
    EXPECT_FLOAT_EQ(model.actuator(NAN, 3.7f, 0.25f, 0.95f), 0.25f);
    EXPECT_FLOAT_EQ(model.actuator(INFINITY, 3.7f, 0.25f, 0.95f), 0.25f);
    EXPECT_FLOAT_EQ(model.actuator(0.1f, NAN, 0.25f, 0.95f), 0.25f);
    EXPECT_FLOAT_EQ(model.actuator(0.1f, 0.0f, 0.25f, 0.95f), 0.25f);
    EXPECT_FLOAT_EQ(model.actuator(0.1f, -1.0f, 0.25f, 0.95f), 0.25f);
}

AP_GTEST_MAIN()
