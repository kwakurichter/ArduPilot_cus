# Crazyflie 2.1 Brushless cubic thrust model

This opt-in model uses the stock Crazyflie 2.1 Brushless motor, propeller and ESC
calibration from Bitcraze's `crazyflie-firmware`,
`src/platform/interface/platform_defaults_cf21bl.h`. Its coefficients are
compiled into the `crazyflie2_bl` board definition. The existing expo model
remains the default (`MOT_THST_MODEL=0`). This implementation and its tests are
AI-assisted and require human review and thrust-stand validation before flight.

The calibration fits per-motor force in newtons to effective motor voltage:

```text
F(V) = -0.014058926705279723
       + 0.04265273261724981 V
       + 0.0018327760144017432 V²
       + 0.0020576974784587178 V³
```

See [Bitcraze's calibration discussion](https://www.bitcraze.io/2025/10/keeping-thrust-consistent-as-the-battery-drains/).

## Integration

The reference maximum is 0.2 N per motor (0.8 N total). The model calculates the
available force at `MOT_SPIN_MAX`, capped at that reference. `lift_max` is the
available/reference force ratio. The mixer uses its existing `1/lift_max`
compensation; the output conversion multiplies mixed thrust by `lift_max` and
the reference force before inverting the polynomial. Air-density compensation
remains in the existing mixer path. This avoids compensating battery voltage twice.

Inversion uses 16 bounded bisection steps on the monotonic polynomial. This avoids
cube-root cancellation and has fixed execution cost. Effective motor voltage is
converted to ArduPilot's actuator convention using
`actuator = 1.024 * Vmotor / Vbattery - 0.024`. This accounts for Crazyflie's
approximately 2048-count ratio scale and ArduPilot's approximately 2000-count
scale with a 48-count DShot offset. Integer PWM and DShot quantization remain.
The calibration is for ordinary, non-reversible DShot operation.

Measured battery voltage is filtered at 1.6 Hz using the actual motor-loop time
step. The first healthy sample initializes the filter. `MOT_BAT_VOLT_MIN/MAX`
bound compensation, so voltage below the configured minimum receives no further
compensation. Unhealthy, nonfinite or implausible voltage samples retain the last
usable value; existing battery failsafes remain responsible for the response.
The cubic model always uses measured voltage, regardless of `MOT_OPTIONS` bit 0.
`MOT_THST_EXPO` is ignored only while this model is selected.

Shutdown, ground-idle, spool transitions and slew limiting retain the existing
ArduPilot code paths. `MOT_SPIN_MIN/MAX` are absolute actuator limits, not an
additional scale applied to the calibrated command. Requests below the force at
`SPIN_MIN` are clamped to that idle output; they cannot be reproduced physically.
The inverse reports modeled force, including that minimum force, rather than
claiming a perfect round trip through clipping. The arming check rejects an idle
setting below the measured curve's minimum of 0.0213626 N at the minimum voltage.

## Opting in

1. Save the aircraft's current parameters, particularly the spin limits, hover
   thrust, battery limits and motor protocol.
2. Build for `crazyflie2_bl` and load `crazyflie_bl_cubic.params`. This selects the
   cubic model and DShot300, sets the voltage limits to 3.3–4.2 V, and sets the
   spin limits to 0.25–0.95. No parameters are sent to an aircraft by this change.
3. Set `MOT_THST_HOVER` for the new physical thrust scale. A starting estimate is
   `mass_kg * 9.80665 / 0.8`; for a 40 g aircraft this is approximately 0.49.
   Use the actual mass with its battery and decks. The old learned hover value
   uses a different scale and must not be assumed valid.
4. Reboot. Verify battery readings, normal DShot operation and idle behavior.
   Measure force at several requests and supply voltages on a secured thrust
   stand before flight. Review attitude and altitude tuning after changing the
   thrust scale.

To return to the expo model, set `MOT_THST_MODEL=0`, restore the saved spin and
hover settings (and any other parameters changed by the opt-in file), and reboot.

## Validation

Unit tests cover the calibrated DShot mapping, monotonicity, clipping, forward
and inverse conversion, mixer compensation, voltage filtering and loss of
telemetry, shutdown/idle output, invalid configuration, and expo regression.
They test command/model agreement; they do not establish real thrust accuracy.

To include the board's calibration in a SITL test build:

```sh
rg '^define AP_MOTORS_THRUST_CUBIC_' libraries/AP_HAL_ChibiOS/hwdef/crazyflie2_bl/hwdef.dat > /tmp/cubic-sitl.dat
# This branch's SwarmMesh SITL header needs this otherwise undefined option.
printf '%s\n' 'define AP_SIM_SWARMMESH_LOSS_ENABLED 0' >> /tmp/cubic-sitl.dat
./waf configure --board sitl --no-submodule-update --extra-hwdef /tmp/cubic-sitl.dat
./waf --targets tests/test_thrust_cubic,tests/test_thrust_linearization
./build/sitl/tests/test_thrust_cubic
./build/sitl/tests/test_thrust_linearization
```

The stock SITL motor physics do not implement this measured cubic calibration.
A flight using the stock simulator is not evidence of hardware thrust accuracy.

Validation performed on `codex/flowdeck-phase2`:

- `crazyflie2_bl` Copter builds passed with cubic support both enabled and disabled.
- Seven focused unit tests passed with the profile enabled. The four applicable
  tests also passed with cubic support disabled.
- Copter `MotorTest` passed with the default expo model.
- A separate SITL instance running the cubic model rejected `SPIN_MIN=0.15` and
  accepted `SPIN_MIN=0.25` in its motor configuration check at 3.7 V. This was a
  motor-check test, not an armed flight test.
- Copter `TakeoffAlt` timed out waiting for waypoint 2, both with the feature
  available but unselected and with the feature compiled out. That unresolved
  branch/environment failure is not treated as a successful flight validation.
- Python lint and `git diff --check` passed.

After integrating branch updates through `c0b03904fd`, the combined
`crazyflie2_bl` Copter build passed. All 21 C++ tests passed (seven thrust-model
tests and 14 FlowDeck timing/logging tests), along with all 31 companion Guided
sequencer tests. Python lint and `git diff --check` also passed. Flight tests were
not repeated for this integration; the `TakeoffAlt` limitation above remains.

The user subsequently supplied `log_28_UnknownDate.bin` from a hardware GUIDED
flight with `MOT_THST_MODEL=1` and DShot300. During the settled segment (118–165 s
after boot), estimated altitude tracking error was 2.1 cm RMS. Early and late
window averages showed battery voltage falling from 3.804 to 3.663 V while mean
motor command rose from 70.25% to 73.02% and normalized thrust stayed near 0.701.
No sampled motor output reached its configured limits during this segment.
This supports functional operation, not independent thrust calibration or an
improvement over the expo model without a matched baseline flight.

Outstanding observations from that flight are a 31 cm estimated takeoff
overshoot, unchanged hover thrust of 0.474 versus settled demand near 0.70, and
loss of the EKF relative-position estimate near touchdown followed by an EKF
failsafe. Hover learning is capped at 0.6875 in this branch; reconciling the
physical thrust scale, actual flying mass and hover-learning behavior remains
necessary. The flight observations do not establish the cause of the landing
estimator issue.

Builds used the existing local submodule checkouts with automatic updates disabled.
No thrust-stand measurement or isolated on-device execution-time measurement has
been performed for this change.
