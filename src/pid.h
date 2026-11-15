// pid.h - Discrete PID controller with output clamping and anti-windup.
//
// Clean, dependency-free implementation suitable for embedded motor
// control: fixed-step update, integral clamping (conditional integration
// is also applied when the output is saturated), and derivative on the
// error with an optional first-order filter applied by the caller via
// the measured value it passes in.
#pragma once

class PidController {
public:
    // kp, ki, kd: PID gains (ki in 1/s, kd in s, both scaled by dt in update).
    // out_min / out_max: output clamp, e.g. PWM duty 0..100 (%).
    // integral_limit: absolute clamp on the integral term's contribution.
    PidController(double kp, double ki, double kd,
                  double out_min, double out_max,
                  double integral_limit = 100.0);

    // Reset integral and previous-error state.
    void reset();

    // One control step. Returns the clamped control output.
    //   setpoint: desired value (e.g. RPM)
    //   measured: current measured value (e.g. encoder RPM)
    //   dt:       step period in seconds (must be > 0)
    double update(double setpoint, double measured, double dt);

    double integral() const { return integral_; }
    double last_output() const { return last_output_; }

private:
    double kp_, ki_, kd_;
    double out_min_, out_max_;
    double integral_limit_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double last_output_ = 0.0;
    bool first_ = true;
};
