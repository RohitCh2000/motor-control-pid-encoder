// pid.cpp - See pid.h for the interface.
#include "pid.h"

#include <algorithm>

PidController::PidController(double kp, double ki, double kd,
                             double out_min, double out_max,
                             double integral_limit)
    : kp_(kp), ki_(ki), kd_(kd),
      out_min_(out_min), out_max_(out_max),
      integral_limit_(integral_limit) {}

void PidController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    last_output_ = 0.0;
    first_ = true;
}

double PidController::update(double setpoint, double measured, double dt) {
    if (dt <= 0.0) return last_output_;

    const double error = setpoint - measured;

    // Proportional.
    const double p = kp_ * error;

    // Derivative on error (zero on the first step to avoid a kick).
    const double d = first_ ? 0.0 : kd_ * (error - prev_error_) / dt;

    // Tentative integral with clamping (anti-windup, method 1).
    double new_integral = integral_ + error * dt;
    const double i_clamped =
        std::clamp(ki_ * new_integral, -integral_limit_, integral_limit_);
    new_integral = (ki_ != 0.0) ? (i_clamped / ki_) : new_integral;

    double output = p + ki_ * new_integral + d;

    // Output clamp + conditional integration (anti-windup, method 2):
    // if the output is saturated and the error would push it further
    // into saturation, do not accept the new integral accumulation.
    const double clamped = std::clamp(output, out_min_, out_max_);
    const bool saturated = (clamped != output);
    const bool pushing_further =
        (output > out_max_ && error > 0.0) || (output < out_min_ && error < 0.0);
    if (!(saturated && pushing_further)) {
        integral_ = new_integral;
        output = p + ki_ * integral_ + d;
        output = std::clamp(output, out_min_, out_max_);
    } else {
        output = clamped;
    }

    prev_error_ = error;
    first_ = false;
    last_output_ = output;
    return output;
}
