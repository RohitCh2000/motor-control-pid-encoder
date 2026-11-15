// main.cpp - Self-contained DC motor speed-control simulation.
//
// A first-order-lag + inertia DC motor plant is driven by the PID
// controller in pid.h. Speed feedback comes from a (simulated)
// quadrature encoder with finite counts/rev, so the measured speed is
// quantized exactly as it would be on real hardware.
//
// Scenario (deterministic, no randomness):
//   t = 0.0 s   target steps to 120 RPM
//   t = 2.0 s   target steps to 150 RPM
//   t = 4.0 s   payload disturbance: inertia rises (lag tau 0.35 -> 0.60 s)
//               and a constant load drag of 30 RPM (steady-state equiv.)
//               is applied - the loop must recover to 150 RPM.
//
// Build: g++ -std=c++17 -Wall -Wextra -O2 src/main.cpp src/pid.cpp -o motor_pid_sim
// (or just `make`)

#include <cmath>
#include <cstdio>
#include <vector>

#include "pid.h"

namespace {

// --- Plant parameters -------------------------------------------------
constexpr double kDt = 0.01;              // control / plant step, s (100 Hz)
constexpr double kDuration = 6.0;         // total simulation time, s
constexpr double kMotorGain = 2.2;        // steady RPM per % PWM duty
constexpr double kTauNoLoad = 0.35;       // motor lag, s
constexpr double kTauLoaded = 0.60;       // motor lag with payload, s
constexpr double kLoadDragRpm = -30.0;    // load drag, steady-state RPM equiv.
constexpr double kEncoderCpr = 1024.0;    // encoder counts per revolution

// --- Scenario ----------------------------------------------------------
double target_rpm(double t) { return (t < 2.0) ? 120.0 : 150.0; }
bool payload_on(double t) { return t >= 4.0; }

struct Sample {
    double t, target, rpm;
};

}  // namespace

int main() {
    // Tuned for this plant: mostly PI, a touch of D to damp the steps.
    PidController pid(/*kp=*/1.8, /*ki=*/3.0, /*kd=*/0.015,
                      /*out_min=*/0.0, /*out_max=*/100.0,
                      /*integral_limit=*/100.0);

    double rpm = 0.0;              // true shaft speed
    double angle_rev = 0.0;        // true shaft angle, revolutions
    long long prev_counts = 0;     // encoder counts at previous step
    double measured_rpm = 0.0;     // low-pass filtered encoder speed

    std::vector<Sample> history;
    history.reserve(static_cast<size_t>(kDuration / kDt) + 1);

    std::printf("Motor PID simulation: dt=%.2fs, encoder %.0f CPR, "
                "PWM 0-100%%\n",
                kDt, kEncoderCpr);
    std::printf("%6s %9s %9s %8s %12s\n", "t[s]", "target", "actual",
                "PWM%", "enc_counts");
    std::printf("%6s %9s %9s %8s %12s\n", "", "[RPM]", "[RPM]", "", "");

    const int steps = static_cast<int>(kDuration / kDt);
    for (int i = 0; i <= steps; ++i) {
        const double t = i * kDt;
        const double target = target_rpm(t);

        // --- Encoder: quantize angle to whole counts, derive speed. ---
        const long long counts =
            static_cast<long long>(std::floor(angle_rev * kEncoderCpr));
        const double raw_rpm =
            (static_cast<double>(counts - prev_counts) / kEncoderCpr) * 60.0 / kDt;
        prev_counts = counts;
        if (i > 0) measured_rpm += 0.35 * (raw_rpm - measured_rpm);

        // --- Control. ---
        const double pwm = pid.update(target, measured_rpm, kDt);

        if (i % 20 == 0) {  // print every 0.2 s
            std::printf("%6.2f %9.1f %9.2f %8.1f %12lld\n", t, target, rpm,
                        pwm, counts);
        }
        history.push_back({t, target, rpm});

        // --- Plant: first-order lag toward the duty's steady speed. ---
        const double tau = payload_on(t) ? kTauLoaded : kTauNoLoad;
        const double load = payload_on(t) ? kLoadDragRpm : 0.0;
        const double steady_rpm = pwm * kMotorGain + load;
        rpm += (steady_rpm - rpm) * (kDt / tau);
        angle_rev += (rpm / 60.0) * kDt;
    }

    // --- Summary metrics -------------------------------------------------
    // Overshoot on the first step (0 -> 120 RPM).
    double peak_first = 0.0;
    for (const auto& s : history)
        if (s.t < 2.0 && s.rpm > peak_first) peak_first = s.rpm;
    const double overshoot_pct =
        std::fmax(0.0, (peak_first - 120.0) / 120.0 * 100.0);

    // Settling after the payload disturbance (last event, t = 4.0 s):
    // first time the speed enters and then stays within +/-2% of 150 RPM.
    double settling = -1.0;
    for (size_t i = 0; i < history.size(); ++i) {
        if (history[i].t < 4.0) continue;
        bool stays = true;
        for (size_t j = i; j < history.size(); ++j) {
            if (std::fabs(history[j].rpm - 150.0) > 0.02 * 150.0) {
                stays = false;
                break;
            }
        }
        if (stays) {
            settling = history[i].t - 4.0;
            break;
        }
    }

    // Steady-state error: mean error over the final 0.5 s.
    double err_sum = 0.0;
    int err_n = 0;
    for (const auto& s : history) {
        if (s.t >= kDuration - 0.5) {
            err_sum += (150.0 - s.rpm);
            ++err_n;
        }
    }
    const double ss_error = (err_n > 0) ? err_sum / err_n : 0.0;

    std::printf("\nSummary\n");
    std::printf("  Final target:            150.0 RPM (after 120 RPM step at "
                "t=2.0s and payload at t=4.0s)\n");
    if (settling >= 0.0)
        std::printf("  Settling time (post-payload, +/-2%% band): %.2f s\n",
                    settling);
    else
        std::printf("  Settling time (post-payload, +/-2%% band): not "
                    "settled within the run\n");
    std::printf("  Steady-state error (last 0.5 s): %+.2f RPM\n", ss_error);
    std::printf("  Overshoot (first step to 120 RPM): %.1f %% (peak %.1f "
                "RPM)\n",
                overshoot_pct, peak_first);
    std::printf("  Final actual speed: %.2f RPM\n", history.back().rpm);
    return 0;
}
