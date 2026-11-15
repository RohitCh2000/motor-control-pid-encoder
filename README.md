# ⚙️ Motor Control — PID & Encoder Feedback

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue) ![PID](https://img.shields.io/badge/control-PID-brightgreen) ![Build](https://img.shields.io/badge/build-passing-success)

Embedded C++ motor-control and speed-regulation module using **PWM, PID and wheel-encoder feedback**, with a full DC-motor plant simulation — including a target step change and a payload disturbance, just like a real AMR drive wheel.

*Personal project based on professional AMR motor-control experience at Honeywell — no employer code.*

## ✨ Highlights
- Steady-state error **+0.56 RPM** on a 150 RPM target (< 0.4%)
- Recovers from a payload disturbance in **0.74 s** (±2% band)
- Anti-windup PID, 1024 CPR encoder model with quantisation, 100 Hz control loop

## Code, Build & Sample Output

Real, runnable code — not just a description:

- `src/pid.h` / `src/pid.cpp` — a discrete PID controller class
  (Kp/Ki/Kd, output clamp 0–100% PWM, and anti-windup via integral
  clamping plus conditional integration when the output is saturated).
- `src/main.cpp` — a self-contained C++17 simulation (standard library
  only) of a DC motor: first-order lag + inertia plant, a 1024 counts/rev
  quadrature encoder with quantization, and the PID loop driving it.
  Scenario: target steps to **120 RPM** at t=0, steps to **150 RPM** at
  t=2.0 s, then a **payload disturbance** at t=4.0 s (lag 0.35 → 0.60 s
  plus a 30 RPM load drag) that the loop must recover from.
  Gains used: Kp = 1.8, Ki = 3.0, Kd = 0.015, loop at 100 Hz.

### Build & run

```bash
make            # or: g++ -std=c++17 -Wall -Wextra -O2 src/main.cpp src/pid.cpp -o motor_pid_sim
./motor_pid_sim # or: make run
```

A `CMakeLists.txt` is also provided (`cmake -B build && cmake --build build`).
Builds clean with `-Wall -Wextra`, no warnings. Output is deterministic.

### Sample output (actual run)

```
Motor PID simulation: dt=0.01s, encoder 1024 CPR, PWM 0-100%
  t[s]    target    actual     PWM%   enc_counts
           [RPM]     [RPM]                      
  0.00     120.0      0.00    100.0            0
  0.20     120.0     90.28     63.9          186
  0.40     120.0    104.10     50.0          526
  0.60     120.0    108.35     55.0          889
  0.80     120.0    111.38     52.7         1265
  1.00     120.0    113.60     56.0         1649
  1.20     120.0    115.14     57.5         2040
  1.40     120.0    116.57     53.4         2436
  1.60     120.0    117.13     53.4         2836
  1.80     120.0    118.22     55.5         3237
  2.00     150.0    118.64    100.0         3641
  2.20     150.0    144.58     67.0         4110
  2.40     150.0    146.55     67.8         4608
  2.60     150.0    147.54     69.1         5110
  2.80     150.0    148.15     69.2         5615
  3.00     150.0    148.84     64.2         6122
  3.20     150.0    149.18     64.8         6630
  3.40     150.0    149.46     65.3         7139
  3.60     150.0    149.58     65.1         7649
  3.80     150.0    149.45     71.4         8159
  4.00     150.0    149.50     71.1         8670
  4.20     150.0    144.95     76.4         9170
  4.40     150.0    145.24     79.2         9665
  4.60     150.0    146.54     81.8        10162
  4.80     150.0    147.31     82.4        10664
  5.00     150.0    148.12     83.6        11168
  5.20     150.0    148.51     83.5        11675
  5.40     150.0    149.14     78.7        12183
  5.60     150.0    149.21     84.6        12692
  5.80     150.0    149.40     84.8        13202
  6.00     150.0    149.68     78.9        13713

Summary
  Final target:            150.0 RPM (after 120 RPM step at t=2.0s and payload at t=4.0s)
  Settling time (post-payload, +/-2% band): 0.74 s
  Steady-state error (last 0.5 s): +0.56 RPM
  Overshoot (first step to 120 RPM): 0.0 % (peak 118.7 RPM)
  Final actual speed: 149.68 RPM
```

### Results

- **Steady-state error: +0.56 RPM** on a 150 RPM target (last 0.5 s mean) — under 0.4%.
- **Settling time after the payload disturbance: 0.74 s** to enter and stay in the ±2% band; the payload dips speed to ~145 RPM and the loop recovers by raising PWM from ~71% to ~83%.
- **Overshoot: 0.0%** on the initial step to 120 RPM (peak 118.7 RPM in the first segment, monotonic approach).
- Full captured output is also saved in `sample_output.txt`.


---
*Created by Rohith Reddy Chinthala — Embedded Software Engineer at Honeywell, Saint Louis, MO.*
