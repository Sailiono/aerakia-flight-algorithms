# G0 zero-noise trajectory screen

The trajectory screen asks a narrow structural question: can the current
analyzer see five local target directions when generated measurement noise is
set to zero? It is not an estimator-tuning result, a noisy true-positive rate,
or a flight-readiness claim.

At 100 Hz, all four frozen trajectories remained numerically healthy with zero
navigation recoveries:

| Trajectory | Maximum rank | Structural-ready windows | Meaning |
| --- | ---: | ---: | --- |
| `bias_cv_hover_axis_pulses` | 3 | 0 | insufficient fixed-yaw excitation |
| `bias_cv_takeoff_box_land` | 5 | 58 | eligible structural candidate |
| `bias_cv_yaw_quadrant_hover` | 5 | 67 | strongest structural candidate |
| `bias_cv_early_transition_s_curve` | 5 | transient | final-window coverage fails |

The two rank-5 trajectories are candidates for a future clean v2 protocol. They
do not show that the present estimator converges from nonzero bias: the existing
cross-validation protocol already contains the first two maneuvers and still
has direction-sensitive convergence misses. The next algorithm experiment must
therefore change the information model or estimator strategy, not merely add
more maneuver names.

The machine-readable result is
[`validation/public/g0_trajectory_screen.json`](../validation/public/g0_trajectory_screen.json).
