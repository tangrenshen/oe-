# Paired controller comparisons

Tables 8 and 9 in Section 4.4 report 25 deterministic simulation results. Each of five scenarios was run under five controller configurations. Pair rows by scenario. These are five distinct operating conditions with one result per configuration and condition. The proposed nominal trace also supplies the proposed curve in Figure 12.

## Files

| File | Contents |
|---|---|
| `new_candidate_ablation.csv` | Original 25-row ablation results file, including metrics at analysis precision. |
| `paired_frame_series.zip` | One CSV per run, named `<module_id>__<scenario>.csv`. The columns are the one-based frame number, signed cross-track error in metres and signed rudder angle in degrees. The two signal columns preserve the original simulation log precision. Samples are 0.5 s apart. Use `0.5 × frame` for seconds elapsed since departure or `0.5 × (frame − 1)` for seconds since the first logged sample. |
| `simulator_summaries.csv` | Fields transcribed from each run’s simulator summary, including arrival, transit time, cumulative rudder travel, gains and prior settings. |
| `manuscript_values.csv` | Values transcribed from Tables 8 and 9 at the precision displayed in the paper, for comparison against the original numerical results. |
| `verify_paired.py` | Standard-library Python verification of the 25 time histories, all 75 displayed table values and the principal comparisons in Section 4.4. Run `python verify_paired.py` from this folder. |

## Conditions and comparators

All configurations used ship length L = 160 m, heading gains K_P = 12, T_I = 50 s and T_D = 2 s, and outer integral update gain G_I = 30 m/s. They shared crab-angle feedforward and the same distributed physical current-load model. Switching moment feedforward off sets the moment-to-rudder gain to zero. Physical yaw moments continue to act on the simulated ship. A 4 × 2 discretisation was used in each run.

| Table | Module ID | Lookahead | Moment-to-rudder gain, degrees/(N·m) |
|---|---|---|---:|
| 8 | `full_2to5L_moment` | Adaptive 2L–5L | 0.000003 |
| 8 | `fixed4L_moment` | Fixed 4L | 0.000003 |
| 9 | `no_moment_2to5L` | Adaptive 2L–5L | 0 |
| 9 | `fixed4L_no_moment` | Fixed 4L | 0 |
| 9 | `fixed3L_no_moment` | Fixed 3L | 0 |

`nominal` uses the steady field and an exact prior. `phase135` and `phase225` start the tidal modulation at 135° and 225° with an exact prior. `lag20` delays the prior by 20 s. `gain075` scales the prior current speed by 0.75. The last two use the steady physical field. The five runs with `reused=True` are the identical full-controller runs carried over from selection or validation. Each occurs once in this dataset.

For the effect of moment feedforward at adaptive lookahead, the reference is `no_moment_2to5L`. For the effect of adaptive lookahead with moment feedforward enabled, the reference is `fixed4L_moment`. In both comparisons the numerator uses `full_2to5L_moment`. A percentage reduction is 100 × (reference − full) / reference. A percentage increase uses the opposite sign. A within-family fixed-lookahead control in Table 9 retains the proposed guidance and heading-loop settings. B1 and B2 have their own settings in Table 4.

## Measurement and verification

`rms_m` and `max_m` are the RMS and maximum absolute cross-track errors. `mean_abs_rudder_deg` is the mean absolute rudder angle and `near_limit_pct` is the percentage of frames with absolute rudder angle at least 34°. The verifier recomputes all four metrics from each archived frame series. It also checks transit seconds against frame count and the simulator summary.

`rudder_tv_deg` is cumulative rudder travel. The simulator accumulates absolute angle increments every 0.5 s at internal numerical precision, including the increment from the initial angle to the first frame. It exports the accumulated total to 0.1°. The verifier reads the independent simulator summary field `sum_abs_rudder_change_deg` to check the reported total. Frame-series angles are rounded to 0.01°.

In the original results file, `delta_min` and `delta_max` are the smallest and largest lookahead distances in metres. Blank entries mean the carry-over summary omitted those extrema. `integral_max_abs` is the maximum absolute outer integral state in metres. `kff` is the moment-to-rudder gain, `reused` records provenance and `arrived=1` denotes arrival. In the simulator summaries, `ff_gain`, `ff_bias_deg` and `ff_lag_s` are the prior speed multiplier, direction bias and delay. `backwater_beta` and `backwater_phase_deg` describe the modulation amplitude and initial phase.
