# Controller selection and route simulations

Run `python verify.py` in this directory using Python 3.8 or later. The script checks the six settings in Table 4, all 440 numerical entries in Tables 5, 6, 11, C2, D1 and D2, and the main nominal, direction-bias and paired-noise comparisons. Its inputs are the files in this directory. All results describe simulated upstream passages.

## Files and coverage

| Files | Contents | Use in the manuscript |
|---|---|---|
| `protocol.json`, `training_results.csv`, `frozen_selections.json` | Candidate lists, training conditions, all 32 selection passages and the four archived selections across two budgets and two designs | Table 4 and candidate selection; nominal and speed-factor 0.85 results in Tables 5, 11, D1 and D2 |
| `followup_protocol.json`, `targeted_followup_results.csv` | Eight fixed configurations in each of 17 follow-up conditions, including four departure phases and five paired noise seeds | Tables 6, 11, C2, D1 and D2 |
| `dir20_supplement_results.csv` | Sixteen additional runs for the same eight configurations at direction errors of −20° and +20° | Tables 11, D1 and D2 |
| `earlier_controller_settings.json`, `earlier_passages.csv` | Historical settings and six full-precision run summaries for conventional ILOS and earlier ILOS with crab-angle compensation | Historical comparison rows in Tables 4–6 |
| `run_configuration_evidence.json` | Applied settings from the four nominal run manifests in the current selection grid that appear in Table 4, along with lookahead bounds and the default integral update speed read from model source | Table 4 |
| `nominal_B1.csv.gz`, `nominal_B2.csv.gz`, `nominal_Proposed.csv.gz` | All original samples of simulation time, signed true cross-track error and rudder angle for the three selected nominal runs | Independent recomputation of the abstract RMS errors and five nominal tracking, duration and steering measures |
| `reported_values.csv`, `reported_settings.csv` | Machine-readable transcription of the displayed manuscript values, used as comparison targets | Tables 4, 5, 6, 11, C2, D1 and D2 |
| `provenance.csv` | Source basenames, SHA-256 fingerprints of the original files and a description of each extraction | Source provenance |

The source experiments use `baseline` for ILOS with crab-angle feedforward and `shared` for the proposed controller using the same prior for crab-angle and yaw-moment feedforward. B1 denotes `base_kp15.0_ki0p02`, selected under the 14,500° mean-travel ceiling. B2 denotes `base_kp12.0_ki0p02`, selected under 12,500°. Both budgets selected `prop_kp12.0_rate30.0`. The slow-update comparator is `prop_kp12.0_rate1.0` and the earlier proposed controller is `prop_kp15.0_rate1.0`. The historical file uses `ilos` for conventional ILOS and `ilos_crab` for its earlier crab-compensated counterpart. Its `dir0` is the nominal steady-current scenario.

## Fields and calculations

Each record in the three current experiment CSV files represents one route simulation. The pair `candidate_id`, `scenario` uniquely identifies a run across these files. The `stage` values `train` and `val` identify selection and subsequent fixed-configuration comparisons. The follow-up protocol and `history` field record prior use of individual scenario families. A `reused` flag records reuse of an existing run where present. Numeric fields retain their source precision.

| Field | Definition |
|---|---|
| `kp`, `ti`, `ki_or_rate` | Heading proportional gain, integral time (s), and baseline integral coefficient (s⁻¹) or proposed integral update speed (m/s) |
| `arrived`, `transit_s` | Completion flag (1 for arrival) and transit time in seconds |
| `rms_m`, `max_m` | Root mean square and maximum absolute true cross-track error, both in metres |
| `mean_abs_rudder_deg`, `near_limit_pct` | Mean absolute rudder angle in degrees and percentage of frames with absolute rudder angle at least 34° |
| `rudder_tv_deg` | Total absolute change in rudder angle, accumulated by the simulator at 0.5 s intervals and saved in the run summary to 0.1°. The angle in the exported trace has 0.01° precision. The verifier uses the run-summary value for travel. |
| `integral_max_abs`, `integral_cap_pct` | Maximum absolute controller integral state as recorded in the source run summary, and the percentage of samples that reach the implementation's cap criterion |
| `sim_time`, `xte_m`, `rudder_deg` | Absolute simulated time (s), signed true cross-track error (m) and rudder angle (degrees) at 0.5 s intervals. For Fig. 12, subtract the first recorded time from each time and divide by 60 for elapsed minutes on the plotting axis. |

Scenarios named `gainNNN` scale the online current speed, such that `gain075` means 0.75. `dirNN` and `dirmNN` rotate the online current by +NN° and −NN°. `lagNN` supplies the online current with NN seconds of delay. `phaseNN` sets the departure tide phase in degrees. `phase135_lag40` combines a departure phase of 135° with a 40 s delay. `noise2m_sNNN` is a temporally correlated 2 m cross-track measurement-noise case identified by its seed. The recorded error measure uses true cross-track error. For the appendix noise means, each metric is averaged over the five passage-level metrics with equal weights. Paired differences compare controllers on the same seed.

The selection protocol requires both selection passages to finish under a mean travel ceiling. Within each feasible controller family, candidates rank first on worst-case RMS error, then mean RMS error, mean travel and finally candidate ID. Nominal steady current is one of the two selection conditions.

Relative nominal RMS reductions use the full-precision values computed from the compressed sample series. The reductions are 6.249108201% against B1 and 9.035185252% against B2, reported as 6.2% and 9.0% to one decimal place. Compute percentage changes before rounding the input errors for display.

The three nominal sample series support frame-level recalculation of RMS and maximum error, mean absolute rudder, near-limit use and duration. The other comparisons use simulator run summaries. Re-executing the vessel dynamics would additionally require the simulator and its scenario input files.
