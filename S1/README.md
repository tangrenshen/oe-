# Supplementary Data S1

**Adaptive ship path following in curved, confined waterways using yaw-moment feedforward from a nonuniform current field**  
Yifan Li, Yixiong He, Kun Zhang, Wenlu Liu and Shuzhe Chen. Version 2026-11-21-r2.

This archive contains simulation outputs and analytic-field tests for the Yingongzhou case study. To reproduce the comparisons, extract the entire archive and run `python verify.py` from its top-level directory using Python 3.10 or newer. Verification uses the Python standard library and runs offline. It first checks the SHA-256 fingerprints of the distributed files. The results include recalculated metrics, agreement with the printed tables and the five paired ablation comparisons.

## Index by manuscript display

| Tables or figures | Input records | Calculations |
|---|---|---|
| Table 4 | `core/protocol.json`, `core/training_results.csv`, `core/frozen_selections.json` and corresponding controller settings | Re-select the baseline and proposed controller under each steering budget using both training passages per candidate. |
| Table 5 and Fig. 12 | `core/nominal_*.csv.gz`, `core/training_results.csv`, `core/earlier_passages.csv` | Compute the nominal errors for B1, B2 and the proposed controller from recorded samples. Compare steering demand using the simulator totals. |
| Table 6 and Fig. 13 | `core/targeted_followup_results.csv` and `core/earlier_passages.csv` | Compare controllers at four departure phases. |
| Table 7 and Fig. 14 | `checks/tradeoff_results.csv`, `checks/heldout_selection.json`, `checks/heldout_results.csv` | Repeat the earlier error-constrained selection and compare the selected settings at two departure phases. |
| Tables 8 and 9 | `paired/paired_frame_series.zip`, `paired/simulator_summaries.csv` and `paired/new_candidate_ablation.csv` | Recalculate five paired cases with five configurations per case. Test how moment feedforward and lookahead affect error and rudder travel. |
| Table 10 | `checks/analytic_anchors.csv`, `checks/analytic_queries.csv`, `checks/analytic_field_metrics.csv` | Compute reconstruction errors at 240 query locations for each of 18 field configurations. |
| Tables 11, D1 and D2 | `core/training_results.csv`, `core/targeted_followup_results.csv`, `core/dir20_supplement_results.csv` | Compare errors under speed scaling, direction bias, time delay and selected combined conditions, including the two training cases. |
| Table 12 and Figs. 15 and 16 | `checks/mechanism_results.csv` and `checks/figure16_series.zip` | Compare earlier single-parameter interventions and inspect the integral-state histories at speed factor 0.75. |
| Table C1 | `checks/randomized_runs.csv` and `checks/randomized_summary.json` | Summarise the earlier ten-seed tests for measurement noise and derivative perturbations and calculate paired differences. |
| Table C2 | `core/targeted_followup_results.csv` | Compare configurations on the same five noise seeds. |
| Fig. 11 | `checks/mesh_refinement.csv` | Compare tracking error and yaw moment over four hull discretisations. |

## Reading the results

B1 and B2 are ILOS baselines with the same crab-angle feedforward as the proposed controller. Their gains were selected under mean rudder-travel ceilings of 14,500° and 12,500°, respectively. Both ceilings select the same proposed configuration. The nominal and 0.85 speed-factor passages took part in selection. A passage can appear in more than one display. For example, the full-controller nominal passage is also the reference in the paired ablation tests.

Calculate relative changes from the archived numbers before rounding. The nominal RMS errors for the proposed controller, B1 and B2 round to 1.305, 1.392 and 1.434 m. The reductions relative to the two baselines round to 6.2% and 9.0%. Rudder travel uses the total accumulated by the simulator, starting with the initial rudder angle and sampling at 0.5 s. The exported rudder angle is rounded to 0.01°. For the individual runs, the original simulator total is recorded to 0.1°.

The package supports reanalysis of the reported simulations. Full vessel-dynamics reruns also require the simulator, its scenario inputs and the original electronic navigational charts. The charts are subject to the provider's redistribution terms. Tables 1–3 describe input scales and manoeuvring benchmarks and are outside the scope of this archive.
