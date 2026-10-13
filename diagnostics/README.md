# Representative-run diagnostics

The charts and metrics are derived from the fifteen representative reruns and the nominal G4 steady run. `scripts/analyse_representative_runs.py` regenerates these outputs with NumPy, SciPy, and Matplotlib; Python 3.11 or later is required by the companion rerun runner. No chart database or model execution is needed to analyse the supplied records.

`seed7_acute_bend.png` and its PDF compare the nominal model with the same derivative-seed-7 perturbation at propulsion commands 94.5, 100, and 105. The shaded region is fixed planned-route chainage 35.3–37.4 km. The panels show true cross-track error, recorded rudder angle, and water-relative speed. Command settings are the software's legacy interface values, not actual shaft rpm.

`current_prior_comparison.png` and its PDF compare magnitude errors of ±30%, direction biases of ±15 degrees, a 60 s spatial-prior lag, and combined prior and measurement errors. The hydrodynamic field is unchanged; the current supplied to the controller is perturbed.

`diagnostic_metrics.json` distinguishes a fixed bend window from samples selected by absolute error above 20 m anywhere on the route. Near-limit rudder use means an absolute recorded rudder angle of at least 34 degrees. The recorded crab angle is heading minus course over ground, not the feedforward command. In particular, the historical 80%/55% percentages used error-conditioned samples, whereas fixed-window fractions are 87.6%/25.5% for commands 94.5/105.

`derived/` contains planned-route chainage and total distributed load corrections. Chainage is the nearest projection of each logged position onto `planned_route.csv` using the shared scaled-Mercator projection. Forces are summed over all eight cells of the 4-by-2 division; the yaw correction is the sum of x_i dY_i − y_i dX_i. Local longitudinal cell coordinates are 60, 20, −20, and −60 m; transverse coordinates are ±7.25 m. These are postprocessed values, not a separate hydrodynamic solve.

Integration and control use 0.1 s steps; CSV and summary samples use 0.5 s frames. The terminal arrival frame is excluded from summary accumulation, and CSV rudder angles are rounded to two decimals. Metrics computed from all rounded records therefore need not equal the higher-precision summary values exactly. The raw rerun summaries remain the reported experiment results.
