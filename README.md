# Yingongzhou current reconstruction and ship path-following data

This is the companion data repository for the manuscript *Depth-informed current-field reconstruction and adaptive ship path following in a curved river reach subject to tidal backwater* by Yifan Li, Yixiong He, Kun Zhang, and Wenlu Liu.

Repository: <https://github.com/tangrenshen/oe->. This companion-data snapshot is version `1.1-draft`; no DOI has been assigned. Cite the repository using `CITATION.cff`, preferably with the exact commit or an archived release DOI once available.

## Contents and provenance

- `tables/`: editable CSV versions of the eight reported tables. Their provenance is the retained Chinese manuscript and experiment summaries; they are not independent observations.
- `current_fields/`: reconstructed field exports retained with the original figures. These are model outputs, not measured currents. Speed in `speed_kn` is converted to m/s by multiplying by 0.514444.
- `recovered_runs/steady/g0` to `g3`: four full-route runs repeated on 13 October 2026 using the retained executable. Manuscript groups G1–G4 correspond to CLI groups 0–3. Each folder contains a summary, 0.5 s vessel record, and eight-cell force record.
- `recovered_runs/representative/`: fifteen additional full-route runs repeated on the same date: the G2/G4 derivative-seed-7 pair, two higher-command G4 cases with that parameter set, six degraded-current-prior cases, and five measurement-noise seeds. State and eight-cell force records, commands, hashes, and historical comparisons are retained separately from the historical summaries.
- `historical_summaries/`: nine selected historical tool outputs and nine CSV tables extracted from a retained conversation archive. The derivative CSV contains all 32 paired G2/G4 sample results; the noise CSV contains all five seeds. These are printed summaries at their original precision, not original time series. Provenance identifies source timestamps, archive line numbers, tool identifiers, and the stored analysis commands; no complete conversation is distributed.
- `diagnostics/`: route-projected loads, metric definitions, and charts derived from the repeated representative runs. Distributed load corrections are reconstructed from all eight cells of the 4-by-2 division; the fixed eight-slot log format does not capture every cell of finer divisions.
- `model_source/`: preserved source files. `prepare_build.py` creates a separate build copy with a portable chart-database path; it leaves the archived model unchanged.
- `provenance/`: retained experiment notes and a numerical comparison of the recovered steady-field results.
- `manifest.csv`: file sizes and SHA-256 hashes.
- `scripts/verify_archive.py`: verifies the file manifest, four steady-field summaries, fifteen representative reruns, and provenance hashes using Python's standard library.

The four recovered RMS errors are 46.14, 20.68, 46.18, and 2.29 m. All fifteen additional reruns also match their historical RMS and maximum-error values at the reported precision. These reruns are new records of the same simulation configurations; they are not recovered bytes of the original temporary logs. The five repeated noise seeds have a mean RMS error of 2.75 m.

## Archive completeness

Historical summary outputs have been recovered for departure phases, degraded priors, fair noise comparisons, all sixteen paired derivative parameter sets, command/gain sweeps, and cell refinement. Their values can be checked directly against the selected historical outputs rather than only against the manuscript tables. The first noise comparison applied noise unfairly to some groups; its obsolete noise rows are excluded from the final CSV summaries in favour of the later fair comparison.

The four steady runs and fifteen representative cases now have newly generated time series. Time series remain unavailable for the other fifteen derivative parameter sets, the four-group/four-phase suite, the baseline-group noise cases, cell refinement, and the complete gain/command sweeps. No missing records or unprinted parameters have been invented. This is a verified partial replication archive, not a complete archive of every manuscript experiment.

The ENC database is not bundled in this release. Possession of a chart file does not establish its redistribution terms; those terms have not been verified. A compatible `Yangtze_Mile235_270.db` is required to rerun the simulation. Water-level screenshots used for the backwater estimate are described in the manuscript; no independent current observations are supplied.

## Reproduction

The original project uses Qt 5.14.2 and MinGW 7.3 (32 bit). Prepare the build copy:

```powershell
python scripts/prepare_build.py
$env:YINGONGZHOU_CHART_DB = 'ABSOLUTE_PATH_TO/Yangtze_Mile235_270.db'
cd build_source
qmake xjp.pro CONFIG+=release
mingw32-make -f Makefile.Release
release/xjp_release.exe --batch --group 3 --out ../runs/steady/g3 --max-time 16000
```

To check the supplied files without compiling Qt or obtaining the chart database:

```powershell
python scripts/verify_archive.py
```

Use `--group 0`, `1`, `2`, or `3` for manuscript G1–G4. The source may expose diagnostic APIs beyond those enabled in the paper; the manuscript describes only active features.

For departure phases, add `--backwater 0.40 --celerity 15 --phase 0` and repeat for 90, 180, and 270 degrees. For current-prior errors use `--ff-gain 0.7` or `1.3`, `--ff-bias 15` or `-15`, and `--ff-lag 60`. Noise uses `--xte-noise 2 --noise-seed N` for N = 1…5. The combined case uses gain 0.7, bias +15 degrees, lag 60 s, and correlated noise.

Derivative cases use `--deriv-perturb 0.20 --deriv-seed N` for N = 1…16, with groups 1 and 3 using the same seeds. Cell refinement uses `--fsi-split 4`, `8`, `16`, and `32` for groups 2 and 3. Command tests use `--base-rpm 94.5` or `--own-rpm 100`; these flags control legacy command settings, not actual shaft rpm. Gain sweeps use the baseline gain flag shown in `main.cpp`.

For the seed-7 margin comparison, use group 3 with `--deriv-perturb 0.20 --deriv-seed 7`, then repeat with `--own-rpm 100` and `--own-rpm 105`. Each repeated case's `run_metadata.json` preserves the exact argument list used on the original machine. The historical wrapper for the five-case margin summary reported exit code 255 while printing results; that historical status is preserved. The new representative runs all report exit code 0 and successful arrival.

To repeat the fifteen-case suite with Python 3.11 or later, point the supplied runner at a compiled executable and the private chart database, using a fresh output directory:

```powershell
python scripts/repeat_representative_cases.py --executable ABSOLUTE_PATH_TO/xjp_release.exe --chart ABSOLUTE_PATH_TO/Yangtze_Mile235_270.db --output ABSOLUTE_PATH_TO/new_reruns
```

To regenerate the included diagnostic charts without running the model, install NumPy, SciPy, and Matplotlib and run `python scripts/analyse_representative_runs.py`.

## Reading the records

`sim_time` is the simulation clock, which starts near 33534 s in the steady case; subtract the first clock value for elapsed time. `lon`/`lat` are degrees, `stw_kn` and `sog_kn` are water-relative and ground speeds, `xte_m` is true cross-track error, and heading/rudder/current-direction columns are degrees. Roll and pitch moment columns are zero placeholders in this three-degree-of-freedom model.

Integration and control use 0.1 s steps. Summary RMS, maximum error, and rudder total variation are accumulated once per 0.5 s frame after five control steps; vessel CSVs use the same frame interval. The terminal arrival frame is written to CSV but excluded from summary accumulation, and recorded rudder angles are rounded to two decimals. Exact metric reconstruction must account for these details. The archived `phase` experiment notes also flag that older pre-refactor CSVs used `sog_kn` for water speed; the newly repeated records have separate columns.

The diagnostic crab angle is the recorded heading minus course over ground; it is not the controller's commanded feedforward angle. Fixed-bend diagnostics use planned-route chainage 35.3–37.4 km. The historical 80%/55% near-limit rudder percentages instead used all samples with absolute error above 20 m; these conditionally selected samples differ in location between cases. The fixed-bend comparison gives 87.6%/25.5% for commands 94.5/105, using a 34-degree near-limit threshold. Both definitions are retained in `diagnostic_metrics.json` to avoid mixing them.

## Deposit

This repository is intended for public access to the retained companion data. Its contents are not a complete archive of all manuscript experiment time series; the missing records are listed above. A GitHub URL is not a DOI. Mendeley Data or a Zenodo archive of a GitHub release can provide a persistent dataset DOI. Use `deposit_metadata.json` as a starting point and check the inventory before release. Select a data license at deposit; existing source-code license notices, including QCustomPlot's notices, remain applicable. Cite the actual repository URL and commit, and replace or supplement that citation with the real archived version and DOI when available.
