# Yingongzhou current reconstruction and ship path-following data

This is the companion data repository for the manuscript *Depth-informed current-field reconstruction and adaptive ship path following in a curved river reach subject to tidal backwater* by Yifan Li, Yixiong He, Kun Zhang, and Wenlu Liu.

Repository: <https://github.com/tangrenshen/oe->. This companion-data snapshot is version `1.0-draft`; no DOI has been assigned. Cite the repository using `CITATION.cff`, preferably with the exact commit or an archived release DOI once available.

## Contents and provenance

- `tables/`: editable CSV versions of the eight reported tables. Their provenance is the retained Chinese manuscript and experiment summaries; they are not independent observations.
- `current_fields/`: reconstructed field exports retained with the original figures. These are model outputs, not measured currents. Speed in `speed_kn` is converted to m/s by multiplying by 0.514444.
- `recovered_runs/steady/g0` to `g3`: four full-route runs repeated on 13 October 2026 using the retained executable. Manuscript groups G1–G4 correspond to CLI groups 0–3. Each folder contains a summary, 0.5 s vessel record, and eight-cell force record.
- `model_source/`: preserved source files. `prepare_build.py` creates a separate build copy with a portable chart-database path; it leaves the archived model unchanged.
- `provenance/`: retained experiment notes and a numerical comparison of the recovered steady-field results.
- `manifest.csv`: file sizes and SHA-256 hashes.
- `scripts/verify_archive.py`: verifies the file manifest and the four steady-field summary results using Python's standard library.

The four recovered RMS errors are 46.14, 20.68, 46.18, and 2.29 m. Maximum errors also match the manuscript. These reruns are new records of the same simulation configuration; they are not recovered bytes of the original temporary logs.

## Archive completeness

Original time series for departure phases, degraded priors, derivative perturbations, command/gain sweeps, and cell refinement are absent from the retained temporary archive. Their reported summaries remain in the CSV tables and provenance notes. No individual Monte Carlo samples or noise-seed records have been invented. Regenerating all scenario records requires the model and input chart database and should precede a release described as a complete replication dataset.

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

## Reading the records

`sim_time` is the simulation clock, which starts near 33534 s in the steady case; subtract the first clock value for elapsed time. `lon`/`lat` are degrees, `stw_kn` and `sog_kn` are water-relative and ground speeds, `xte_m` is true cross-track error, and heading/rudder/current-direction columns are degrees. Roll and pitch moment columns are zero placeholders in this three-degree-of-freedom model.

Summary RMS, maximum error, and rudder total variation use every 0.1 s control step. Vessel CSVs are recorded every 0.5 s. RMS reconstructed from CSVs should be close; rudder total variation from a downsampled CSV need not equal the summary. The archived `phase` experiment notes also flag that older pre-refactor CSVs used `sog_kn` for water speed; the newly recovered records have separate columns.

## Deposit

This repository is intended for public access to the retained companion data. Its contents are not a complete archive of all manuscript experiment time series; the missing records are listed above. A GitHub URL is not a DOI. Mendeley Data or a Zenodo archive of a GitHub release can provide a persistent dataset DOI. Use `deposit_metadata.json` as a starting point and check the inventory before release. Select a data license at deposit; existing source-code license notices, including QCustomPlot's notices, remain applicable. Cite the actual repository URL and commit, and replace or supplement that citation with the real archived version and DOI when available.
