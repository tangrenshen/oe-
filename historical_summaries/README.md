# Historical experiment summary provenance

Nine selected tool outputs were recovered on 13 October 2026 from the retained experiment conversation archive. The outputs contain printed numerical results, not complete simulation records. The accompanying CSVs preserve the printed precision; unavailable fields are blank. `source_provenance.json` retains the original timestamps, archive line numbers, tool identifiers, output hashes, and analysis commands. Those commands are stored as provenance and were not re-executed during recovery. The full conversation is not distributed.

The nine CSVs cover departure phases, current-prior conditions, fair noisy comparisons, individual noise seeds, individual paired derivative samples, cell refinement, command alignment, gain sweeps, and the five-case rudder-margin experiment. The derivative table contains sixteen seeds for both G2 and G4. `validation.json` records recalculated distribution statistics and checks against manuscript tables 5–7.

`historical_outputs/prior_initial.txt` includes an early noise comparison that did not inject noise into all groups fairly. Its obsolete noise rows are not used in the final parsed summaries; the later `fair_noise.txt` output supplies those results. The original rudder-margin wrapper reported exit code 255 despite printing five case summaries. This status is preserved rather than interpreted as a successful historical batch exit.

Newly repeated cases are stored in `../recovered_runs/representative/`, with complete 0.5 s states and eight-cell force records. All fifteen new cases exited with code 0 and match the relevant historical RMS and maximum errors at their reported precision. New records must not be described as the original historical log files.
