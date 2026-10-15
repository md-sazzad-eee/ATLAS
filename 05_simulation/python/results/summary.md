| subset | strategy | n | accuracy | runs w/ wrong concl. | missing | time (min) | re-measurements | energy (Wh) |
|---|---|---|---|---|---|---|---|---|
| all runs | fixed | 300 | 0.923 | 14.0% | 0.23 | 13.9 | 0.00 | 1.01 |
| all runs | reassign | 300 | 0.954 | 11.0% | 0.12 | 17.6 | 0.00 | 1.26 |
| all runs | full_redo | 300 | 0.973 | 0.3% | 0.16 | 20.8 | 1.43 | 1.49 |
| all runs | evidence | 300 | 0.977 | 0.3% | 0.14 | 19.1 | 0.66 | 1.37 |
| sensor drift drawn | fixed | 150 | 0.892 | 28.0% | 0.19 | 14.0 | 0.00 | 1.02 |
| sensor drift drawn | reassign | 150 | 0.937 | 21.3% | 0.09 | 18.2 | 0.00 | 1.31 |
| sensor drift drawn | full_redo | 150 | 0.974 | 0.0% | 0.15 | 24.7 | 2.87 | 1.77 |
| sensor drift drawn | evidence | 150 | 0.981 | 0.0% | 0.11 | 21.3 | 1.32 | 1.53 |
| agent failure drawn | fixed | 67 | 0.803 | 9.0% | 1.04 | 12.6 | 0.00 | 0.87 |
| agent failure drawn | reassign | 67 | 0.893 | 6.0% | 0.55 | 16.5 | 0.00 | 1.06 |
| agent failure drawn | full_redo | 67 | 0.883 | 0.0% | 0.70 | 18.5 | 1.04 | 1.14 |
| agent failure drawn | evidence | 67 | 0.898 | 0.0% | 0.61 | 17.4 | 0.48 | 1.10 |
| no fault | fixed | 82 | 1.000 | 0.0% | 0.00 | 13.8 | 0.00 | 1.02 |
| no fault | reassign | 82 | 0.998 | 1.2% | 0.00 | 16.8 | 0.00 | 1.24 |
| no fault | full_redo | 82 | 0.998 | 1.2% | 0.00 | 16.8 | 0.00 | 1.24 |
| no fault | evidence | 82 | 0.998 | 1.2% | 0.00 | 16.8 | 0.00 | 1.24 |

Safety invariants over all runs: carrier moved with unlatched agent + reserve violations = 0
