# Experimental programme (blueprint §11)

| Test | Status | Where |
|---|---|---|
| Agent docking, 100 cycles, >= 95 % success, no undetected false lock | hardware pending; FSM logic verified (8 scenarios) | `docking_log_template.csv` |
| Task reassignment on agent failure | sim PASS | `tests/test_blueprint_section11.py` |
| Communication loss -> agent stops independently | sim PASS | same |
| Battery-aware planning respects return reserve | sim PASS (agent returns to charge) | same |
| Measurement fault -> evidence flagged, dependants reviewed | sim PASS | same |
| Research-task quality vs strategy | sim: evidence-aware 0 % wrong vs 21 % (reassign only) in drift runs | `05_simulation/python/results/summary.md` |

Before the hardware tests, define the environment, sample size and failure criteria in this folder.
First physical test: print the five clearance coupons (`01_mechanical/docking_system/test_coupons/`)
and record which clearance gives reliable entry with the real agent.
