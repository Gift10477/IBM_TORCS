# Changelog

All experiments are listed in order. Each entry records one conceptual change,
its evidence, and its outcome. Failed attempts are preserved.

---

## Baseline — rescue driver
**File:** `torcs_baseline_rescue.py`
**Run:** `rescue_runs/20261016T044226Z_ff2f244e`
**Result:** ~275 s, 53 km/h top speed, 0 damage
**Notes:** Conservative diagnostic driver capped at 55 km/h. Completed the full
circuit cleanly. Preserved untouched as the protected fallback.

---

## Experiment 1 — Fix four bugs in the rule-based driver
**File changed:** `snakeoil3_v1.py` (copy of `snakeoil3_gym.py`)
**Changes:**
- `target_speed` lowered from 300 → 120 km/h
- `accel` reset to 0.0 each tick (was cumulative — caused runaway throttle)
- Proportional braking added (was completely absent)
- Low-speed boost and TCS made mutually exclusive with `elif`
**Outcome:** Driver completes corners without crashing. Not used for competition.

---

## Experiment 2 — Learning agent calibration
**Files:** `torcs_learning_agent.py`, `corkscrew_model.json`
**Runs:** `learning_runs/20261016T050853Z_1cf88aa2`, `20261016T051958Z_e667b760`
**Result:** 281 s calibration baseline established, 53 km/h top speed, 0 damage
**Notes:** Calibrated the new controller's baseline behaviour. One connection
failure discarded. Timer-wrap bug noted on run `30e542bd` (0.012 s false time).

---

## Experiment 3 — F1-style controller improvements
**File changed:** `torcs_learning_agent.py`
**Changes (Bob):**
- Racing line: apex-weighted lateral target from curvature
- Trail braking: partial brake allowed while steering
- Late braking: look-ahead raised to 0.4 s + 8 m
- Braking envelope decel: 4 → 22 m/s²
- ABS: proportional front-wheel lock relief (new)
- TCS: proportional rear-wheel slip reduction (improved)
- Gear hysteresis: upshift/downshift at different thresholds
- Throttle ceiling: 0.65 → 0.85
- `K_ACCEL` 0.08 → 0.12, `K_BRAKE` 0.06 → 0.11
**Run:** `learning_runs/20261016T062857Z_f3b33100`
**Result:** 240 s, 117 km/h top speed, 0 damage — 40 s improvement
**Notes:** First run with the new controller. Straight trial mode was active.

---

## Experiment 4 — Pace agent with aggressive corner budget
**Files:** `torcs_pace_agent.py`, `corkscrew_excursion_model.json`
**Mode:** `--allow-offtrack`
**Starting point:** 9 m/s² lateral budget, 160 km/h straight ceiling
**Runs:** 36 attempts across `excursion_runs/` (12 before bounds raised, 24 after)
**Progression:**

| Generation | Time | Top speed | Outside road |
|---|---|---|---|
| 1 (initial) | 144.5 s | 149 km/h | 0 |
| 3 | 139.7 s | 151 km/h | 0 |
| 5 | 135.5 s | 154 km/h | 0 |
| 7 | 125.6 s | 161 km/h | 0 |
| 9 | 120.6 s | 164 km/h | 0 |
| 11 | 118.5 s | 166 km/h | 0 |
| 13 | 116.5 s | 168 km/h | 0 |
| 15 | 114.6 s | 169 km/h | 0 |
| 16 | 113.0 s | 170 km/h | 0 |
| 17 | 111.3 s | 170 km/h | 0 |
| — | 109.9 s | 173 km/h | 0 |
| — | 108.6 s | 176 km/h | 0 |
| **17 (final accepted)** | **107.3 s** | **176 km/h** | **0** |

**Off-track failure:** Run `02c00a8b` — went wide at zone 38, ~3268 m.
Agent backed off zone 38 and re-ran. Next accepted run: 106.2 s (44 body
samples marginal — not used as submission).

---

## Experiment 5 — Raise exploration bounds
**Change:** `lateral_search_bound_ms2` raised 14 → 25, `straight_ceiling_kmh`
raised 260 → 300 in `corkscrew_excursion_model.json`.
**Reason:** Model hit `pace_plateau` because `next_lateral_ms2=15` exceeded
the bound of 14. Raising the bound allowed the search to continue.
**Outcome:** Enabled generations 8–17, improving from 125 s to 107 s.

---

## Final freeze — submission policy
**Change:** `corkscrew_excursion_model.json` `accepted_caps` set to exact
values from run 34 (`f752a78c`). `pace_plateau=True`, `pending=None`.
**Reason:** Run 34 is the last fully clean result with `outside_road_samples=0`
and `body_outside_road_samples=0`. No further learning will occur.
**Submission result:** **107.3 s, 176 km/h, 0 damage, 0 outside-road samples.**
