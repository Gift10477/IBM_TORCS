# IBM AI Racing League — Submission

## Competitor
giftg — Strathmore University Hackathon

---

## How to run the competition driver

**Requirements:** Python 3.8 or later, standard library only (no pip installs).

**TORCS setup:**
1. Open TORCS on Windows.
2. Configure: Corkscrew track, one-lap race, standing start, car `car1-ow1`, damage enabled.
3. Add exactly ONE external server driver.
4. Click New Race and wait for the waiting screen.

**Run the driver** from the `gym_torcs` folder:
```powershell
py torcs_pace_agent.py --allow-offtrack --evaluate
```

The driver connects automatically, completes one lap, and saves a summary to `excursion_runs/<run_id>/`.

---

## Best confirmed result

| Metric | Value |
|---|---|
| **Lap time (simulator reported)** | **107.3 seconds (1 min 47 s)** |
| Damage | 0 |
| Outside-road samples (SCR rays) | 0 |
| Top speed | 176 km/h |
| Full circuit covered | Yes |
| Run ID | `20261016T151016Z_f752a78c` |
| Evidence file | `evidence/best_run_summary.json` |

The lap time shown above is from the simulator's internal sensor timer.
The TORCS-displayed final time on screen is the authoritative competition figure —
record that value when you run the driver for your submission.

---

## Baseline for comparison

| Metric | Rescue baseline | Submitted driver |
|---|---|---|
| Lap time | ~275 s (4 min 35 s) | **107 s (1 min 47 s)** |
| Top speed | 53 km/h | 176 km/h |
| Damage | 0 | 0 |
| Run ID | `20261016T044226Z_ff2f244e` | `20261016T151016Z_f752a78c` |

That is a **168-second improvement** through measured, evidence-backed experiments.

---

## Files required to run

| File | Purpose |
|---|---|
| `torcs_pace_agent.py` | Main competition driver |
| `torcs_pace_reference.py` | Core episode runner and model helpers |
| `torcs_adaptive_agent.py` | Telemetry observer (imported by pace agent) |
| `torcs_learning_reference.py` | Shared geometry and connection utilities |
| `torcs_baseline_rescue.py` | Packet parser, encoder, sensor constants |
| `corkscrew_excursion_model.json` | Frozen speed policy (run-34 caps, 722 bins) |

No other files are needed to run the competition lap.

---

## What IBM Bob contributed

Bob (the AI assistant in this session) contributed the following — each item was
verified by me in a live race before being accepted:

1. **Diagnosed four bugs in the original `snakeoil3` driver**: accumulating throttle,
   missing brakes, `target_speed=300`, and TCS/boost conflict. Fixed in `snakeoil3_v1.py`.

2. **Designed the F1-style controller** in `torcs_learning_agent.py`:
   - Racing line (apex-out-apex using curvature-weighted track position)
   - Trail braking (partial brake pressure while turning in)
   - Late braking (braking envelope using 22 m/s² instead of 4 m/s²)
   - Proportional ABS and TCS with wheel-radius calibration
   - Gear hysteresis to prevent gear hunting

3. **Physics audit**: confirmed sub-60s is impossible on Corkscrew
   (3608 m track requires 216 km/h average; tightest corners cap at ~32 km/h minimum).

4. **Raised exploration bounds** when the pace agent hit its search ceiling,
   allowing the model to continue improving from 125 s toward 107 s.

5. **Froze the model** to run-34's exact speed policy (107.3 s, zero damage,
   zero outside-road samples) by setting `pace_plateau=True` and copying the
   722 proven cap values into `accepted_caps`.

6. **Explained every decision** with evidence from the recorded telemetry files
   before any change was implemented.

## What I (the learner) did

- Ran all 36 live races and operated TORCS for each attempt.
- Verified each result on the TORCS screen after each race.
- Chose to use `--allow-offtrack` mode after understanding the rule risk.
- Selected run 34 as the submission result after reviewing all run outcomes.
- Confirmed zero damage on every accepted run.
- Kept the rescue baseline untouched as a fallback throughout.

---

## Note on off-track mode

`--allow-offtrack` is experimental. On all 32 clean accepted runs including
run 34, `outside_road_samples = 0` — the car's SCR sensor rays never left
the road. One body-position estimate showed 44 marginal samples on run 36,
which is why run 34 was chosen instead.

The road-boundary alternative (no off-track) is available:
```powershell
py torcs_pace_agent.py --evaluate
```
This uses `corkscrew_pace_model.json` and is guaranteed to stay within the
road boundary at all times.

---

## SkillsBuild activities
[Complete your individually assigned SkillsBuild activities and list them here
before final submission — check the organiser's current instructions.]
