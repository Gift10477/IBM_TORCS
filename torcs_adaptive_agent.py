"""Learn faster clean corner/straight speeds from real TORCS experiments.

Run from gym_torcs: py torcs_adaptive_agent.py --episodes 20
Select a fresh single-car race in TORCS for each episode.
"""

import argparse
from collections import deque
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

import torcs_learning_reference as core


RUNS_ROOT = None


def make_zones(geometry):
    zones = []
    for i,b in enumerate(geometry["bins"]):
        kind = "straight" if abs(b["curvature"]) < .0001 else "corner"
        if not zones or zones[-1]["kind"] != kind:
            zones.append(dict(id=str(len(zones)),kind=kind,indices=[],attempts=0,
                              failures=0,accepted=0,step_kmh=None,no_gain_count=0))
        zones[-1]["indices"].append(i)
    for zone in zones:
        zone["start_m"] = geometry["bins"][zone["indices"][0]]["s"]
        zone["length_m"] = len(zone["indices"])*geometry["bin_size_m"]
    return zones


def initialise(source_path, runs_root, max_speed=260, corner_lateral_bound=14):
    source = json.loads(Path(source_path).read_text())
    model = copy.deepcopy(source)
    # Prefer the fastest measured clean full-circuit policy, even if its second
    # confirmation was interrupted. It remains a calibration candidate here.
    candidates = []
    for folder in Path(runs_root).iterdir():
        if not folder.is_dir() or not (folder/"summary.json").exists():
            continue
        result = json.loads((folder/"summary.json").read_text())
        if (result.get("finish_supported") and result.get("max_damage")==0 and
            result.get("max_abs_track_pos",1)<.75 and result.get("comparison_time_s",0)>5 and
            result.get("code_sha256")==core.PROVEN_CONTROLLER_RUN_SHA256 and
            result.get("track_sha256")==source["geometry"]["track_sha256"] and
            result.get("car_label")==source["environment"]["car_label"] and
            result.get("settings_label")==source["environment"]["settings_label"]):
            candidates.append((result["comparison_time_s"],folder,result))
    seed_times = []
    if candidates:
        metric,folder,result = min(candidates,key=lambda item:item[0])
        policy = json.loads((folder/"policy_used.json").read_text())
        model["accepted_caps"] = policy["caps"][:]
        seed_times = [metric]
        model["adaptive_seed_run"] = dict(run_id=result["run_id"],time_s=metric,
                                         source_summary_sha256=core.digest(folder/"summary.json"),
                                         note="One clean run; repeat with adaptive controller before exploration")
    model.update(adaptive_version=1,generation=0,pending=None,history=[],accepted_metric_s=None,
                 calibration_clean_times=seed_times,failures_by_sector={},attempts_by_sector={},
                 zones=make_zones(model["geometry"]),next_kind="corner",failure_memory=[],
                 zone_observations={},source_model_sha256=core.digest(source_path),
                 exploration=dict(straight_ceiling_kmh=max_speed,lateral_search_bound_ms2=corner_lateral_bound,
                                  description="Exploration bounds, not measured grip or guaranteed safe speeds"))
    model["hard_caps"] = [max(model["accepted_caps"][i],
        max_speed if abs(b["curvature"])<.0001 else min(180,math.sqrt(corner_lateral_bound/abs(b["curvature"]))*3.6))
        for i,b in enumerate(model["geometry"]["bins"])]
    model["upper_caps"] = model["hard_caps"][:]
    model["controller_fingerprints"] = dict(adaptive=core.digest(__file__),reference=core.digest(core.__file__))
    return model


def propose(model):
    if model["accepted_metric_s"] is None:
        return dict(kind="calibration",caps=model["accepted_caps"][:],sector=None)
    if model.get("pending"):
        return model["pending"]
    accepted = model["accepted_caps"]
    baseline = core.make_envelope(model,accepted)
    ds = model["geometry"]["bin_size_m"]
    choices = []
    for zone in model["zones"]:
        indices = zone["indices"]
        if len(indices)<3 or zone["no_gain_count"]>=3:
            continue
        average = statistics.mean(accepted[i] for i in indices)
        initial_step = min(10,max(3,average*.15)) if zone["kind"]=="corner" else min(30,max(10,average*.2))
        step = zone["step_kmh"] if zone["step_kmh"] is not None else initial_step
        caps = accepted[:]
        for i in indices:
            caps[i] = min(model["upper_caps"][i],accepted[i]+step)
        actual_step = max(caps[i]-accepted[i] for i in indices)
        if actual_step < .5:
            continue
        envelope = core.make_envelope(model,caps)
        estimated_gain = sum(ds*3.6*(1/a-1/b) for a,b in zip(baseline,envelope))
        if estimated_gain < .03:
            continue
        score = estimated_gain/(1+.1*zone["attempts"]+.15*zone["failures"])
        proposal = dict(kind="adaptive_trial",caps=caps,sector=zone["id"],
                        sector_name=f"{zone['kind']} at {zone['start_m']:.0f} m",
                        zone_kind=zone["kind"],step_kmh=actual_step,clean_times=[],
                        estimated_gain_s=estimated_gain)
        choices.append((zone["kind"]==model["next_kind"],score,proposal))
    if not choices:
        return dict(kind="evaluation",caps=accepted[:],sector=None,plateau=True)
    proposal = max(choices,key=lambda item:(item[0],item[1]))[2]
    model["pending"] = proposal
    return proposal


def read_observations(model, result):
    """Use measured telemetry to locate the first warning near a failed attempt."""
    if RUNS_ROOT is None:
        return
    path = Path(RUNS_ROOT)/result["run_id"]/"telemetry.csv"
    if not path.exists():
        return
    lookup = {i:z["id"] for z in model["zones"] for i in z["indices"]}
    recent = deque(maxlen=150)
    worst_position_point = None
    observations = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("bin") in (None,""):
                # Terminal rows still carry the exact distance and raw sensors.
                index=min(len(model['accepted_caps'])-1,int(float(row['distFromStart'])/model['geometry']['bin_size_m']))
            else:
                index=int(row["bin"])
            zone_id=lookup[index]
            speed=float(row["speedX"])
            target=float(row["target_speed_kmh"]) if row.get("target_speed_kmh") else None
            point=dict(distance_m=float(row["distFromStart"]),bin=index,zone=zone_id,
                       speed_kmh=speed,target_kmh=target,position=float(row["trackPos"]),
                       angle=float(row["angle"]),damage=float(row["damage"]))
            recent.append(point)
            if worst_position_point is None or abs(point["position"])>abs(worst_position_point["position"]):
                worst_position_point=point
            item=observations.setdefault(zone_id,dict(samples=0,peak_speed_kmh=0,max_abs_position=0,max_abs_angle=0))
            item["samples"]+=1
            item["peak_speed_kmh"]=max(item["peak_speed_kmh"],speed)
            item["max_abs_position"]=max(item["max_abs_position"],abs(point["position"]))
            item["max_abs_angle"]=max(item["max_abs_angle"],abs(point["angle"]))
    model["zone_observations"][result["run_id"]]=observations
    driving_failure=result["status"] in ("damage","off_track","unstable","stuck","reversing")
    margin_failure=result.get("finish_supported") and result.get("max_abs_track_pos",0)>=.75
    if (not driving_failure and not margin_failure) or not recent:
        return
    warning=(worst_position_point if margin_failure else
             next((p for p in recent if abs(p["position"])>.6 or abs(p["angle"])>.25),recent[-1]))
    if warning["target_kmh"] is not None and warning["speed_kmh"]>warning["target_kmh"]+8:
        cause="entry_speed_above_target"
    elif abs(warning["position"])>.6:
        cause="lateral_position_margin_lost"
    elif abs(warning["angle"])>.25:
        cause="heading_instability"
    else:
        cause="driving_failure_cause_unconfirmed"
    result["failure_context"] = dict(first_warning=warning,last_sample=recent[-1],cause=cause,
                                     note="Observation and attribution heuristic, not proof of causation")


def learn(model, proposal, result):
    read_observations(model,result)
    metric=result.get("comparison_time_s")
    clean=(metric is not None and math.isfinite(metric) and metric>5 and result.get("finish_supported")
           and result.get("max_damage")==0 and result.get("max_abs_track_pos",1)<.75)
    decision="no_update_incomplete"
    if proposal["kind"]=="calibration" and clean:
        model["calibration_clean_times"].append(metric)
        decision="calibration_recorded"
        if len(model["calibration_clean_times"])>=2:
            model["accepted_metric_s"]=statistics.median(model["calibration_clean_times"][-2:])
            decision="calibration_complete"
    elif proposal["kind"]=="adaptive_trial":
        zone=next(z for z in model["zones"] if z["id"]==proposal["sector"])
        if clean:
            proposal["clean_times"].append(metric)
            model["pending"]=proposal
            decision="repeat_candidate"
            if len(proposal["clean_times"])>=2:
                candidate_time=statistics.median(proposal["clean_times"][-2:])
                if candidate_time<model["accepted_metric_s"]-.2:
                    model["accepted_caps"]=proposal["caps"][:]
                    model["accepted_metric_s"]=candidate_time
                    model["generation"]+=1
                    zone["accepted"]+=1
                    zone["step_kmh"]=min(15 if zone["kind"]=="corner" else 35,proposal["step_kmh"]*1.15)
                    zone["no_gain_count"]=0
                    decision="accepted_clean_faster_corner" if zone["kind"]=="corner" else "accepted_clean_faster_straight"
                else:
                    zone["no_gain_count"]+=1
                    zone["step_kmh"]=max(.5,proposal["step_kmh"]*.5)
                    decision="rejected_no_measured_gain_smaller_probe_next"
                zone["attempts"]+=1
                model["pending"]=None
                model["next_kind"]="straight" if zone["kind"]=="corner" else "corner"
        elif result.get("finish_supported") or result["status"] in ("damage","off_track","unstable","stuck","reversing"):
            fault=result.get("failure_context",{}).get("first_warning",{})
            attributed=not fault or fault.get("zone")==zone["id"]
            if fault and not attributed:
                ahead=(zone["start_m"]-fault["distance_m"])%model["geometry"]["length_m"]
                attributed=ahead<min(150,(max(0,fault["speed_kmh"])/3.6)**2/8+20)
            zone["attempts"]+=1
            if attributed:
                zone["failures"]+=1
                for i in zone["indices"]:
                    model["upper_caps"][i]=max(model["accepted_caps"][i],
                        min(model["upper_caps"][i],proposal["caps"][i]-.25))
                zone["step_kmh"]=max(.5,proposal["step_kmh"]*.5)
                decision="failure_located_smaller_local_step_next"
            else:
                decision="failure_elsewhere_pause_for_review"
                result["pause_for_review"]=True
            model["failure_memory"].append(dict(run_id=result["run_id"],tested_zone=zone["id"],
                                                attributed_to_probe=attributed,context=result.get("failure_context"),
                                                next_step_kmh=zone["step_kmh"]))
            model["pending"]=None
            # Retry this type next; accepted speeds elsewhere are untouched.
            model["next_kind"]=zone["kind"]
    model["history"].append(dict(run_id=result["run_id"],status=result["status"],kind=proposal["kind"],
                                zone=proposal.get("sector"),clean=clean,comparison_time_s=metric,decision=decision))
    result["controller_fingerprints"]=model["controller_fingerprints"]
    return decision


class AdaptiveController(core.LearningController):
    """Keep the known clean control law while learning faster corner targets.

    No unverified apex offset or invented grip estimate is added to steering.
    The geometry braking envelope still constrains every proposed increase.
    """
    pass


def main():
    global RUNS_ROOT
    here=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes",type=int,default=1)
    parser.add_argument("--evaluate",action="store_true")
    parser.add_argument("--model",type=Path,default=here/"corkscrew_adaptive_model.json")
    parser.add_argument("--source-model",type=Path,default=here/"corkscrew_fast_model.json")
    parser.add_argument("--history",type=Path,default=here/"learning_runs")
    parser.add_argument("--output",type=Path,default=here/"adaptive_runs")
    parser.add_argument("--speed-ceiling",type=float,default=260,
                        help="Initial straight exploration ceiling; not an immediate speed target")
    parser.add_argument("--host",default="127.0.0.1")
    parser.add_argument("--port",type=int,default=3001)
    parser.add_argument("--timeout",type=float,default=5)
    parser.add_argument("--startup-timeout",type=float,default=30)
    parser.add_argument("--connection-attempts",type=int,default=20)
    parser.add_argument("--duration",type=float,default=600)
    parser.add_argument("--prepare",action="store_true")
    args=parser.parse_args()
    if min(args.episodes,args.timeout,args.startup_timeout,args.connection_attempts,args.duration)<=0 or not 120<=args.speed_ceiling<=300:
        parser.error("Positive counts/timeouts and speed ceiling 120..300 required")
    if not args.model.exists():
        model=initialise(args.source_model,args.history,args.speed_ceiling)
        core.save_json(args.model,model)
    else:
        model=json.loads(args.model.read_text())
    if args.prepare:
        print(f"Prepared {args.model}. {len(model['zones'])} zones; corners and straights both explored.")
        return
    expected=dict(adaptive=core.digest(__file__),reference=core.digest(core.__file__))
    if model["controller_fingerprints"]!=expected:
        raise SystemExit("Controller changed. Use a new model path and recalibrate; old timing references cannot be mixed.")
    track=here.parent/"torcs/tracks/road/corkscrew/corkscrew.xml"
    if model["geometry"]["track_sha256"]!=core.digest(track):
        raise SystemExit("Track XML changed; use a separate model")
    args.car=model["environment"]["car_label"]
    args.settings_label=model["environment"]["settings_label"]
    RUNS_ROOT=args.output
    core.LearningController=AdaptiveController
    core.learn=learn
    lock=args.model.with_suffix(".lock")
    try:
        handle=lock.open("x")
    except FileExistsError:
        raise SystemExit(f"Model lock exists: {lock}. Stop other learners; remove a stale lock only after they have exited.")
    try:
        with handle:
            handle.write("adaptive learner owns this model\n");handle.flush()
            for i in range(args.episodes):
                proposal=(dict(kind="evaluation",caps=model["accepted_caps"][:],sector=None)
                          if args.evaluate else propose(model))
                if proposal.get("plateau"):
                    print("Search resolution reached; no untested speed improvement remains within current bounds.")
                    break
                core.save_json(args.model,model)
                print(f"Attempt {i+1}/{args.episodes}: {proposal['kind']} {proposal.get('sector_name','')} generation={model['generation']}")
                print("Start a fresh Corkscrew race with one external driver. Keep the same car and settings.")
                result=core.run_episode(args,model,proposal,args.output)
                core.save_json(args.model,model)
                print(f"{result['status']}: {result['decision']}. Logs: {args.output/result['run_id']}")
                if result["status"] in ("interrupted","connection_or_sensor_error") or result.get("pause_for_review"):
                    break
                if proposal["kind"]=="calibration" and not result["finish_supported"]:
                    print("Calibration failed. Inspect this run before speed exploration.")
                    break
                if i+1<args.episodes:
                    print("Select New Race in TORCS; waiting for the next attempt.")
    finally:
        lock.unlink(missing_ok=True)


if __name__=="__main__":
    main()
