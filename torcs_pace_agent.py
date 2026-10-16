"""Experimental faster corner profile, with optional map-bounded excursions."""

import argparse
import copy
import csv
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET

import torcs_adaptive_agent as diagnostics
import torcs_pace_reference as core


def roadside_limits(geometry, xml_path, allow_offtrack):
    root=ET.fromstring(Path(xml_path).read_text().replace("&default-surfaces;",""))
    main=root.find("section[@name='Main Track']")
    inherited={s:dict(end_width=0,border_width=0,border_style="plan",surface="unknown",
                      barrier_width=0,barrier_style="fence") for s in ("Left","Right")}
    sections=[]
    for segment in main.find("section[@name='Track Segments']"):
        limits={}
        for side in ("Left","Right"):
            prev=inherited[side]
            def read(section):
                return {v.get('name'):v.get('val') for v in section} if section is not None else {}
            data=read(segment.find(f"section[@name='{side} Side']"))
            border=read(segment.find(f"section[@name='{side} Border']"))
            barrier=read(segment.find(f"section[@name='{side} Barrier']"))
            start=float(data.get("start width",prev["end_width"]))
            end=float(data.get("end width",data.get("width",start)))
            width=float(border.get("width",prev["border_width"]))
            style=border.get("style",prev["border_style"])
            height=float(border.get("height",0))
            surface=data.get("surface",prev["surface"])
            barrier_style=barrier.get("style",prev["barrier_style"])
            barrier_width=0 if barrier_style=="fence" else float(barrier.get("width",prev["barrier_width"]))
            inherited[side]=dict(end_width=end,border_width=width,border_style=style,surface=surface,
                                 barrier_style=barrier_style,barrier_width=barrier_width)
            # A raised border/wall at the road edge blocks access to any side
            # behind it. Use the narrower endpoint across a variable-width side.
            blocked=width>0 and (style=="wall" or height>.15)
            runoff=0 if blocked else max(0,width+min(start,end)-barrier_width)
            half=geometry["width_m"]/2
            margin_limit=(half+runoff-1.2-.4)/half
            ceiling=1.15 if allow_offtrack else .8
            limits[side.lower()]=max(.2,min(ceiling,margin_limit))
            limits[side.lower()+"_surface"]=surface
        sections.append(limits)
    return [sections[b["sector"]] for b in geometry["bins"]]


def pace_caps(model, lateral, straight):
    return [max(model["accepted_caps"][i],min(straight,model["exploration"]["straight_ceiling_kmh"],
                    straight if abs(b["curvature"])<.0001 else math.sqrt(lateral/abs(b["curvature"]))*3.6))
            for i,b in enumerate(model["geometry"]["bins"])]


def propose(model):
    if model.get("pending"):
        return model["pending"]
    if model.get("pace_plateau"):
        return dict(kind="evaluation",caps=model["accepted_caps"][:],sector=None,plateau=True)
    lateral=min(model["next_lateral_ms2"],model["exploration"]["lateral_search_bound_ms2"])
    straight=min(model["next_straight_kmh"],model["exploration"]["straight_ceiling_kmh"])
    caps=pace_caps(model,lateral,straight)
    if max(b-a for a,b in zip(model["accepted_caps"],caps))<.5:
        model["pace_plateau"]=True
        return dict(kind="evaluation",caps=model["accepted_caps"][:],sector=None,plateau=True)
    trial=dict(kind="pace_trial",caps=caps,sector="whole_circuit",clean_times=[],
               sector_name=f"corner budget {lateral:.1f} m/s^2, straight ceiling {straight:.0f} km/h",
               lateral_budget_ms2=lateral,straight_limit_kmh=straight,local_repairs=[])
    model["pending"]=trial
    return trial


class PaceController:
    def __init__(self, model, caps):
        self.model=model
        self.geometry=model["geometry"]
        self.profile=core.make_envelope(model,caps)
        self.previous_steer=0.0

    def act(self,state,dt=.02):
        g=self.geometry;n=len(g["bins"]);ds=g["bin_size_m"]
        speed=max(0,state["speedX"])
        index=min(n-1,int((state["distFromStart"]%g["length_m"])/ds))
        preview=math.ceil((speed/3.6*.3+5)/ds)
        target=min(self.profile[(index+j)%n] for j in range(preview+1))
        limits=self.model["position_limits"][index]
        pos=state["trackPos"]
        # Replace the old blanket 30 km/h cut at |trackPos| > .4 with a
        # gradual response to the actual mapped clearance envelope.
        distance_to_limit=(limits["left"]-pos if pos>=0 else limits["right"]+pos)*g["width_m"]/2
        if distance_to_limit<.5:
            target=min(target,max(25,target*max(.35,distance_to_limit/.5)))
        if abs(state["angle"])>.35:
            target=min(target,60)
        if abs(state["angle"])>.6:
            target=min(target,30)
        forward=state["track"][9]
        if forward<0:
            if not self.model["allow_offtrack"] or abs(pos)<=1:
                raise ValueError("Invalid road rays without a permitted mapped excursion")
            # Map-only fallback; SCR does not supply wall-distance rays here.
            target=min(target,60)
        # A short forward ray can be a bend viewed from the current heading,
        # not an obstacle. The map handles bends; an extreme short ray while
        # poorly aligned still triggers an emergency slowdown.
        elif forward<8 and abs(state["angle"])>.2:
            target=min(target,25)
        kappa=g["bins"][(index+int(speed/3.6*.15/ds))%n]["curvature"]
        feed=.65*math.atan(2.95*kappa)/math.radians(21)
        desired=core.clamp(feed+state["angle"]/math.radians(21)-.5*pos,-1,1)
        rate=3*core.clamp(dt,.005,.1)
        steer=core.clamp(desired,self.previous_steer-rate,self.previous_steer+rate)
        self.previous_steer=steer
        error=target-state["speedX"]
        accel=core.clamp(error*.08,0,.8) if error>2 else 0
        brake=core.clamp(-error*.06,0,.85) if error<-2 else 0
        wheels=state.get("wheelSpinVel");radii=self.model["wheel_radius_m"]
        if isinstance(wheels,list) and len(wheels)==4 and speed>20:
            if radii["rear"]:
                slip=(statistics.mean(wheels[2:])*radii["rear"]-speed/3.6)/max(speed/3.6,5)
                if slip>.12:
                    accel*=core.clamp(1-(slip-.12)*3,0,1)
            if radii["front"] and statistics.mean(wheels[:2])*radii["front"]<.75*speed/3.6:
                brake*=.4
        action=dict(steer=steer,accel=accel,brake=brake,
                    gear=1+sum(speed>v for v in (50,80,110,140,170)),clutch=0,focus=0,meta=0)
        return action,dict(target_speed_kmh=target,bin=index,sector=g["bins"][index]["sector"],curvature=kappa)


def learn(model,trial,result):
    diagnostics.read_observations(model,result)
    metric=result.get("comparison_time_s")
    damage_free=(metric is not None and math.isfinite(metric) and metric>5 and
                 result.get("finish_supported") and result.get("max_damage")==0 and
                 result.get("position_envelope_ok",False))
    decision="no_update_incomplete"
    if trial["kind"]=="pace_trial":
        if damage_free:
            trial["clean_times"].append(metric)
            decision="repeat_damage_free_pace_trial"
            if len(trial["clean_times"])>=2:
                candidate=statistics.median(trial["clean_times"][-2:])
                if candidate<model["accepted_metric_s"]-.2:
                    model["accepted_caps"]=trial["caps"][:]
                    model["accepted_metric_s"]=candidate
                    model["generation"]+=1
                    model["no_gain_trials"]=0
                    model["next_lateral_ms2"]=trial["lateral_budget_ms2"]+1
                    model["next_straight_kmh"]=trial["straight_limit_kmh"]+20
                    decision="accepted_faster_damage_free_profile"
                else:
                    model["no_gain_trials"]+=1
                    model["next_lateral_ms2"]=max(6,trial["lateral_budget_ms2"]-.5)
                    if model["no_gain_trials"]>=3:
                        model["pace_plateau"]=True
                    decision="rejected_no_measured_gain"
                model["pending"]=None
        elif result["status"] in ("damage","off_track","unstable","stuck","reversing"):
            fault=result.get("failure_context",{}).get("first_warning",{})
            if "bin" in fault:
                index=fault["bin"]
                zone=next(z for z in model["zones"] if index in z["indices"])
                changed=False
                for i in zone["indices"]:
                    old=trial["caps"][i]
                    trial["caps"][i]=(old+model["accepted_caps"][i])*.5
                    changed=changed or old-trial["caps"][i]>.25
                trial["clean_times"]=[]
                repair=dict(run_id=result["run_id"],zone=zone["id"],context=result["failure_context"])
                trial["local_repairs"].append(repair)
                model["failure_memory"].append(repair)
                if changed and len(trial["local_repairs"])<=6:
                    model["pending"]=trial
                    decision="local_corner_backoff_fast_sections_preserved"
                else:
                    result["pause_for_review"]=True
                    decision="failure_at_known_policy_pause_for_review"
            else:
                result["pause_for_review"]=True
                decision="failure_location_unknown_pause_for_review"
    result["training_damage_free"]=bool(damage_free)
    result["controller_fingerprints"]=model["controller_fingerprints"]
    model["history"].append(dict(run_id=result["run_id"],status=result["status"],kind=trial["kind"],
                                comparison_time_s=metric,damage_free=bool(damage_free),
                                outside_road_samples=result.get("outside_road_samples"),decision=decision))
    return decision


def create_model(source,track,allow_offtrack):
    model=copy.deepcopy(json.loads(Path(source).read_text()))
    model.update(pace_version=1,pending=None,history=[],failure_memory=[],zone_observations={},
                 source_model_sha256=core.digest(source),allow_offtrack=allow_offtrack,
                 position_limits=roadside_limits(model["geometry"],track,allow_offtrack),
                 next_lateral_ms2=9.0,next_straight_kmh=160.0,no_gain_trials=0,
                 rule_status="Off-track eligibility unverified" if allow_offtrack else "Road-boundary mode")
    model["controller_fingerprints"]=dict(pace=core.digest(__file__),reference=core.digest(core.__file__))
    return model


def main():
    here=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes",type=int,default=1)
    parser.add_argument("--allow-offtrack",action="store_true",help="Experimental limited excursions; organiser eligibility unverified")
    parser.add_argument("--evaluate",action="store_true")
    parser.add_argument("--model",type=Path)
    parser.add_argument("--source-model",type=Path,default=here/"corkscrew_adaptive_model.json")
    parser.add_argument("--output",type=Path)
    parser.add_argument("--prepare",action="store_true")
    parser.add_argument("--host",default="127.0.0.1")
    parser.add_argument("--port",type=int,default=3001)
    parser.add_argument("--timeout",type=float,default=8)
    parser.add_argument("--startup-timeout",type=float,default=30)
    parser.add_argument("--connection-attempts",type=int,default=20)
    parser.add_argument("--duration",type=float,default=600)
    args=parser.parse_args()
    if min(args.episodes,args.timeout,args.startup_timeout,args.connection_attempts,args.duration)<=0:
        parser.error("Counts and timeouts must be positive")
    mode="excursion" if args.allow_offtrack else "pace"
    args.model=args.model or here/f"corkscrew_{mode}_model.json"
    args.output=args.output or here/f"{mode}_runs"
    track=here.parent/"torcs/tracks/road/corkscrew/corkscrew.xml"
    if not args.model.exists():
        core.save_json(args.model,create_model(args.source_model,track,args.allow_offtrack))
    model=json.loads(args.model.read_text())
    if args.prepare:
        print(f"Prepared {args.model}; mode={mode}; old models preserved")
        return
    if model["allow_offtrack"]!=args.allow_offtrack:
        raise SystemExit("Use separate model files for road-boundary and excursion experiments")
    if model["controller_fingerprints"]!={"pace":core.digest(__file__),"reference":core.digest(core.__file__)}:
        raise SystemExit("Controller changed; fork a new model before comparing results")
    if core.digest(track)!=model["geometry"]["track_sha256"]:
        raise SystemExit("Track fingerprint mismatch")
    args.car=model["environment"]["car_label"];args.settings_label=model["environment"]["settings_label"]
    diagnostics.RUNS_ROOT=args.output
    core.LearningController=PaceController;core.learn=learn
    lock=args.model.with_suffix('.lock')
    try:
        handle=lock.open('x')
    except FileExistsError:
        raise SystemExit(f"Model in use or stale lock: {lock}")
    try:
        with handle:
            for i in range(args.episodes):
                trial=(dict(kind="evaluation",caps=model["accepted_caps"][:],sector=None)
                       if args.evaluate else propose(model))
                if trial.get("plateau"):
                    print("No further tested improvement within current search settings; review telemetry.");break
                core.save_json(args.model,model)
                print(f"Attempt {i+1}: {trial.get('sector_name','accepted policy')}; {mode} mode")
                print("Start a fresh single-car race. Excursions are experimental and not verified eligible.")
                result=core.run_episode(args,model,trial,args.output)
                core.save_json(args.model,model)
                print(f"{result['status']}: {result['decision']}; logs {args.output/result['run_id']}")
                if result["status"] in ("interrupted","connection_or_sensor_error") or result.get("pause_for_review"):
                    break
                if i+1<args.episodes:
                    print("Select New Race in TORCS to continue.")
    finally:
        lock.unlink(missing_ok=True)


if __name__=="__main__":
    main()
