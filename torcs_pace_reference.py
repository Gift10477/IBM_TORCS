"""Track-specific learning driver: measured speed-policy search, not a neural network.

Prepare: py torcs_learning_agent.py --prepare
Drive:   py torcs_learning_agent.py --episodes 10
TORCS must start a fresh single-car race for each episode. The client never
changes simulator physics or launches/kills TORCS. See LEARNING_AGENT_GUIDE.md.
"""

import argparse
from bisect import bisect_right
from collections import deque
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import statistics
import time
import uuid
import xml.etree.ElementTree as ET

from torcs_baseline_rescue import ANGLES, clamp, encode_action, parse_packet


VERSION = 1
PROVEN_CONTROLLER_RUN_SHA256 = "2339414924eb183bd090d63478136982e0d9398c517f396a2f338a4b9704138d"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    """Atomic replacement; previous revisions also remain in per-run artifacts."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def read_geometry(path, spacing=5):
    # Only this known local entity is omitted. No external entities are fetched.
    root = ET.fromstring(Path(path).read_text().replace("&default-surfaces;", ""))
    main = root.find("./section[@name='Main Track']")
    if main is None:
        raise ValueError("Track XML has no Main Track")
    width = float(main.find("attnum[@name='width']").get("val"))
    global_step = float(main.find("attnum[@name='profil steps length']").get("val"))
    pieces, sectors = [], []
    distance = x = y = heading = 0.0
    for sector_id, segment in enumerate(main.find("section[@name='Track Segments']")):
        data = {e.get("name"): e for e in segment if e.tag in ("attnum", "attstr")}
        def value(name, default=None):
            return data[name].get("val") if name in data else default
        kind = value("type")
        if kind == "str":
            radii, length = [None], float(value("lg"))
        else:
            radius, end = float(value("radius")), float(value("end radius", value("radius")))
            arc = float(value("arc"))
            if data["arc"].get("unit") == "deg":
                arc = math.radians(arc)
            elif data["arc"].get("unit") not in (None, "rad"):
                raise ValueError("Unsupported arc unit")
            length = (radius + end) * .5 * arc
            steps = int(float(value("profil steps", 1)))
            if value("profil", "spline") == "spline":
                if steps == 1:
                    step_length = float(value("profil steps length", global_step))
                    steps = int(length / step_length) + 1 if step_length else 1
            else:
                steps = 1
            if steps < 1 or min(radius, end) <= 0:
                raise ValueError("Invalid track geometry")
            radii = [radius + (end-radius)*i/(steps-1) for i in range(steps)] if steps > 1 else [radius]
            # Reproduce TORCS track4.cpp's spiral correction, not radius*arc
            # or the arithmetic average. This matches the live 3608.45 m track.
            if radius != end and steps > 1:
                length = steps * arc / sum(1/r for r in radii)
        sector = dict(id=sector_id, name=segment.get("name"), kind=kind,
                      start_m=distance, length_m=length)
        sectors.append(sector)
        for radius in radii:
            ds = length / len(radii)
            curvature = 0 if radius is None else (1 if kind == "lft" else -1) / radius
            pieces.append(dict(s=distance, length=ds, curvature=curvature,
                               sector=sector_id, x=x, y=y, heading=heading))
            if curvature:
                x += (math.sin(heading+curvature*ds)-math.sin(heading))/curvature
                y += (-math.cos(heading+curvature*ds)+math.cos(heading))/curvature
            else:
                x += ds*math.cos(heading)
                y += ds*math.sin(heading)
            heading += curvature*ds
            distance += ds
    count = math.ceil(distance/spacing)
    step = distance/count
    bins, piece_index = [], 0
    for i in range(count):
        s = i*step
        while piece_index+1 < len(pieces) and pieces[piece_index+1]["s"] <= s:
            piece_index += 1
        p = pieces[piece_index]
        d, k, h = s-p["s"], p["curvature"], p["heading"]
        bx = p["x"] + ((math.sin(h+k*d)-math.sin(h))/k if k else d*math.cos(h))
        by = p["y"] + ((-math.cos(h+k*d)+math.cos(h))/k if k else d*math.sin(h))
        bins.append(dict(s=s, curvature=k, sector=p["sector"], x=bx, y=by))
    return dict(length_m=distance, width_m=width, bin_size_m=step, bins=bins,
                sectors=sectors, track_sha256=digest(path))


def seed_from_clean_run(geometry, source):
    source = Path(source)
    summary = json.loads((source/"summary.json").read_text())
    with (source/"telemetry.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or summary.get("status") != "server_shutdown":
        raise ValueError("Seed must end with server shutdown and have measured coverage")
    n, step, length = len(geometry["bins"]), geometry["bin_size_m"], geometry["length_m"]
    targets = [[] for _ in range(n)]
    front, rear = [], []
    for row in rows:
        damage, position = float(row["damage"]), float(row["trackPos"])
        if damage != 0 or abs(position) > .8:
            raise ValueError("Seed contains damage or inadequate road-position margin")
        s = float(row["distFromStart"])
        if s < 0 or s > length+1:
            raise ValueError("Seed distances disagree with track XML")
        index = min(n-1, int(s/step))
        targets[index].append(float(row["target_speed_kmh"]))
        speed = float(row["speedX"])/3.6
        wheels = json.loads(row["wheelSpinVel"])
        if speed > 8 and abs(float(row["angle"])) < .03 and float(row["accel"]) < .05 and float(row["brake"]) == 0:
            if min(wheels) > 1:
                front.append(speed/statistics.mean(wheels[:2]))
                rear.append(speed/statistics.mean(wheels[2:]))
    coverage = sum(bool(t) for t in targets)/n
    if coverage < .99 or float(rows[-1]["distRaced"])-float(rows[0]["distRaced"]) < length-2:
        raise ValueError("Seed does not cover a full circuit")
    caps = [min(t) if t else 20.0 for t in targets]
    # Absolute geometric upper bounds are exploratory limits, not known safe speeds.
    hard = [200.0 if abs(b["curvature"]) < .0001 else
            min(120.0, max(caps[i], math.sqrt(4/max(abs(b["curvature"]), .0001))*3.6))
            for i, b in enumerate(geometry["bins"])]
    return dict(version=VERSION, geometry=geometry, accepted_caps=caps, hard_caps=hard,
                accepted_metric_s=None, calibration_clean_times=[], pending=None,
                failures_by_sector={}, attempts_by_sector={}, history=[], generation=0,
                seed=dict(run_id=summary.get("run_id"), telemetry_sha256=digest(source/"telemetry.csv"),
                          summary_sha256=digest(source/"summary.json"), coverage=coverage,
                          finish_evidence="user-reported finish plus server shutdown and full distance coverage"),
                wheel_radius_m=dict(front=statistics.median(front) if front else None,
                                    rear=statistics.median(rear) if rear else None))


def make_envelope(model, caps):
    """Cyclic backward braking and forward acceleration feasibility in m/s."""
    ds = model["geometry"]["bin_size_m"]
    speeds = [v/3.6 for v in caps]
    n = len(speeds)
    for _ in range(3):
        for i in range(n-1, -1, -1):
            speeds[i] = min(speeds[i], math.sqrt(speeds[(i+1)%n]**2+2*4*ds))
        for i in range(n):
            speeds[i] = min(speeds[i], math.sqrt(speeds[(i-1)%n]**2+2*3*ds))
    return [v*3.6 for v in speeds]


def position_allowed(model, state):
    limits = model.get("position_limits")
    if limits is None:
        return abs(state["trackPos"]) <= 1
    index = min(len(limits)-1,int(state["distFromStart"]/model["geometry"]["bin_size_m"]))
    return -limits[index]["right"] <= state["trackPos"] <= limits[index]["left"]


def propose(model):
    if model["accepted_metric_s"] is None:
        return dict(kind="calibration", caps=model["accepted_caps"][:], sector=None)
    if model["pending"]:
        return model["pending"]
    if model.get("coarse_straight_learning"):
        accepted = model.get("accepted_straight_limit_kmh", 60.0)
        upper = model.get("straight_upper_limit_kmh", 200.0)
        target = min(upper, model.get("next_straight_limit_kmh", 120.0))
        if target > accepted + 1:
            caps = model["accepted_caps"][:]
            for i,b in enumerate(model["geometry"]["bins"]):
                if abs(b["curvature"]) < .0001:
                    caps[i] = max(caps[i], min(target, model["hard_caps"][i]))
            proposal = dict(kind="straight_trial",caps=caps,sector="all_straights",
                            sector_name="all straights; corner caps unchanged",
                            straight_limit_kmh=target,clean_times=[])
            model["pending"] = proposal
            return proposal
    options = []
    bins, ds = model["geometry"]["bins"], model["geometry"]["bin_size_m"]
    for sector in model["geometry"]["sectors"]:
        sid = sector["id"]
        indices = [i for i,b in enumerate(bins) if b["sector"] == sid]
        if len(indices) < 4 or model["failures_by_sector"].get(str(sid), 0) >= 3:
            continue
        increase = 5.0 if sector["kind"] == "str" else 1.0
        candidate = model["accepted_caps"][:]
        for i in indices:
            candidate[i] = min(model["hard_caps"][i], candidate[i]+increase)
        improvement = sum(ds*(1/(model["accepted_caps"][i]/3.6)-1/(candidate[i]/3.6)) for i in indices)
        attempts = model["attempts_by_sector"].get(str(sid), 0)
        score = improvement/(1+attempts*.15)
        if score > .001:
            options.append((score, dict(kind="speed_trial", caps=candidate, sector=sid,
                                       sector_name=sector["name"], clean_times=[])))
    if not options:
        return dict(kind="evaluation", caps=model["accepted_caps"][:], sector=None)
    proposal = max(options, key=lambda item:item[0])[1]
    model["pending"] = proposal
    return proposal


def learn(model, proposal, result):
    """Promote only repeated clean, faster trials; never claim monotonic gains."""
    metric = result["comparison_time_s"]
    valid_time = metric is not None and math.isfinite(metric) and metric > 0
    clean = valid_time and result["finish_supported"] and result["max_damage"] == 0 and result["max_abs_track_pos"] < .75
    decision = "no_update_incomplete"
    if proposal["kind"] == "calibration":
        if clean:
            model["calibration_clean_times"].append(result["comparison_time_s"])
            decision = "calibration_recorded"
            if len(model["calibration_clean_times"]) >= 2:
                model["accepted_metric_s"] = statistics.median(model["calibration_clean_times"][-2:])
                decision = "calibration_complete"
    elif proposal["kind"] in ("speed_trial", "straight_trial"):
        sid = str(proposal["sector"])
        model["attempts_by_sector"][sid] = model["attempts_by_sector"].get(sid, 0)+1
        if clean:
            proposal.setdefault("clean_times", []).append(result["comparison_time_s"])
            decision = "repeat_candidate"
            model["pending"] = proposal
            if len(proposal["clean_times"]) >= 2:
                metric = statistics.median(proposal["clean_times"][-2:])
                if metric < model["accepted_metric_s"] - .2:
                    model["accepted_caps"] = proposal["caps"][:]
                    model["accepted_metric_s"] = metric
                    model["generation"] += 1
                    decision = "accepted_repeated_clean_gain"
                else:
                    decision = "rejected_no_measured_gain"
                    model["failures_by_sector"][sid] = model["failures_by_sector"].get(sid, 0)+1
                model["pending"] = None
        elif result["finish_supported"] or result["status"] in ("damage", "off_track", "unstable", "stuck", "reversing"):
            model["failures_by_sector"][sid] = model["failures_by_sector"].get(sid, 0)+1
            model["pending"] = None
            decision = "rejected_unstable_accepted_policy_preserved"
        # A timeout/interruption does not establish whether a candidate works.
        if proposal["kind"] == "straight_trial":
            limit = proposal["straight_limit_kmh"]
            if decision == "accepted_repeated_clean_gain":
                model["accepted_straight_limit_kmh"] = limit
                model["next_straight_limit_kmh"] = min(200.0,limit+20)
            elif decision.startswith("rejected"):
                accepted = model.get("accepted_straight_limit_kmh",60.0)
                model["straight_upper_limit_kmh"] = min(model.get("straight_upper_limit_kmh",200.0),limit-1)
                model["next_straight_limit_kmh"] = (accepted+limit)/2
    model["history"].append(dict(run_id=result["run_id"], status=result["status"],
                               kind=proposal["kind"], sector=proposal["sector"], clean=clean,
                               comparison_time_s=result["comparison_time_s"], decision=decision))
    return decision


def finish_metrics(geometry, first, latest, coverage, status, top_speed, peak_timer):
    """Validate coverage and reject reset timers or physically impossible times."""
    result = dict(finish_supported=False,comparison_time_s=None,official_lap_time_s=None,
                  timing_method="not completed",simulator_reported_lap_time_s=None)
    if first is None or latest is None:
        return result
    length, start = geometry["length_m"], first["distFromStart"]
    progress = latest["distRaced"]-first["distRaced"]
    needed = length+(length-start if start>.9*length else -start)
    covered = (status=="server_shutdown" and coverage>=.99 and
               (start<20 or start>.9*length) and progress>=needed-2)
    if not covered or not top_speed or top_speed<=0:
        return result
    # Lap time cannot be substantially below distance / fastest observed speed.
    # Small tolerance accounts for start-line placement and sample discretisation.
    lower_bound = .95*length/(top_speed/3.6)-1.0
    reported = latest["lastLapTime"]
    if reported > 0 and reported >= lower_bound:
        time_s, method = reported, "simulator lastLapTime after validated full-distance finish"
        result["simulator_reported_lap_time_s"] = reported
    elif latest["curLapTime"] >= lower_bound and latest["curLapTime"] >= peak_timer-.1:
        time_s, method = latest["curLapTime"], "last sensor timer at full-distance shutdown; approximate, verify TORCS result"
    else:
        result["timing_method"] = "unverified: reset or impossible finish timer; excluded from learning"
        return result
    result.update(finish_supported=True,comparison_time_s=time_s,timing_method=method)
    return result


def rebuild_timing_reference(original, runs_root):
    """Replay retained evidence into a new model; never rewrite historical logs."""
    history = original.get("history",[])
    if not history:
        repaired = copy.deepcopy(original)
        repaired["timing_validation_version"] = 2
        return repaired
    first_snapshot = Path(runs_root)/history[0]["run_id"]/"model_before.json"
    repaired = json.loads(first_snapshot.read_text())
    repaired.update(history=[],pending=None,accepted_metric_s=None,calibration_clean_times=[],
                    generation=0,failures_by_sector={},attempts_by_sector={})
    corrections = []
    for old in history:
        folder = Path(runs_root)/old["run_id"]
        summary = json.loads((folder/"summary.json").read_text())
        proposal = json.loads((folder/"policy_used.json").read_text())
        path = folder/"telemetry.csv"
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        if rows:
            first = json.loads(rows[0]["raw_sensors_json"])
            latest = json.loads(rows[-1]["raw_sensors_json"])
            peak = max(float(r["curLapTime"]) for r in rows)
            coverage = len({min(len(repaired['geometry']['bins'])-1,
                        int(float(r['distFromStart'])/repaired['geometry']['bin_size_m'])) for r in rows})/len(repaired['geometry']['bins'])
            summary.update(finish_metrics(repaired["geometry"],first,latest,coverage,
                                         summary["status"],summary["top_speed_kmh"],peak))
        if proposal["kind"] in ("speed_trial","straight_trial"):
            pending = repaired.get("pending")
            proposal["clean_times"] = pending["clean_times"][:] if pending and pending["caps"]==proposal["caps"] else []
        decision = learn(repaired,proposal,summary)
        if old.get("comparison_time_s") != summary["comparison_time_s"] or old.get("decision") != decision:
            corrections.append(dict(run_id=old["run_id"],old_time_s=old.get("comparison_time_s"),
                                    corrected_time_s=summary["comparison_time_s"],decision=decision))
    repaired["timing_validation_version"] = 2
    repaired["timing_corrections"] = corrections
    repaired["repair_note"] = "Recomputed from retained telemetry; original model and run files unchanged"
    return repaired


class LearningController:
    def __init__(self, model, caps):
        self.model, self.geometry = model, model["geometry"]
        self.profile = make_envelope(model, caps)
        self.previous_steer = 0.0

    def act(self, state, dt=.02):
        geometry, speed = self.geometry, max(0, state["speedX"])
        bins, step = geometry["bins"], geometry["bin_size_m"]
        index = int((state["distFromStart"] % geometry["length_m"])/step)
        # Look ahead by reaction distance as well as using the braking envelope.
        preview_bins = math.ceil((speed/3.6*.3+5)/step)
        target = min(self.profile[(index+j)%len(bins)] for j in range(preview_bins+1))
        forward = state["track"][9]
        if forward < 0:
            raise ValueError("Invalid forward ray")
        if forward < 170:
            target = min(target, math.sqrt((25/3.6)**2+2*4*max(0,forward-10))*3.6)
        if abs(state["angle"]) > .12:
            target = min(target, 55)
        if abs(state["angle"]) > .3 or abs(state["trackPos"]) > .4:
            target = min(target, 30)
        kappa = bins[(index+int((speed/3.6*.15)/step))%len(bins)]["curvature"]
        feedforward = .65*math.atan(2.95*kappa)/math.radians(21)
        desired = clamp(feedforward + state["angle"]/math.radians(21) - .5*state["trackPos"], -1, 1)
        rate = 3*clamp(dt,.005,.1)
        steer = clamp(desired,self.previous_steer-rate,self.previous_steer+rate)
        self.previous_steer = steer
        error = target-state["speedX"]
        accel = clamp(error*.08,0,.65) if error > 2 else 0
        brake = clamp(-error*.06,0,.8) if error < -2 else 0
        wheels, radii = state.get("wheelSpinVel"), self.model["wheel_radius_m"]
        if isinstance(wheels,list) and len(wheels)==4 and speed > 20:
            if radii["rear"]:
                slip = (statistics.mean(wheels[2:])*radii["rear"]-speed/3.6)/max(speed/3.6,5)
                if slip > .12:
                    accel *= clamp(1-(slip-.12)*3,0,1)
            if radii["front"] and statistics.mean(wheels[:2])*radii["front"] < .75*speed/3.6:
                brake *= .4
        action = dict(steer=steer,accel=accel,brake=brake,
                      gear=1+sum(speed>v for v in (50,80,110,140,170)),clutch=0,focus=0,meta=0)
        return action, dict(target_speed_kmh=target,bin=index,sector=bins[index]["sector"],curvature=kappa)


def run_episode(args, model, proposal, output):
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"_"+uuid.uuid4().hex[:8]
    run_dir = output/run_id
    run_dir.mkdir(parents=True,exist_ok=False)
    save_json(run_dir/"policy_used.json", proposal)
    save_json(run_dir/"model_before.json", model)
    controller = LearningController(model, proposal["caps"])
    result = dict(run_id=run_id,status="not_connected",finish_supported=False,
                  comparison_time_s=None,official_lap_time_s=None,max_damage=0.0,
                  max_abs_track_pos=0.0,top_speed_kmh=None,samples=0,
                  code_sha256=digest(__file__),track_sha256=model["geometry"]["track_sha256"],
                  timing_method="not completed",proposal_kind=proposal["kind"],sector=proposal["sector"],
                  car_label=args.car,settings_label=args.settings_label,source_model_generation=model["generation"])
    result["model_kind"]="track-specific tabular speed policy with repeated-trial policy search"
    result["sensor_angles_deg"]=ANGLES
    result.update(offtrack_allowed=model.get("allow_offtrack",False),outside_road_samples=0,
                  body_outside_road_samples=0,position_envelope_ok=True,
                  rule_status="Experimental; organiser acceptance of off-track driving not verified")
    result["units"]={"speedX":"km/h","speedY":"km/h","speedZ":"km/h","angle":"radians",
                     "trackPos":"normalised half-track width","distFromStart":"metres","distRaced":"metres",
                     "curLapTime":"seconds","lastLapTime":"seconds","damage":"simulator damage units"}
    first=previous=latest=None
    peak_timer=0.0
    visited=set()
    phase="handshake"
    progress=deque()
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)
    started=time.monotonic()
    fields=["run_id","timestamp_utc","step","simulator_dt_s","status","speedX","speedY","speedZ",
            "curLapTime","lastLapTime","damage","damage_change","trackPos","angle","distFromStart",
            "distRaced","rpm","sensor_gear","target_speed_kmh","bin","sector","curvature",
            "steer","accel","brake","gear","command_datagram_sent","raw_sensors_json"]
    try:
        sock.connect((args.host,args.port))
        init=("SCR(init "+" ".join(map(str,ANGLES))+")").encode()
        for _ in range(args.connection_attempts):
            sock.send(init)
            try:
                packet=sock.recv(2**17).decode()
            except socket.timeout:
                continue
            if "***identified***" in packet:
                break
        else:
            raise TimeoutError("Start a NEW single-car race; no SCR handshake")
        phase="first_sensor"
        deadline=time.monotonic()+args.startup_timeout
        print("Connected. Waiting for sensors; keep TORCS unpaused.")
        with (run_dir/"telemetry.csv").open("x",newline="",encoding="utf-8") as stream:
            writer=csv.DictWriter(stream,fieldnames=fields)
            writer.writeheader()
            result["status"]="running"
            while time.monotonic()-started < args.duration:
                wait=deadline-time.monotonic() if latest is None else args.timeout
                if wait<=0:
                    raise TimeoutError("No first sensor packet")
                sock.settimeout(wait)
                packet=sock.recv(2**17).decode()
                if "***shutdown***" in packet or "***restart***" in packet:
                    result["status"]="server_shutdown" if "shutdown" in packet else "server_restart"
                    break
                if "***identified***" in packet:
                    continue
                phase="sensor_parse"
                state=parse_packet(packet)
                phase="sensor_stream"
                latest=state
                peak_timer=max(peak_timer,state["curLapTime"])
                if first is None:
                    first=state.copy()
                result["samples"]+=1
                result["max_damage"]=max(result["max_damage"],state["damage"])
                result["max_abs_track_pos"]=max(result["max_abs_track_pos"],abs(state["trackPos"]))
                result["outside_road_samples"] += int(abs(state["trackPos"])>1)
                result["body_outside_road_samples"] += int(abs(state["trackPos"])+1.2/(model["geometry"]["width_m"]/2)>1)
                result["position_envelope_ok"] = result["position_envelope_ok"] and position_allowed(model,state)
                result["top_speed_kmh"]=max(result["top_speed_kmh"] or 0,state["speedX"])
                length=model["geometry"]["length_m"]
                if not 0<=state["distFromStart"]<=length+1:
                    raise ValueError("Live track distance disagrees with the learned map")
                visited.add(min(len(model["geometry"]["bins"])-1,int(state["distFromStart"]/model["geometry"]["bin_size_m"])))
                now=time.monotonic()
                progress.append((now,state["distRaced"]))
                while len(progress)>1 and now-progress[1][0]>=5:
                    progress.popleft()
                dt=state["curLapTime"]-previous["curLapTime"] if previous else .02
                if state["damage"]>0:
                    result["status"]="damage"
                elif not position_allowed(model,state):
                    result["status"]="off_track"
                elif abs(state["angle"])>1.0:
                    result["status"]="unstable"
                elif state["speedX"] < -5:
                    result["status"]="reversing"
                elif now-progress[0][0]>=5 and state["distRaced"]-progress[0][1]<1:
                    result["status"]="stuck"
                if result["status"]=="running":
                    action,extras=controller.act(state,dt)
                else:
                    action=dict(steer=0,accel=0,brake=1,gear=1,clutch=0,focus=0,meta=0)
                    extras=dict(target_speed_kmh=None,bin=None,sector=None,curvature=None)
                row={key:state.get(key,"") for key in fields}
                row.update(run_id=run_id,timestamp_utc=datetime.now(timezone.utc).isoformat(),
                           step=result["samples"]-1,simulator_dt_s=dt if dt>0 else "",
                           status=result["status"],sensor_gear=state.get("gear",""),
                           damage_change=state["damage"]-previous["damage"] if previous else "",
                           raw_sensors_json=json.dumps(state,separators=(",",":")),**extras)
                row.update({key:action[key] for key in ("steer","accel","brake","gear")})
                row["command_datagram_sent"]=False
                try:
                    sock.send(encode_action(action))
                    row["command_datagram_sent"]=True
                finally:
                    writer.writerow(row)
                    stream.flush()
                if result["samples"]%500==0:
                    print(f"s={state['distFromStart']:.0f} m speed={state['speedX']:.1f} target={extras['target_speed_kmh']} damage={state['damage']:g}")
                if result["status"]!="running":
                    break
                previous=state
            else:
                result["status"]="duration_limit"
    except KeyboardInterrupt:
        result["status"]="interrupted"
    except (OSError,ValueError) as error:
        result.update(status="connection_or_sensor_error",error=str(error),error_phase=phase)
    finally:
        if first:
            try:
                sock.send(encode_action(dict(steer=0,accel=0,brake=1,gear=1,clutch=0,focus=0,meta=0)))
            except OSError:
                pass
        sock.close()
        result["wall_elapsed_s"]=time.monotonic()-started
        result["coverage"]=len(visited)/len(model["geometry"]["bins"])
        if latest:
            progress_m=latest["distRaced"]-first["distRaced"]
            # This package shuts down just before publishing the final wrap.
            # Infer finish only with whole-circuit coverage, a start near the
            # start line, sufficient forward distance, and server shutdown.
            result.update(distance_progress_m=progress_m,initial_damage=first["damage"],
                          ending_damage=latest["damage"],damage_increase=latest["damage"]-first["damage"],
                          last_observed_timer_s=latest["curLapTime"])
            result.update(finish_metrics(model["geometry"],first,latest,result["coverage"],
                                         result["status"],result["top_speed_kmh"],peak_timer))
        result["decision"]=learn(model,proposal,result)
        save_json(run_dir/"summary.json",result)
        flat={k:v for k,v in result.items() if not isinstance(v,(dict,list))}
        with (run_dir/"summary.csv").open("x",newline="",encoding="utf-8") as stream:
            writer=csv.DictWriter(stream,fieldnames=list(flat))
            writer.writeheader();writer.writerow(flat)
        save_json(run_dir/"model_after.json",model)
    return result


def write_track_svg(geometry,path):
    bins=geometry["bins"]
    xs,ys=[b["x"] for b in bins],[b["y"] for b in bins]
    scale=650/max(max(xs)-min(xs),max(ys)-min(ys))
    def point(b):
        return f"{40+(b['x']-min(xs))*scale:.2f},{730-(b['y']-min(ys))*scale:.2f}"
    lines=['<svg xmlns="http://www.w3.org/2000/svg" width="800" height="800" viewBox="0 0 800 800">',
           '<rect width="800" height="800" fill="#101820"/>',
           '<text x="24" y="28" fill="white" font-family="sans-serif">Corkscrew: green straights, orange corners (geometry only)</text>']
    for a,b in zip(bins,bins[1:]+bins[:1]):
        color="#5ee4a4" if abs(a["curvature"])<.0001 else "#ffb454"
        lines.append(f'<polyline points="{point(a)} {point(b)}" fill="none" stroke="{color}" stroke-width="5"/>')
    lines.append('</svg>')
    Path(path).write_text("\n".join(lines),encoding="utf-8")


def main():
    here=Path(__file__).resolve().parent
    # Works in either gym_torcs or the preparation workspace beside it.
    torcs_root=here.parent/"torcs"
    if not torcs_root.exists():
        raise SystemExit("Cannot locate sibling torcs installation")
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare",action="store_true",help="Create model from a clean run; does not drive")
    parser.add_argument("--model",type=Path)
    parser.add_argument("--fast",action="store_true",help="Use a separate repaired model and learn a circuit-wide straight limit")
    parser.add_argument("--straight-limit",type=float,default=120.0,help="First fast-mode trial ceiling, 65..200 km/h")
    parser.add_argument("--seed-run",type=Path)
    parser.add_argument("--seed-root",type=Path,default=here/"rescue_runs")
    parser.add_argument("--track-xml",type=Path,default=torcs_root/"tracks/road/corkscrew/corkscrew.xml")
    parser.add_argument("--car",default="car1-ow1")
    parser.add_argument("--settings-label",default="Corkscrew one-lap standing start; verify in TORCS")
    parser.add_argument("--episodes",type=int,default=1)
    parser.add_argument("--evaluate",action="store_true",help="Drive the accepted speed policy without proposing increases")
    parser.add_argument("--host",default="127.0.0.1")
    parser.add_argument("--port",type=int,default=3001)
    parser.add_argument("--connection-attempts",type=int,default=20)
    parser.add_argument("--timeout",type=float,default=3)
    parser.add_argument("--startup-timeout",type=float,default=30)
    parser.add_argument("--duration",type=float,default=600)
    parser.add_argument("--output",type=Path,default=here/"learning_runs")
    args=parser.parse_args()
    if args.model is None:
        args.model=here/("corkscrew_fast_model.json" if args.fast else "corkscrew_model.json")
    if not 65<=args.straight_limit<=200:
        parser.error("--straight-limit must be 65..200 km/h")
    if min(args.episodes,args.connection_attempts,args.timeout,args.startup_timeout,args.duration)<=0:
        parser.error("Counts and timeouts must be positive")
    if args.prepare:
        if args.model.exists():
            raise SystemExit("Model already exists; use a different --model path to start a separate experiment")
        geometry=read_geometry(args.track_xml)
        sources=[args.seed_run] if args.seed_run else sorted(args.seed_root.glob("*"),key=lambda p:p.stat().st_mtime,reverse=True)
        failures=[]
        for source in sources:
            try:
                model=seed_from_clean_run(geometry,source)
                break
            except (OSError,ValueError,KeyError) as error:
                failures.append(f"{source}: {error}")
        else:
            raise SystemExit("No suitable clean full-circuit seed found.\n"+"\n".join(failures))
        model["environment"]=dict(car_label=args.car,settings_label=args.settings_label)
        save_json(args.model,model)
        write_track_svg(geometry,args.model.with_suffix(".svg"))
        print(f"Prepared {args.model}: {geometry['length_m']:.3f} m, {len(geometry['bins'])} bins. No new races trained yet.")
        return
    if args.fast and not args.model.exists():
        source=here/"corkscrew_model.json"
        original=json.loads(source.read_text())
        model=rebuild_timing_reference(original,args.output)
        model["pending"]=None
        model.update(coarse_straight_learning=True,accepted_straight_limit_kmh=max(model["accepted_caps"]),
                     next_straight_limit_kmh=args.straight_limit,straight_upper_limit_kmh=200.0,
                     forked_from_model_sha256=digest(source))
        save_json(args.model,model)
        print(f"Created separate fast model. Repaired reference: {model['accepted_metric_s']} s. Original model preserved.")
    model=json.loads(args.model.read_text())
    if model.get("accepted_metric_s") is not None and model["accepted_metric_s"] < .85*model["geometry"]["length_m"]/(max(model["accepted_caps"])/3.6):
        raise SystemExit("Existing comparison time is physically inconsistent. Use --fast to fork a repaired model from saved logs.")
    if model.get("version")!=VERSION or model["geometry"]["track_sha256"]!=digest(args.track_xml):
        raise SystemExit("Model version or track fingerprint mismatch; use a separate model")
    if model["environment"]!={"car_label":args.car,"settings_label":args.settings_label}:
        raise SystemExit("Car/settings labels differ from this model; use a separate model")
    for episode in range(args.episodes):
        proposal=(dict(kind="evaluation",caps=model["accepted_caps"][:],sector=None)
                  if args.evaluate else propose(model))
        # Persist pending trial before launch so interruption cannot silently change it.
        save_json(args.model,model)
        print(f"Episode {episode+1}/{args.episodes}: {proposal['kind']} sector={proposal.get('sector_name')} generation={model['generation']}")
        print("Start a NEW Corkscrew race with one external driver now. Each episode needs a fresh race.")
        result=run_episode(args,model,proposal,args.output)
        save_json(args.model,model)
        print(f"Stopped: {result['status']}; learning decision: {result['decision']}; logs: {args.output/result['run_id']}")
        if result["status"] in ("interrupted","connection_or_sensor_error"):
            break
        if proposal["kind"]=="calibration" and not result["finish_supported"]:
            print("Calibration did not complete. Inspect this run before further attempts; speed learning has not started.")
            break
        if episode+1<args.episodes:
            print("Return to TORCS and select New Race; this client will wait for the next handshake.")
    print("Accepted model saved. Future launches resume it. Faster results are not guaranteed on every trial.")


if __name__=="__main__":
    main()
