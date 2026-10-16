"""Conservative diagnostic SCR client. Candidate only: not race-validated.

Start a fresh organiser-configured race in TORCS before running this script.
Uses only the standard library; never launches/reconfigures the simulator.
"""

import argparse
from collections import deque
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import platform
from pathlib import Path
import re
import socket
import time
import uuid


ANGLES = [-45, -19, -12, -7, -4, -2.5, -1.7, -1, -.5, 0,
          .5, 1, 1.7, 2.5, 4, 7, 12, 19, 45]


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_packet(packet):
    state = {}
    for name, payload in re.findall(r"\((\w+)\s+([^()]*)\)", packet):
        values = [float(value) for value in payload.split()]
        if not values or not all(math.isfinite(value) for value in values):
            raise ValueError(f"Invalid sensor: {name}")
        state[name] = values[0] if len(values) == 1 else values
    for name in ("angle", "trackPos", "speedX", "damage", "curLapTime",
                 "lastLapTime", "distFromStart", "distRaced", "track"):
        if name not in state:
            raise ValueError(f"Missing required sensor: {name}")
    if not isinstance(state["track"], list) or len(state["track"]) != 19:
        raise ValueError("Expected exactly 19 track sensors")
    return state


class ConservativeController:
    def __init__(self, max_speed=55.0):
        self.max_speed = max_speed
        self.previous_steer = 0.0

    def act(self, state, dt=0.02):
        speed = state["speedX"]
        angle = state["angle"]
        position = state["trackPos"]
        forward = state["track"][9]
        if forward < 0:
            raise ValueError("Forward track ray unavailable; restart on the road")

        # This is a cautious preview heuristic, not a certified corner limit.
        target = min(self.max_speed, 25.0 + 0.6 * forward)
        target = min(target, self.max_speed / (1.0 + 3.0 * abs(angle)))
        if abs(position) > 0.5:
            target = min(target, 30.0)
        target = max(15.0, target)

        # SCR positive angle requires positive steering; positive trackPos
        # is left of centre and therefore requires a negative correction.
        desired = clamp(angle / math.radians(21) - 0.5 * position, -1, 1)
        step_limit = 3.0 * clamp(dt, .005, .1)
        steer = clamp(desired, self.previous_steer - step_limit,
                      self.previous_steer + step_limit)
        self.previous_steer = steer

        error = target - speed
        accel = clamp(error * .08, 0, .45) if error > 2.0 else 0.0
        brake = clamp(-error * .06, 0, .7) if error < -2.0 else 0.0
        # Modest launch demand; no reverse/division-based throttle boost.
        gear = 1 + sum(speed > threshold for threshold in (50, 80, 110, 140, 170))
        return {"steer": steer, "accel": accel, "brake": brake,
                "gear": gear, "clutch": 0, "focus": 0, "meta": 0}, target


def encode_action(action):
    return "".join(f"({key} {value:.6f})" for key, value in action.items()).encode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", "-H", default="127.0.0.1")
    parser.add_argument("--port", "-p", type=int, default=3001)
    parser.add_argument("--max-speed", type=float, default=55.0,
                        help="Candidate speed ceiling in km/h; not a safe-speed guarantee")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0,
                        help="Seconds to wait for the first sensor packet after identification")
    parser.add_argument("--duration", type=float, default=360.0)
    parser.add_argument("--track", default="unknown", help="User-supplied track label; verify in TORCS")
    parser.add_argument("--car", default="unknown", help="User-supplied car label; verify in TORCS")
    parser.add_argument("--start-procedure", choices=("unknown", "standing_start", "flying_lap"),
                        default="unknown")
    parser.add_argument("--notes", default="")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parent / "rescue_runs")
    args = parser.parse_args()
    if not 20 <= args.max_speed <= 100 or args.timeout <= 0 or args.startup_timeout <= 0 or args.duration <= 0:
        parser.error("Use speed 20..100 km/h and positive timeout/duration")

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    output = args.output / run_id
    output.mkdir(parents=True, exist_ok=False)
    metadata = {"run_id": run_id, "code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "host": args.host, "port": args.port, "max_speed_kmh": args.max_speed,
                "sensor_angles_deg": ANGLES, "status": "not_connected",
                "completed_lap_time_s": None, "elapsed_simulator_time_s": None,
                "track_car_settings": "User labels only; not independently identified by this client",
                "track_label": args.track, "car_label": args.car,
                "start_procedure": args.start_procedure, "notes": args.notes,
                "timeout_s": args.timeout, "startup_timeout_s": args.startup_timeout,
                "duration_limit_s": args.duration,
                "python_version": platform.python_version(), "platform": platform.platform(),
                "started_utc": datetime.now(timezone.utc).isoformat(),
                "lap_time_validation": "No completed lap verified",
                "units": {"speedX": "km/h", "speedY": "km/h", "speedZ": "km/h",
                          "angle": "radians", "trackPos": "normalised half-track width",
                          "track": "metres", "distFromStart": "metres", "distRaced": "metres",
                          "curLapTime": "seconds", "lastLapTime": "seconds",
                          "wheelSpinVel": "radians/second", "rpm": "revolutions/minute",
                          "damage": "simulator damage units", "steer": "normalised [-1,1]",
                          "accel": "normalised [0,1]", "brake": "normalised [0,1]"}}
    config = {key: metadata[key] for key in ("host", "port", "max_speed_kmh", "sensor_angles_deg",
              "track_label", "car_label", "start_procedure", "timeout_s", "startup_timeout_s", "duration_limit_s")}
    metadata["config_sha256"] = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    controller = ConservativeController(args.max_speed)
    initial = previous = latest = None
    status = "not_connected"
    events = []
    sample_count = 0
    speed_sum = 0.0
    top_speed = minimum_speed = max_abs_track_pos = peak_damage = None
    outside_track_samples = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)
    fields = ["run_id", "timestamp_utc", "step", "wall_elapsed_s", "simulator_step_dt_s",
              "status", "target_speed_kmh", "angle", "trackPos", "speedX", "speedY", "speedZ",
              "damage", "damage_change", "curLapTime", "lastLapTime", "distFromStart",
              "distRaced", "rpm", "sensor_gear", "racePos", "fuel", "z",
              "forward_track_m", "min_valid_track_m", "track", "wheelSpinVel",
              "steer", "accel", "brake", "gear", "clutch", "focus", "meta",
              "command_datagram_sent", "send_error", "raw_sensors_json"]
    started = time.monotonic()
    phase = "handshake"
    try:
        sock.connect((args.host, args.port))
        init = ("SCR(init " + " ".join(map(str, ANGLES)) + ")").encode()
        # Bounded handshake: no process killing, hidden relaunch or flag changes.
        for attempt in range(5):
            sock.send(init)
            try:
                reply = sock.recv(2**17).decode()
            except socket.timeout:
                continue
            if "***identified***" in reply:
                break
        else:
            raise TimeoutError("No SCR handshake. Start a fresh race with the external driver and check the port.")
        print(f"Connected. Candidate ceiling {args.max_speed:g} km/h. Logs: {output}")
        print("TORCS identified this client. Waiting for race sensors: ensure the race is unpaused "
              "and only ONE external server driver is selected for this single-client test.")
        status = "running"
        phase = "first_sensor"
        started = time.monotonic()
        first_sensor_deadline = started + args.startup_timeout
        with (output / "telemetry.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            step = 0
            progress_window = deque()
            while time.monotonic() - started < args.duration:
                wait = first_sensor_deadline - time.monotonic() if latest is None else args.timeout
                if wait <= 0:
                    raise TimeoutError("First sensor deadline expired")
                sock.settimeout(wait)
                packet = sock.recv(2**17).decode()
                if "***shutdown***" in packet or "***restart***" in packet:
                    status = "server_shutdown" if "shutdown" in packet else "server_restart"
                    break
                if "***identified***" in packet:
                    continue
                phase = "parse_sensor"
                state = parse_packet(packet)
                latest = state
                phase = "sensor_stream"
                sample_count += 1
                speed_sum += state["speedX"]
                top_speed = state["speedX"] if top_speed is None else max(top_speed, state["speedX"])
                minimum_speed = state["speedX"] if minimum_speed is None else min(minimum_speed, state["speedX"])
                max_abs_track_pos = max(max_abs_track_pos or 0, abs(state["trackPos"]))
                peak_damage = max(peak_damage or 0, state["damage"])
                outside_track_samples += int(abs(state["trackPos"]) > 1)
                if initial is None:
                    initial = state.copy()
                    if state["damage"] != 0:
                        status = "started_with_damage"
                dt = .02 if previous is None else state["curLapTime"] - previous["curLapTime"]
                now = time.monotonic()
                progress_window.append((now, state["distRaced"]))
                while len(progress_window) > 1 and now - progress_window[1][0] >= 5:
                    progress_window.popleft()
                if state["damage"] > 0:
                    status = "damage_detected_restart_required"
                elif abs(state["trackPos"]) > 1:
                    status = "left_track_restart_required"
                elif state["speedX"] < -5:
                    status = "reversing_restart_required"
                elif state["track"][9] < 0:
                    status = "invalid_track_sensor_restart_required"
                elif now - progress_window[0][0] >= 5 and state["distRaced"] - progress_window[0][1] < 1:
                    status = "no_forward_progress_restart_required"
                elif previous and state["curLapTime"] < previous["curLapTime"] and state["lastLapTime"] > 0:
                    # Candidate boundary only; verify full distance and timing externally.
                    status = "lap_boundary_observed_verify_in_torcs"
                    events.append({"lastLapTime": state["lastLapTime"], "distRaced": state["distRaced"]})
                if status == "running":
                    action, target = controller.act(state, dt)
                else:
                    target = None
                    action = dict(steer=0, accel=0, brake=.7, gear=1, clutch=0, focus=0, meta=0)
                row = {name: state.get(name, "") for name in fields}
                for name in ("track", "wheelSpinVel"):
                    if name in state:
                        row[name] = json.dumps(state[name])
                valid_track = [v for v in state["track"] if v >= 0]
                row.update(run_id=run_id, timestamp_utc=datetime.now(timezone.utc).isoformat(),
                           step=step, wall_elapsed_s=time.monotonic()-started,
                           simulator_step_dt_s=dt if previous is not None and dt > 0 else "",
                           status=status, target_speed_kmh=target, sensor_gear=state.get("gear", ""),
                           damage_change=state["damage"] - previous["damage"] if previous else "",
                           forward_track_m=state["track"][9],
                           min_valid_track_m=min(valid_track) if valid_track else "",
                           raw_sensors_json=json.dumps(state, separators=(",", ":")),
                           **action)
                # A successful UDP send does not acknowledge simulator execution.
                # Log even if sending fails, so the final sensor sample survives.
                try:
                    sock.send(encode_action(action))
                    row["command_datagram_sent"] = True
                except OSError as error:
                    row["command_datagram_sent"] = False
                    row["send_error"] = str(error)
                    row["status"] = "command_send_failed"
                    raise
                finally:
                    writer.writerow(row)
                    stream.flush()
                if step % 250 == 0:
                    print(f"distance={state['distFromStart']:.1f} speed={state['speedX']:.1f} "
                          f"position={state['trackPos']:.3f} angle={state['angle']:.3f} damage={state['damage']:g}")
                if status != "running":
                    break
                previous = state
                step += 1
            else:
                status = "duration_limit"
    except KeyboardInterrupt:
        status = "interrupted"
    except TimeoutError as error:
        if phase == "handshake":
            status = "handshake_timeout"
            hint = "No identification received. Start New Race with the correct external driver and verify host/port."
        elif latest is None:
            status = "first_sensor_timeout"
            hint = ("TORCS identified the client but sent no race sensors. Unpause/start the race; "
                    "select only ONE external server driver, or connect a client for every selected server car. "
                    "Stop old clients and start a fresh race before retrying.")
        else:
            status = "sensor_stream_timeout"
            hint = "Sensor updates stopped. Check whether TORCS was paused, closed or waiting on another car."
        metadata.update(error=str(error), error_phase=phase, troubleshooting=hint)
        print(hint)
    except (OSError, ValueError, TimeoutError) as error:
        status = "communication_or_sensor_error"
        metadata.update(error=str(error), error_phase=phase)
        print(str(error))
    finally:
        # Best effort only; on timeout the server may continue applying its last command.
        if initial is not None:
            shutdown_action = dict(steer=0, accel=0, brake=1, gear=1, clutch=0, focus=0, meta=0)
            metadata["shutdown_action"] = shutdown_action
            try:
                sock.send(encode_action(shutdown_action))
                metadata["shutdown_brake_datagram_sent"] = True
            except OSError as error:
                metadata["shutdown_brake_datagram_sent"] = False
                metadata["shutdown_send_error"] = str(error)
        sock.close()
        metadata.update(status=status, lap_boundary_events=events,
                        ended_utc=datetime.now(timezone.utc).isoformat(),
                        wall_elapsed_s=time.monotonic()-started, sample_count=sample_count,
                        top_speed_kmh=top_speed, minimum_speed_kmh=minimum_speed,
                        mean_observed_speed_kmh=speed_sum / sample_count if sample_count else None,
                        max_abs_track_position=max_abs_track_pos, outside_track_samples=outside_track_samples,
                        peak_damage=peak_damage,
                        observed_last_lap_time_s=latest["lastLapTime"] if latest else None)
        if latest is not None:
            metadata.update(initial_damage=initial["damage"], ending_damage=latest["damage"],
                            final_distance_m=latest["distFromStart"],
                            distance_progress_m=latest["distRaced"]-initial["distRaced"],
                            damage_increase=latest["damage"]-initial["damage"],
                            elapsed_simulator_time_s=latest["curLapTime"])
        (output / "summary.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        summary_fields = ["run_id", "status", "started_utc", "ended_utc", "wall_elapsed_s",
                          "elapsed_simulator_time_s", "completed_lap_time_s", "observed_last_lap_time_s",
                          "lap_time_validation", "initial_damage", "ending_damage", "damage_increase",
                          "peak_damage", "top_speed_kmh", "minimum_speed_kmh", "mean_observed_speed_kmh",
                          "max_abs_track_position", "outside_track_samples", "distance_progress_m",
                          "sample_count", "max_speed_kmh", "track_label", "car_label", "start_procedure",
                          "code_sha256", "config_sha256", "error"]
        with (output / "summary.csv").open("x", newline="", encoding="utf-8") as stream:
            summary_writer = csv.DictWriter(stream, fieldnames=summary_fields)
            summary_writer.writeheader()
            summary_writer.writerow({key: metadata.get(key, "") for key in summary_fields})
        print(f"Stopped: {status}. Return to TORCS and pause/end this race. Summary: {output / 'summary.json'}")


if __name__ == "__main__":
    main()
