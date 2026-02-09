#!/usr/bin/env python3

'''
Run an automated ArduPlane L1 A/B test (feedback OFF vs ON) in SITL.

Artifacts:
- full DataFlash log copy
- full TLOG copy (if available)
- per-segment CSV data (OFF and ON)
- JSON metrics summary (MAE/RMSE/p95/max_out/within10)

AP_FLAKE8_CLEAN
'''

import argparse
import csv
import json
import math
import os
import shutil
import sys

from pymavlink import DFReader
from pymavlink import mavutil

import vehicle_test_suite
from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException
from pysim import util


os.environ['MAVLINK20'] = '1'


def decode_param_name(name):
    if isinstance(name, bytes):
        return name.decode(errors='ignore').strip('\x00')
    return name


def local_distance_m(lat0_deg, lon0_deg, lat_deg, lon_deg):
    radius_earth = 6378137.0
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    dlat = lat - lat0
    dlon = lon - lon0
    east = dlon * math.cos(lat0) * radius_earth
    north = dlat * radius_earth
    return math.hypot(east, north)


def percentile(values, pct):
    if not values:
        return None
    sorted_values = sorted(values)
    idx = (len(sorted_values) - 1) * pct
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return float(sorted_values[lo])
    frac = idx - lo
    return float(sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac)


def compute_metrics(rows):
    if not rows:
        raise NotAchievedException("No rows for segment metrics")
    errors = [row["error_m"] for row in rows]
    abs_errors = [abs(error) for error in errors]
    mean_abs = sum(abs_errors) / len(abs_errors)
    mean_sq = sum(error * error for error in errors) / len(errors)
    return {
        "samples": len(rows),
        "mae_m": mean_abs,
        "rmse_m": math.sqrt(mean_sq),
        "p95_abs_error_m": percentile(abs_errors, 0.95),
        "max_outward_error_m": max(errors),
        "max_inward_error_m": min(errors),
        "within_10m_percent": 100.0 * sum(1 for value in abs_errors if value <= 10.0) / len(abs_errors),
    }


def parse_qgc_wpl(filepath):
    items = []
    with open(filepath, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("QGC WPL"):
                continue
            parts = line.split()
            if len(parts) < 12:
                continue
            items.append({
                "seq": int(parts[0]),
                "command": int(parts[3]),
                "p1": float(parts[4]),
                "p2": float(parts[5]),
                "p3": float(parts[6]),
                "p4": float(parts[7]),
                "lat": float(parts[8]),
                "lon": float(parts[9]),
                "alt": float(parts[10]),
            })
    if not items:
        raise NotAchievedException(f"Could not parse mission file: {filepath}")
    return items


def home_string_from_qgc_wpl(filepath):
    items = parse_qgc_wpl(filepath)
    home = min(items, key=lambda item: item["seq"])
    return f'{home["lat"]},{home["lon"]},{home["alt"]},0'


class RunL1IndiAB(vehicle_test_suite.TestSuite):
    def __init__(
        self,
        vehicle_binary,
        model,
        mission_filepath,
        speedup,
        settle_time,
        off_duration,
        on_duration,
        output_dir,
        sim_speedup_param,
        wind_speed,
        wind_dir,
        wind_tc,
        wind_turb,
        k_tc,
        fb_tc,
        fb_p,
        fb_i,
        fb_d,
        instance,
    ):
        super().__init__(vehicle_binary, speedup=speedup)
        self.model = model
        self.mission_filepath = mission_filepath
        self.settle_time = settle_time
        self.off_duration = off_duration
        self.on_duration = on_duration
        self.output_dir = output_dir
        self.sim_speedup_param = sim_speedup_param
        self.wind_speed = wind_speed
        self.wind_dir = wind_dir
        self.wind_tc = wind_tc
        self.wind_turb = wind_turb
        self.k_tc = k_tc
        self.fb_tc = fb_tc
        self.fb_p = fb_p
        self.fb_i = fb_i
        self.fb_d = fb_d
        self.instance = instance

    def log_name(self):
        return "ArduPlane"

    def vehicleinfo_key(self):
        return "ArduPlane"

    def default_frame(self):
        return "plane"

    def adjust_ardupilot_port(self, port):
        return port + 10 * self.instance

    def sitl_rcin_port(self, offset=0):
        return super().sitl_rcin_port(offset) + 10 * self.instance

    def run(self):
        defaults = self.model_defaults_filepath(self.model, vehicleinfo_key="ArduPlane")
        sitl_home = home_string_from_qgc_wpl(self.mission_filepath)

        self.start_SITL(
            binary=self.binary,
            model=self.model,
            sitl_home=sitl_home,
            speedup=self.speedup,
            defaults_filepath=defaults,
            customisations=[f"-I{self.instance}"],
        )
        self.get_mavlink_connection_going()

        self.set_parameters({
            "SIM_SPEEDUP": self.sim_speedup_param,
            "SIM_WIND_DIR": 360,
            "SIM_WIND_SPD": 8,
            "SIM_WIND_TC": 2.0,
            "SIM_WIND_TURB": 0.5,
            "NAVL1_LACC_K_TC": self.k_tc,
            "NAVL1_LACC_FB_TC": self.fb_tc,
            "NAVL1_LACC_FB_P": 0.0,
            "NAVL1_LACC_FB_I": 0.0,
            "NAVL1_LACC_FB_D": 0.0,
        })

        self.load_mission_from_filepath(self.mission_filepath, strict=False)
        self.wait_ready_to_arm()
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.wait_current_waypoint(2, timeout=240)
        self.progress("At loiter waypoint, starting settle period")
        self.delay_sim_time(self.settle_time)

        self.progress("Starting OFF segment (feedback disabled)")
        self.set_parameters({
            "SIM_WIND_DIR": self.wind_dir,
            "SIM_WIND_SPD": self.wind_speed,
            "SIM_WIND_TC": self.wind_tc,
            "SIM_WIND_TURB": self.wind_turb,
            "NAVL1_LACC_FB_P": 0.0,
            "NAVL1_LACC_FB_I": 0.0,
            "NAVL1_LACC_FB_D": 0.0,
        })
        self.delay_sim_time(self.off_duration)

        self.progress("Starting ON segment (feedback enabled)")
        self.set_parameters({
            "NAVL1_LACC_FB_P": self.fb_p,
            "NAVL1_LACC_FB_I": self.fb_i,
            "NAVL1_LACC_FB_D": self.fb_d,
        })
        self.delay_sim_time(self.on_duration)

        self.change_mode("RTL")
        self.delay_sim_time(10)

        log_filepath = self.current_onboard_log_filepath()
        self.progress(f"Using onboard log: {log_filepath}")

        # Force reboot to close/flush the current DataFlash log before parsing.
        self.reboot_sitl(force=True, check_position=False, mark_context=False)
        return log_filepath

    def extract_ab_artifacts(self, log_filepath):
        off_rows, on_rows, summary = self._extract_segments(log_filepath)

        os.makedirs(self.output_dir, exist_ok=True)

        off_csv = os.path.join(self.output_dir, "l1_indi_ab_off.csv")
        on_csv = os.path.join(self.output_dir, "l1_indi_ab_on.csv")
        summary_json = os.path.join(self.output_dir, "l1_indi_ab_summary.json")
        df_copy = os.path.join(self.output_dir, "l1_indi_ab_full.bin")
        tlog_copy = os.path.join(self.output_dir, "l1_indi_ab_full.tlog")

        self._write_rows_csv(off_csv, off_rows)
        self._write_rows_csv(on_csv, on_rows)
        with open(summary_json, "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)

        shutil.copy2(log_filepath, df_copy)
        buildlog_path = getattr(self, "buildlog", None)
        if buildlog_path and os.path.exists(buildlog_path):
            shutil.copy2(buildlog_path, tlog_copy)

        self.progress(f"Wrote OFF segment: {off_csv}")
        self.progress(f"Wrote ON segment: {on_csv}")
        self.progress(f"Wrote summary: {summary_json}")
        self.progress(f"Copied DataFlash log: {df_copy}")
        if os.path.exists(tlog_copy):
            self.progress(f"Copied TLOG: {tlog_copy}")

        return {
            "off_csv": off_csv,
            "on_csv": on_csv,
            "summary_json": summary_json,
            "dataflash_log": df_copy,
            "tlog": tlog_copy if os.path.exists(tlog_copy) else None,
        }

    @staticmethod
    def _write_rows_csv(path, rows):
        fieldnames = ["time_s", "radius_m", "radius_des_m", "error_m", "e2t"]
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)

    def _extract_segments(self, log_filepath):
        loiter_lat = None
        loiter_lon = None
        loiter_radius = None
        wind18_t = None
        ctrl_on_t = None
        rtl_t = None
        wind_spd_prev = None
        fb_p_prev = None
        e2t_by_timeus = {}
        pos_records = []

        dfreader = self.dfreader_for_path(log_filepath)
        while True:
            message = dfreader.recv_match()
            if message is None:
                break

            message_type = message.get_type()
            time_us = int(getattr(message, "TimeUS", 0))
            time_s = time_us * 1.0e-6

            if message_type == "MISE" and int(getattr(message, "CId", -1)) == 17 and loiter_lat is None:
                loiter_lat = float(message.Lat)
                loiter_lon = float(message.Lng)
                loiter_radius = abs(float(message.Prm3))
                continue

            if message_type == "CTUN":
                e2t_by_timeus[time_us] = float(getattr(message, "E2T", 1.0))
                continue

            if message_type == "POS":
                pos_records.append((time_us, time_s, float(message.Lat), float(message.Lng)))
                continue

            if message_type == "PARM":
                name = decode_param_name(getattr(message, "Name", ""))
                value = float(getattr(message, "Value", 0.0))
                if name == "SIM_WIND_SPD":
                    if wind_spd_prev is None or abs(value - wind_spd_prev) > 1.0e-6:
                        if math.isclose(value, self.wind_speed, rel_tol=0.0, abs_tol=1.0e-3):
                            wind18_t = time_s
                        wind_spd_prev = value
                elif name == "NAVL1_LACC_FB_P":
                    if fb_p_prev is None or abs(value - fb_p_prev) > 1.0e-6:
                        if value > 1.0e-6 and (fb_p_prev is None or fb_p_prev <= 1.0e-6):
                            ctrl_on_t = time_s
                        fb_p_prev = value
                continue

            if message_type == "MODE":
                mode_number = int(getattr(message, "Mode", -1))
                if mode_number == 11 and rtl_t is None:
                    rtl_t = time_s

        if loiter_lat is None or loiter_lon is None or loiter_radius is None:
            mission_items = parse_qgc_wpl(self.mission_filepath)
            for item in mission_items:
                if item["command"] == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM:
                    loiter_lat = item["lat"]
                    loiter_lon = item["lon"]
                    loiter_radius = abs(item["p3"])
                    break

        if loiter_lat is None or loiter_lon is None or loiter_radius is None:
            raise NotAchievedException("Could not determine loiter center/radius from log or mission")
        if wind18_t is None:
            raise NotAchievedException("Could not find OFF segment start (wind step) in log")
        if ctrl_on_t is None:
            raise NotAchievedException("Could not find ON segment start (FB_P change) in log")
        if rtl_t is None:
            if not pos_records:
                raise NotAchievedException("No POS records in onboard log")
            rtl_t = pos_records[-1][1]

        off_rows = []
        on_rows = []
        for time_us, time_s, lat, lon in pos_records:
            e2t = e2t_by_timeus.get(time_us)
            if e2t is None:
                continue
            radius_m = local_distance_m(loiter_lat, loiter_lon, lat, lon)
            radius_des_m = loiter_radius * (e2t ** 2)
            row = {
                "time_s": round(time_s, 3),
                "radius_m": radius_m,
                "radius_des_m": radius_des_m,
                "error_m": radius_m - radius_des_m,
                "e2t": e2t,
            }
            if wind18_t <= time_s < ctrl_on_t:
                off_rows.append(row)
            elif ctrl_on_t <= time_s < rtl_t:
                on_rows.append(row)

        if not off_rows:
            raise NotAchievedException("OFF segment produced no samples")
        if not on_rows:
            raise NotAchievedException("ON segment produced no samples")

        off_metrics = compute_metrics(off_rows)
        on_metrics = compute_metrics(on_rows)

        summary = {
            "source_log": log_filepath,
            "mission": self.mission_filepath,
            "loiter_center": {"lat": loiter_lat, "lon": loiter_lon},
            "loiter_radius_cmd_m": loiter_radius,
            "events_s": {
                "off_start_wind_step": wind18_t,
                "on_start_fb_enable": ctrl_on_t,
                "on_end_rtl": rtl_t,
            },
            "segments": {
                "off": off_metrics,
                "on": on_metrics,
            },
            "improvement_on_vs_off_percent": {
                "mae_m": 100.0 * (on_metrics["mae_m"] - off_metrics["mae_m"]) / off_metrics["mae_m"],
                "rmse_m": 100.0 * (on_metrics["rmse_m"] - off_metrics["rmse_m"]) / off_metrics["rmse_m"],
                "p95_abs_error_m": 100.0 * (on_metrics["p95_abs_error_m"] - off_metrics["p95_abs_error_m"]) / off_metrics["p95_abs_error_m"],
                "max_outward_error_m": 100.0 * (on_metrics["max_outward_error_m"] - off_metrics["max_outward_error_m"]) / off_metrics["max_outward_error_m"],
            },
            "params": {
                "NAVL1_LACC_K_TC": self.k_tc,
                "NAVL1_LACC_FB_TC": self.fb_tc,
                "NAVL1_LACC_FB_P": self.fb_p,
                "NAVL1_LACC_FB_I": self.fb_i,
                "NAVL1_LACC_FB_D": self.fb_d,
                "SIM_WIND_SPD": self.wind_speed,
                "SIM_WIND_DIR": self.wind_dir,
                "SIM_WIND_TC": self.wind_tc,
                "SIM_WIND_TURB": self.wind_turb,
            },
        }
        return off_rows, on_rows, summary


def parse_args():
    parser = argparse.ArgumentParser("run_l1_indi_abtest.py")
    parser.add_argument(
        "--vehicle-binary",
        type=str,
        default=util.reltopdir("build/sitl/bin/arduplane"),
        help="vehicle binary to run",
    )
    parser.add_argument(
        "--model",
        type=str,
        default="plane",
        help="SITL model/frame",
    )
    parser.add_argument(
        "--mission",
        type=str,
        default=util.reltopdir("Tools/autotest/Generic_Missions/CMAC-loiter-unlim.txt"),
        help="mission file path",
    )
    parser.add_argument(
        "--speedup",
        type=float,
        default=5.0,
        help="SITL speedup",
    )
    parser.add_argument(
        "--sim-speedup-param",
        type=float,
        default=5.0,
        help="SIM_SPEEDUP parameter value",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=25.0,
        help="settle time after entering loiter waypoint (seconds sim-time)",
    )
    parser.add_argument(
        "--off-duration",
        type=float,
        default=130.0,
        help="feedback OFF segment duration (seconds sim-time)",
    )
    parser.add_argument(
        "--on-duration",
        type=float,
        default=130.0,
        help="feedback ON segment duration (seconds sim-time)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="output artifact directory (default: ../buildlogs/l1_indi_ab)",
    )
    parser.add_argument("--wind-speed", type=float, default=18.0)
    parser.add_argument("--wind-dir", type=float, default=270.0)
    parser.add_argument("--wind-tc", type=float, default=0.8)
    parser.add_argument("--wind-turb", type=float, default=1.5)
    parser.add_argument("--k-tc", type=float, default=4.0)
    parser.add_argument("--fb-tc", type=float, default=0.35)
    parser.add_argument("--fb-p", type=float, default=0.10)
    parser.add_argument("--fb-i", type=float, default=0.015)
    parser.add_argument("--fb-d", type=float, default=0.04)
    parser.add_argument(
        "--instance",
        type=int,
        default=1,
        help="SITL instance index (-I). Use 0 only if no other SITL is running.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = os.path.join(vehicle_test_suite.TestSuite.buildlogs_dirpath(), "l1_indi_ab")

    runner = RunL1IndiAB(
        vehicle_binary=args.vehicle_binary,
        model=args.model,
        mission_filepath=args.mission,
        speedup=args.speedup,
        settle_time=args.settle_time,
        off_duration=args.off_duration,
        on_duration=args.on_duration,
        output_dir=output_dir,
        sim_speedup_param=args.sim_speedup_param,
        wind_speed=args.wind_speed,
        wind_dir=args.wind_dir,
        wind_tc=args.wind_tc,
        wind_turb=args.wind_turb,
        k_tc=args.k_tc,
        fb_tc=args.fb_tc,
        fb_p=args.fb_p,
        fb_i=args.fb_i,
        fb_d=args.fb_d,
        instance=args.instance,
    )

    log_filepath = None
    try:
        log_filepath = runner.run()
        artifacts = runner.extract_ab_artifacts(log_filepath)
        print("A/B artifacts ready:")
        for key, value in artifacts.items():
            if value is not None:
                print(f"  {key}: {value}")
    except (AutoTestTimeoutException, NotAchievedException) as error:
        print(f"A/B run failed: {error}")
        return 2
    finally:
        if runner.mav is not None:
            runner.mav.close()
            runner.mav = None
        if hasattr(runner, "sitl") and runner.sitl is not None and runner.sitl.isalive():
            runner.stop_SITL()

    return 0


if __name__ == "__main__":
    os.environ['PYTHONUNBUFFERED'] = '1'
    if sys.platform != "darwin":
        os.putenv('TMPDIR', util.reltopdir('tmp'))
    raise SystemExit(main())
