# =============================================================================
# DashDisplay: Software-Defined Automotive Instrument Cluster
#
# Developed at the Department of Electrical and Computer Engineering,
# University of Peloponnese.
#
# Copyright (c) 2026 Georgios Kourtis, Paris Kitsos
# Licensed under the MIT License.
# =============================================================================


import json
import math
import os
import queue
import re
import subprocess
import threading
import time
import webbrowser
from datetime import datetime
import tkinter as tk
from tkinter import messagebox, ttk

from PIL import Image, ImageTk
import RPi.GPIO as GPIO
import can
from gps3 import gps3
import mysql.connector
import spidev

CONFIG_PATH = "/root/DashDisplay/can_config.json"


# ADC Reader for MCP3008
class MCP3008:
    def __init__(self, bus=0, device=1):  # CE1 for MCP3008
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1350000

    def read_channel(self, channel):
        assert 0 <= channel <= 7
        r = self.spi.xfer2([1, (8 + channel) << 4, 0])
        raw_val = ((r[1] & 3) << 8) + r[2]
        return raw_val

    def get_voltage(self, channel):
        adc_val = self.read_channel(channel)
        return adc_to_voltage(adc_val)


def adc_to_voltage(adc_value, vref=3.3):
    return vref * adc_value / 1023.0


# --- MAX31855 Helper Class (Software SPI) ---
class MAX31855_SoftSPI:
    def __init__(self, cs_pin, clk_pin, do_pin):
        self.cs_pin = cs_pin
        self.clk_pin = clk_pin
        self.do_pin = do_pin

        GPIO.setwarnings(False)
        try:
            GPIO.setmode(GPIO.BCM)
        except Exception:
            pass

        try:
            GPIO.setup(self.cs_pin, GPIO.OUT)
            GPIO.setup(self.clk_pin, GPIO.OUT)
            GPIO.setup(self.do_pin, GPIO.IN)
        except Exception as e:
            print(f"⚠️ SPI Setup Warning: {e}")

        GPIO.output(self.cs_pin, GPIO.HIGH)
        GPIO.output(self.clk_pin, GPIO.LOW)

    def read_temp(self):
        GPIO.output(self.cs_pin, GPIO.LOW)
        raw = 0
        for i in range(32):
            GPIO.output(self.clk_pin, GPIO.HIGH)
            if GPIO.input(self.do_pin):
                raw |= 1 << (31 - i)
            GPIO.output(self.clk_pin, GPIO.LOW)
        GPIO.output(self.cs_pin, GPIO.HIGH)

        if raw & 0x10000:
            return None

        temp_data = (raw >> 18) & 0x3FFF
        if raw & 0x80000000:
            temp_data -= 16384
        return temp_data * 0.25


# Initialize CAN bus
try:
    bus = can.interface.Bus(
        channel="can0", interface="socketcan", bitrate=500000
    )
except Exception as e:
    print(f"❌ Failed to initialize CAN bus: {e}")
    bus = None


def load_can_config():
    cfg_path = CONFIG_PATH
    default_config = {
        "parameters": {},
        "warnings": {},
        "inputs": {"odometer": 0.0, "channels": {}},
        "display": {
            "columns": {
                "left": {"mode": "6small", "tiles": []},
                "middle": {"mode": "2big", "tiles": []},
                "right": {"mode": "6small", "tiles": []},
            }
        },
    }

    if not os.path.exists(cfg_path):
        return default_config

    try:
        with open(cfg_path, "r") as f:
            config = json.load(f)
    except Exception as e:
        print("⚠️ Error loading JSON → using defaults:", e)
        return default_config

    for key, val in default_config.items():
        if key not in config:
            config[key] = val

    if "inputs" not in config or not isinstance(config["inputs"], dict):
        config["inputs"] = default_config["inputs"]

    if "channels" not in config["inputs"] or not isinstance(
        config["inputs"]["channels"], dict
    ):
        config["inputs"]["channels"] = {}

    if "odometer" not in config["inputs"]:
        config["inputs"]["odometer"] = 0.0

    for ch, cfg in config["inputs"]["channels"].items():
        if "unit" not in cfg:
            cfg["unit"] = ""

    disp = config.get("display")
    if not isinstance(disp, dict):
        config["display"] = default_config["display"]
        disp = config["display"]

    cols = disp.get("columns")
    if not isinstance(cols, dict):
        cols = {"left": [], "middle": [], "right": []}
        disp["columns"] = cols

    def normalize_old_block_list(block_list):
        tiles = []
        for b in block_list:
            if not isinstance(b, dict):
                continue
            btype = b.get("type")
            if btype == "small" and "signal" in b:
                tiles.append({"type": "small", "signal": b["signal"]})
            elif btype == "big" and "signal" in b:
                tiles.append({"type": "big", "signal": b["signal"]})
            elif btype == "split" and "signal1" in b and "signal2" in b:
                tiles.append({
                    "type": "split",
                    "signal1": b["signal1"],
                    "signal2": b["signal2"],
                })
        return tiles

    for col_name in ["left", "middle", "right"]:
        col_cfg = cols.get(col_name)
        if isinstance(col_cfg, list):
            tiles = normalize_old_block_list(col_cfg)
            has_big = any(t.get("type") == "big" for t in tiles)
            mode = "2big" if has_big else "6small"
            cols[col_name] = {"mode": mode, "tiles": tiles}
            continue

        if isinstance(col_cfg, dict):
            mode = col_cfg.get("mode", "6small")
            if mode not in ("2big", "6small"):
                mode = "6small"
            tiles = col_cfg.get("tiles")
            if tiles is None:
                tiles = []
            norm_tiles = []
            for t in tiles:
                if not isinstance(t, dict):
                    continue
                ttype = t.get("type", "small")
                if ttype == "big" and t.get("signal"):
                    norm_tiles.append({"type": "big", "signal": t["signal"]})
                elif ttype == "small" and t.get("signal"):
                    norm_tiles.append({"type": "small", "signal": t["signal"]})
                elif ttype == "split" and t.get("signal1") and t.get("signal2"):
                    norm_tiles.append({
                        "type": "split",
                        "signal1": t["signal1"],
                        "signal2": t["signal2"],
                    })
            cols[col_name] = {"mode": mode, "tiles": norm_tiles}
            continue

        cols[col_name] = {
            "mode": default_config["display"]["columns"][col_name]["mode"],
            "tiles": [],
        }

    return config


def save_can_config(config):
    try:
        with open(CONFIG_PATH, "w") as file:
            json.dump(config, file, indent=4)
        print("💾 Saved →", CONFIG_PATH)
    except Exception as e:
        print("❌ Error saving config:", e)


def extract_can_signal(
    data: bytes,
    start_bit: int,
    type_str: str,
    byte_order: str,
    adj_factor: str | None = None,
):
    if not isinstance(data, (bytes, bytearray)):
        data = bytes(data)

    t = (type_str or "UInt16").strip()
    bt = t.lower()

    if bt == "bit":
        bit_length = 1
    elif bt in ("uint8", "int8"):
        bit_length = 8
    elif bt in ("uint16", "int16"):
        bit_length = 16
    else:
        raise ValueError(f"Unknown type: {type_str}")

    byte_order = (byte_order or "LSB").upper()
    raw = 0

    if bit_length == 1:
        byte_index = start_bit // 8
        bit_in_byte = start_bit % 8
        if byte_index >= len(data):
            raw = 0
        else:
            b = data[byte_index]
            raw = (b >> bit_in_byte) & 0x01
    else:
        num_bytes = bit_length // 8
        byte_index = start_bit // 8
        if byte_index + num_bytes > len(data):
            return 0

        segment = data[byte_index : byte_index + num_bytes]
        if byte_order == "MSB":
            segment = segment[::-1]

        raw = int.from_bytes(segment, byteorder="little", signed=False)
        if bt.startswith("int"):
            sign_bit = 1 << (bit_length - 1)
            if raw & sign_bit:
                raw -= 1 << bit_length

    value = raw
    expr = (adj_factor or "").strip()
    if expr:
        full_expr = f"raw{expr}" if expr[0] in "+-*/" else expr
        try:
            value = eval(full_expr, {"__builtins__": {}}, {"raw": raw})
        except Exception as e:
            print(f"⚠️ Error evaluating adj_factor '{adj_factor}': {e}")
            value = raw

    if isinstance(value, float):
        return round(value, 3)
    return value


def manage_service(service_name, action):
    try:
        subprocess.run(["sudo", "systemctl", action, service_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to {action} service '{service_name}': {e}")


def haversine_distance(lat1, lon1, lat2, lon2):
    """Υπολογίζει την απόσταση σε μέτρα μεταξύ δύο συντεταγμένων GPS"""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(delta_phi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


class CanDashboard:
    def __init__(self, root, data):
        self.root = root
        self.root.withdraw()

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()

        self.root.title("Modern Automotive Dashboard")
        self.root.configure(bg="black")
        self.root.geometry(f"{sw}x{sh}+0+0")
        self.root.attributes("-fullscreen", True)
        self.root.bind(
            "<Escape>", lambda e: self.root.attributes("-fullscreen", False)
        )

        self.data = data
        self.limits = {item["name"]: {"low": 0, "high": 100} for item in data}
        self.sensor_values = {item["name"]: None for item in data}
        self.peak_values = {item["name"]: -9999.0 for item in data}
        self.view_mode = "normal"

        self.can_config = load_can_config()
        self.units = {item["name"]: "" for item in data}

        for param, cfg in self.can_config.get("parameters", {}).items():
            if "unit" in cfg:
                self.units[param] = cfg["unit"]

        self.units["G_Speed"] = "km/h"
        self.units["Fuel Level"] = "%"
        self.units["Odometer"] = "km"
        self.units["EGT"] = "°C"
        self.units["Latitude"] = "°"
        self.units["Longitude"] = "°"
        self.units["V_error"] = "km/h"

        for extra_name in [
            "Odometer",
            "G_Speed",
            "V_error",
            "Longitude",
            "Latitude",
            "Fuel Level",
            "EGT",
        ]:
            if extra_name not in self.sensor_values:
                self.sensor_values[extra_name] = 0.0
            if extra_name not in self.peak_values:
                self.peak_values[extra_name] = 0.0

        self.v_error_sum_sq = 0.0
        self.v_error_count = 0

        self.label_references = {}

        self.initialize_gpsd()
        self.adc = MCP3008()
        self.egt_sensor = MAX31855_SoftSPI(cs_pin=6, clk_pin=13, do_pin=5)
        self.sensor_values["EGT"] = 0.0

        self.is_lap_active = False
        self.lap_start_time = None
        self.start_finish_line = None
        self.last_position_crossed = False
        self.lap_times = []

        self.setup_digital_pins()

        self.odometer_distance = float(
            self.can_config["inputs"].get("odometer", 100.0)
        )
        self.last_saved_km = int(self.odometer_distance)
        self.last_update_time = time.time()
        self.sensor_values["Odometer"] = self.odometer_distance

        self.style = ttk.Style()
        self.style.configure("TFrame", background="black")
        self.style.configure(
            "TLabel", background="black", foreground="white", font=("Helvetica", 12)
        )
        self.style.configure("Alert.TFrame", background="red")
        self.style.configure(
            "Alert.TLabel", background="red", foreground="white", font=("Helvetica", 12)
        )

        self.connected_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/connected.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.disconnected_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/disconnected.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.logging_enabled_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/logging_enabled.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.logging_disabled_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/logging_disabled.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.settings_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/settings.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.no_fix_img = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/no_fix_gps.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.fix_2d_img = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/2d_fix_gps.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.fix_3d_img = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/3d_fix_gps.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.wifi_online_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/online.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.wifi_offline_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/offline.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.flag_inactive_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/flag_inactive.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.flag_active_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/flag_active.png").resize(
                (70, 70), Image.LANCZOS
            )
        )
        self.fault_icon = ImageTk.PhotoImage(
            Image.open("/root/DashDisplay/Icons/fault_code.png").resize(
                (70, 70), Image.LANCZOS
            )
        )

        self.icons_frame = tk.Frame(root, bg="black", height=100)
        self.icons_frame.pack(fill="x", padx=10, pady=(0))

        self.warning_images = {
            "traction": {
                "inactive": ImageTk.PhotoImage(
                    Image.open(
                        "/root/DashDisplay/Icons/!traction_icon.png"
                    ).resize((35, 35), Image.LANCZOS)
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/traction_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "lowfuel": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!lowfuel_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/lowfuel_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "check": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!check_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/check_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "abs": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!abs_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/abs_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "airbag": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!airbag_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/airbag_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "steering": {
                "inactive": ImageTk.PhotoImage(
                    Image.open(
                        "/root/DashDisplay/Icons/!steering_icon.png"
                    ).resize((35, 35), Image.LANCZOS)
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/steering_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "oil": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!oil_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/oil_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "battery": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!battery_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/battery_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "temp": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!temp_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/temp_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "doors": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!doors_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/doors_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "seatbelt": {
                "inactive": ImageTk.PhotoImage(
                    Image.open(
                        "/root/DashDisplay/Icons/!seatbelt_icon.png"
                    ).resize((35, 35), Image.LANCZOS)
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/seatbelt_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
            "ebrake": {
                "inactive": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/!e-brake_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
                "active": ImageTk.PhotoImage(
                    Image.open("/root/DashDisplay/Icons/e-brake_icon.png").resize(
                        (35, 35), Image.LANCZOS
                    )
                ),
            },
        }

        self.warning_icon_refs = {}
        self.last_messages = {}
        self.create_rpm_bar()

        self.logging_status_icon = tk.Label(
            self.icons_frame,
            image=self.logging_disabled_icon,
            bg="black",
            cursor="hand2",
        )
        self.logging_status_icon.pack(side="left", padx=5, pady=5)
        self.logging_status_icon.bind("<Button-1>", self.toggle_logging)

        self.wifi_icon_label = tk.Label(
            self.icons_frame,
            image=self.wifi_offline_icon,
            bg="black",
            cursor="hand2",
        )
        self.wifi_icon_label.pack(side="left", padx=5, pady=5)
        self.wifi_icon_label.bind("<Button-1>", lambda e: self.open_wifi_popup())

        center_frame = tk.Frame(self.icons_frame, bg="black")
        center_frame.pack(side="left", expand=True, padx=10)

        left_icons_frame = tk.Frame(center_frame, bg="black")
        left_icons_frame.pack(side="left", padx=5)
        for i, key in enumerate(
            ["traction", "lowfuel", "check", "abs", "airbag", "steering"]
        ):
            row, col = divmod(i, 3)
            icon_frame = tk.Frame(left_icons_frame, bg="black", width=25, height=25)
            icon_frame.grid(row=row, column=col, padx=2, pady=2, sticky="nsew")
            icon_label = tk.Label(
                icon_frame, image=self.warning_images[key]["inactive"], bg="black"
            )
            icon_label.image = self.warning_images[key]["inactive"]
            icon_label.pack(fill="both", expand=True)
            self.warning_icon_refs[key] = icon_label

        self.flag_icon_label = tk.Label(
            center_frame,
            image=self.flag_inactive_icon,
            bg="black",
            cursor="hand2",
        )
        self.flag_icon_label.pack(side="left", padx=5)
        self.flag_icon_label.bind(
            "<Button-1>", lambda event: self.toggle_lap_timer()
        )

        time_display_frame = tk.Frame(center_frame, bg="black")
        time_display_frame.pack(side="left", padx=5)
        self.best_lap_label = tk.Label(
            time_display_frame,
            text="Best Lap: 00:00.000",
            font=("Helvetica", 15, "bold"),
            fg="white",
            bg="black",
        )
        self.best_lap_label.pack()
        self.lap_time_label = tk.Label(
            time_display_frame,
            text="Time Lap: 00:00.000",
            font=("Helvetica", 15, "bold"),
            fg="white",
            bg="black",
        )
        self.lap_time_label.pack()

        right_icons_frame = tk.Frame(center_frame, bg="black")
        right_icons_frame.pack(side="left", padx=5)
        for i, key in enumerate(
            ["oil", "temp", "ebrake", "battery", "doors", "seatbelt"]
        ):
            row, col = divmod(i, 3)
            icon_frame = tk.Frame(right_icons_frame, bg="black", width=25, height=25)
            icon_frame.grid(row=row, column=col, padx=2, pady=2, sticky="nsew")
            icon_label = tk.Label(
                icon_frame, image=self.warning_images[key]["inactive"], bg="black"
            )
            icon_label.image = self.warning_images[key]["inactive"]
            icon_label.pack(fill="both", expand=True)
            self.warning_icon_refs[key] = icon_label

        self.settings_icon_label = tk.Label(
            self.icons_frame, image=self.settings_icon, bg="black", cursor="hand2"
        )
        self.settings_icon_label.pack(side="right", padx=2, pady=5)
        self.settings_icon_label.bind("<Button-1>", self.open_settings_popup)

        self.gps_status_label = tk.Label(
            self.icons_frame, image=self.no_fix_img, bg="black"
        )
        self.gps_status_label.pack(side="right", padx=2, pady=5)

        self.connection_status_icon = tk.Label(
            self.icons_frame, image=self.disconnected_icon, bg="black"
        )
        self.connection_status_icon.pack(side="right", padx=2, pady=5)

        self.settings_popup_open = False
        self.warnings_popup_open = False
        self.inputs_popup_open = False
        self.display_popup_open = False
        self.diagnostics_popup_open = False
        self.active_fault_codes = []
        self.latched_limit_flags = 0

        self.adc_to_ecu_enabled = self.can_config.get("inputs", {}).get(
            "adc_to_ecu_enabled", False
        )
        self.fuel_filtered = None
        self.fuel_alpha = 0.005
        self.wifi_status_loop()
        self.last_can_rx_time = 0
        self.can_connected = False
        self.logging_enabled = False
        self.grafana_db_active = False

        self.grid_frame = ttk.Frame(root)
        self.grid_frame.pack(fill="both", expand=True, padx=20, pady=0)

        self.create_squares(data)
        self.odometer_queue = queue.Queue()
        self.refresh_odometer_label()
        self.ui_update_queue = queue.Queue()
        self.process_ui_queue()

        self.log_queue = queue.Queue()
        self.last_logged_time = time.time()
        self.log_day = None
        self.log_table_name = None
        self.log_can_names = []
        self.log_adc_names = []
        self.log_columns_sql = []
        self.log_insert_query = None
        self.logging_stop_flag = threading.Event()

        self.can_thread = threading.Thread(
            target=self.update_can_data_thread, daemon=True
        )
        self.can_thread.start()
        self.adc_thread = threading.Thread(
            target=self.adc_ecu_worker_thread, daemon=True
        )
        self.adc_thread.start()
        self.gps_thread = threading.Thread(
            target=self.update_gps_data_thread, daemon=True
        )
        self.gps_thread.start()
        self.log_sampler_thread = threading.Thread(
            target=self.logging_sampler_loop, daemon=True
        )
        self.log_sampler_thread.start()
        self.log_writer_thread = threading.Thread(
            target=self.process_log_queue, daemon=True
        )
        self.log_writer_thread.start()
        self.odometer_thread = threading.Thread(
            target=self.process_odometer_queue, daemon=True
        )
        self.odometer_thread.start()

        self.update_gps_status()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        for name, lbl in self.label_references.items():
            lbl.bind("<Button-1>", self.toggle_view_mode)
            frame = lbl.master
            frame.bind("<Button-1>", self.toggle_view_mode)
            for child in frame.winfo_children():
                child.bind("<Button-1>", self.toggle_view_mode)

        self.root.update_idletasks()
        self.root.deiconify()

    def setup_digital_pins(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.pin_map = {
            "ch1": 17,
            "ch2": 27,
            "ch3": 22,
            "ch4": 24,
            "ch8": 23,
        }

        for warn_key, cfg in self.can_config.get("warnings", {}).items():
            if cfg.get("mode") == "digital":
                pin = self.pin_map.get(cfg.get("channel"))
                if pin:
                    pull_mode = (
                        GPIO.PUD_UP if cfg.get("pull") == "UP" else GPIO.PUD_DOWN
                    )
                    GPIO.setup(pin, GPIO.IN, pull_up_down=pull_mode)

        acc_pin = self.pin_map.get("ch8")
        if acc_pin:
            GPIO.setup(acc_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.acc_off_counter = 0
            self.check_acc_power_loop()

    def check_acc_power_loop(self):
        acc_pin = self.pin_map.get("ch8")
        if acc_pin:
            try:
                if GPIO.input(acc_pin) == GPIO.HIGH:
                    self.acc_off_counter += 1
                    if self.acc_off_counter >= 3:
                        print("🛑 Safe Shutdown: ACC power lost detected via Polling")
                        os.system("sudo shutdown -h now")
                else:
                    self.acc_off_counter = 0
            except Exception:
                pass
        self.root.after(1000, self.check_acc_power_loop)

    def create_rpm_bar(self):
        RPM_BAR_HEIGHT = 70
        RPM_FILL_TOP = 10
        RPM_FILL_BOTTOM = RPM_BAR_HEIGHT - 11
        RPM_TEXT_SIZE = 32
        RPM_TICK_FONT = 22
        RPM_ARROW_FONT = 20

        rpm_frame = tk.Frame(self.root, bg="black")
        rpm_frame.pack(fill="x", padx=10)

        self.rpm_canvas = tk.Canvas(
            rpm_frame, height=RPM_BAR_HEIGHT, bg="black", highlightthickness=0
        )
        self.rpm_canvas.pack(fill="x", expand=True)

        self.rpm_canvas_width = self.root.winfo_screenwidth() - 40
        self.rpm_max_rpm = 9000

        self.rpm_bg_bar = self.rpm_canvas.create_rectangle(
            12,
            RPM_FILL_TOP,
            self.rpm_canvas_width,
            RPM_FILL_BOTTOM,
            fill="#404040",
            outline="white",
            width=1,
        )

        for rpm in range(100, self.rpm_max_rpm + 100, 100):
            x = 10 + (self.rpm_canvas_width - 20) * (rpm / self.rpm_max_rpm)
            if rpm % 1000 == 0:
                self.rpm_canvas.create_text(
                    x,
                    RPM_FILL_BOTTOM - 3,
                    text="▲",
                    font=("Arial", RPM_ARROW_FONT),
                    fill="white",
                )
                self.rpm_canvas.create_text(
                    x,
                    RPM_FILL_TOP + 20,
                    text=str(rpm // 1000),
                    font=("Arial", RPM_TICK_FONT, "bold"),
                    fill="white",
                )
            elif rpm % 500 == 0:
                self.rpm_canvas.create_line(
                    x, RPM_FILL_BOTTOM, x, RPM_FILL_BOTTOM - 12, fill="white", width=2
                )
            else:
                self.rpm_canvas.create_line(
                    x, RPM_FILL_BOTTOM, x, RPM_FILL_BOTTOM - 8, fill="gray", width=1
                )

        self.rpm_bar = self.rpm_canvas.create_rectangle(
            13,
            RPM_FILL_TOP + 1,
            13,
            RPM_FILL_BOTTOM - 1,
            fill="#999999",
            outline="",
        )

        self.rpm_text = self.rpm_canvas.create_text(
            5,
            RPM_FILL_BOTTOM + 10,
            text="",
            font=("Arial", RPM_TEXT_SIZE, "bold"),
            fill="white",
            anchor="sw",
        )

    def update_rpm_progress(self, rpm_value):
        rpm_value = max(0, min(rpm_value, self.rpm_max_rpm))
        max_width = self.rpm_canvas_width - 10
        scaled_value = (rpm_value / self.rpm_max_rpm) * max_width

        if hasattr(self, "rpm_canvas") and hasattr(self, "rpm_bar"):
            self.rpm_canvas.coords(self.rpm_bar, 13, 11, 10 + scaled_value, 59)
            threshold = (
                self.can_config["parameters"].get("RPM", {}).get("limit_high", 9000)
            )
            if rpm_value >= threshold:
                self.rpm_canvas.itemconfig(self.rpm_bar, fill="red")
                self.root.after(0, self.blink_rpm_bar)
            elif rpm_value >= (threshold - 500):
                self.rpm_canvas.itemconfig(self.rpm_bar, fill="orange")
            else:
                self.rpm_canvas.itemconfig(self.rpm_bar, fill="#999999")

            self.rpm_canvas.coords(self.rpm_text, -90 + scaled_value, 60)
            self.rpm_canvas.itemconfig(self.rpm_text, text=f"{int(rpm_value)}")

    def blink_rpm_bar(self, count=1):
        def toggle_color(current, remaining):
            if remaining <= 0:
                self.rpm_canvas.itemconfig(self.rpm_bar, fill="red")
                return
            new_color = "#404040" if current == "red" else "red"
            self.rpm_canvas.itemconfig(self.rpm_bar, fill=new_color)
            self.root.after(10, lambda: toggle_color(new_color, remaining - 1))

        toggle_color("red", count)

    def create_square(
        self,
        parent,
        name,
        number,
        unit,
        height=150,
        value_pady=None,
        value_font_size=None,
        name_font_size=None,
        name_pady=None,
        unit_font_size=None,
    ):
        frame = ttk.Frame(parent, relief="ridge", padding=10)
        frame.grid_propagate(False)
        frame.config(width=150, height=height)

        try:
            txt_value = f"{float(number):.1f}"
        except Exception:
            txt_value = "0.0"

        lbl_value = ttk.Label(
            frame,
            text=txt_value,
            font=("Helvetica", value_font_size if value_font_size else 32, "bold"),
        )
        lbl_value.pack(pady=value_pady if value_pady else (20, 5))

        bottom = tk.Frame(frame, bg="black")
        bottom.pack(side="bottom", pady=name_pady if name_pady else (0, 8))

        lbl_name = ttk.Label(
            bottom,
            text=name,
            font=("Helvetica", name_font_size if name_font_size else 16),
        )
        lbl_name.pack(side="left")

        lbl_unit = ttk.Label(
            bottom,
            text=f" {unit}" if unit else "",
            font=("Helvetica", unit_font_size if unit_font_size else 10),
        )
        lbl_unit.pack(side="left")

        self.label_references[name] = lbl_value
        return frame

    def create_split_square(
        self, parent, sig1, number1, unit1, sig2, number2, unit2
    ):
        frame = ttk.Frame(parent, relief="ridge", padding=8)
        frame.grid_propagate(False)
        frame.config(width=150, height=150)

        container = tk.Frame(frame, bg="black")
        container.pack(fill="both", expand=True)

        top_row = tk.Frame(container, bg="black")
        top_row.pack(pady=(10, 2), anchor="center")

        try:
            txt1 = f"{float(number1):.1f}"
        except Exception:
            txt1 = "0.0"

        lbl_val1 = ttk.Label(top_row, text=txt1, font=("Helvetica", 32, "bold"))
        lbl_val1.pack(side="left")
        lbl_u1 = ttk.Label(top_row, text=unit1, font=("Helvetica", 12))
        lbl_u1.pack(side="left")

        sep = ttk.Separator(container, orient="horizontal")
        sep.pack(fill="x", pady=5, expand=True)

        bottom_row = tk.Frame(container, bg="black")
        bottom_row.pack(pady=(2, 10), anchor="center")

        try:
            txt2 = f"{float(number2):.0f}"
        except Exception:
            txt2 = "0"

        lbl_val2 = ttk.Label(bottom_row, text=txt2, font=("Helvetica", 24, "bold"))
        lbl_val2.pack(side="left")
        lbl_u2 = ttk.Label(bottom_row, text=unit2, font=("Helvetica", 10))
        lbl_u2.pack(side="left")

        self.label_references[sig1] = lbl_val1
        self.label_references[sig2] = lbl_val2
        return frame

    def create_squares(self, data):
        display_cfg = self.can_config.get("display", {})
        columns_cfg = display_cfg.get("columns", {})

        for w in self.grid_frame.winfo_children():
            w.destroy()

        initial_values = dict(self.sensor_values)
        for item in data:
            initial_values.setdefault(item["name"], item["number"])

        col_positions = {"left": 0, "middle": 2, "right": 4}

        for col_key, col_start in col_positions.items():
            col_cfg = columns_cfg.get(col_key, {})
            mode = col_cfg.get("mode", "6small")
            tiles = col_cfg.get("tiles", [])

            if mode == "2big":
                for idx in range(2):
                    if idx >= len(tiles):
                        continue
                    tile = tiles[idx]
                    if tile.get("type") != "big":
                        continue
                    sig = tile.get("signal")
                    if not sig:
                        continue

                    num = initial_values.get(sig, 0)
                    unit = self.get_unit_for_signal(sig)
                    row = 0 if idx == 0 else 3

                    frame = self.create_square(
                        self.grid_frame,
                        sig,
                        num,
                        unit,
                        height=220,
                        value_pady=(60, 5),
                        value_font_size=54,
                        name_font_size=22,
                        name_pady=(0, 15),
                        unit_font_size=14,
                    )

                    frame.grid(
                        row=row,
                        column=col_start,
                        rowspan=3,
                        columnspan=2,
                        padx=5,
                        pady=5,
                        sticky="nsew",
                    )
            else:
                small_positions = [
                    (0, col_start),
                    (0, col_start + 1),
                    (2, col_start),
                    (2, col_start + 1),
                    (4, col_start),
                    (4, col_start + 1),
                ]

                for idx, (row, col) in enumerate(small_positions):
                    if idx >= len(tiles):
                        break
                    tile = tiles[idx]
                    ttype = tile.get("type", "small")

                    if ttype == "small":
                        sig = tile.get("signal")
                        if not sig:
                            continue
                        num = initial_values.get(sig, 0)
                        unit = self.get_unit_for_signal(sig)

                        frame = self.create_square(
                            self.grid_frame,
                            sig,
                            num,
                            unit,
                            height=150,
                            value_pady=(20, 5),
                            value_font_size=38,
                            name_font_size=18,
                            name_pady=(0, 8),
                            unit_font_size=12,
                        )
                        frame.grid(
                            row=row, column=col, rowspan=2, padx=5, pady=5, sticky="nsew"
                        )

                    elif ttype == "split":
                        sig1 = tile.get("signal1")
                        sig2 = tile.get("signal2")
                        if not sig1 or not sig2:
                            continue

                        num1 = initial_values.get(sig1, 0)
                        num2 = initial_values.get(sig2, 0)
                        unit1 = self.get_unit_for_signal(sig1)
                        unit2 = self.get_unit_for_signal(sig2)

                        frame = self.create_split_square(
                            self.grid_frame, sig1, num1, unit1, sig2, num2, unit2
                        )
                        frame.grid(
                            row=row, column=col, rowspan=2, padx=5, pady=5, sticky="nsew"
                        )

        for r in range(6):
            self.grid_frame.rowconfigure(r, weight=1)
        for c in range(6):
            self.grid_frame.columnconfigure(c, weight=1, uniform="grid")

    def refresh_odometer_label(self):
        label = self.label_references.get("Odometer")
        if label:
            label.config(text=f"{self.odometer_distance:.0f}")

    def update_odometer(self, speed_kmh):
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time
        self.last_update_time = current_time

        distance_increment = speed_kmh * (elapsed_time / 3600)
        self.odometer_distance += distance_increment
        self.sensor_values["Odometer"] = self.odometer_distance
        self.can_config["inputs"]["odometer"] = self.odometer_distance

        current_km = int(self.odometer_distance)
        if current_km > self.last_saved_km:
            self.odometer_queue.put(self.odometer_distance)
            self.last_saved_km = current_km
            self.root.after(0, self.refresh_odometer_label)

    def update_speed_error(self):
        """Calculates Kinematic Velocity Error Delta: V_error = |V_ECU - V_GPS|"""
        try:
            v_ecu = float(self.sensor_values.get("Speed", 0.0) or 0.0)
            v_gps = float(self.sensor_values.get("G_Speed", 0.0) or 0.0)
            v_error = abs(v_ecu - v_gps)
            self.sensor_values["V_error"] = round(v_error, 2)
            self.ui_update_queue.put(("V_error", self.sensor_values["V_error"]))

            if self.logging_enabled and v_gps > 5.0:
                self.v_error_sum_sq += v_error**2
                self.v_error_count += 1
        except Exception as e:
            print("V_error Calculation Error:", e)

    def update_value(self, name, value):
        self.sensor_values[name] = value

        try:
            val_float = float(value)
            if val_float > self.peak_values.get(name, -9999.0):
                self.peak_values[name] = val_float
        except Exception:
            pass

        if name in ["ECU_Fault", "Limit_Flags"]:
            val_int = int(value) if value is not None else 0
            if name in self.label_references:
                lbl = self.label_references[name]
                lbl.config(text=f"{val_int}")
                frame = lbl.master
                if val_int > 0:
                    self.set_square_alert(frame)
                else:
                    self.set_square_normal(frame)

            if name == "Limit_Flags":
                self.latched_limit_flags |= val_int

            if (
                name == "ECU_Fault"
                and val_int > 0
                and val_int not in self.active_fault_codes
            ):
                self.active_fault_codes.append(val_int)

            self.update_connection_status(self.can_connected)
            return

        if name in ["Speed", "G_Speed"]:
            if name == "G_Speed":
                self.update_odometer(value)
            self.update_speed_error()

        if name == "RPM":
            self.update_rpm_progress(value)

        if name in self.label_references:
            label_widget = self.label_references[name]
            val_to_show = (
                self.peak_values.get(name, value)
                if self.view_mode == "peak"
                else value
            )

            if name == "Gear":
                formatted_value = f"{int(val_to_show)}"
            elif name in ["Odometer", "V_error"]:
                formatted_value = f"{val_to_show:.0f}"
            elif name in ["Longitude", "Latitude"]:
                formatted_value = f"{val_to_show:.6f}"
            elif name in self.can_config["parameters"]:
                raw_adj = str(
                    self.can_config["parameters"][name].get("adj_factor", "")
                )
                if "*0.00001" in raw_adj:
                    formatted_value = f"{val_to_show:.5f}"
                elif "*0.0001" in raw_adj:
                    formatted_value = f"{val_to_show:.4f}"
                elif "*0.001" in raw_adj:
                    formatted_value = f"{val_to_show:.3f}"
                elif "*0.01" in raw_adj:
                    formatted_value = f"{val_to_show:.2f}"
                elif "*0.1" in raw_adj:
                    formatted_value = f"{val_to_show:.1f}"
                else:
                    formatted_value = f"{val_to_show:.0f}"
            else:
                formatted_value = (
                    f"{val_to_show:.1f}"
                    if isinstance(val_to_show, float)
                    else str(val_to_show)
                )

            if label_widget.winfo_exists():
                label_widget.config(text=formatted_value)

            frame = label_widget.master
            if self.view_mode == "peak":
                self.set_square_alert(frame)
            else:
                param_config = self.can_config["parameters"].get(name, {})
                low_limit = param_config.get("limit_low", -99999)
                high_limit = param_config.get("limit_high", 99999)
                try:
                    v_chk = float(value)
                    if v_chk < low_limit or v_chk > high_limit:
                        self.set_square_alert(frame)
                    else:
                        self.set_square_normal(frame)
                except Exception:
                    self.set_square_normal(frame)

    def set_square_alert(self, frame):
        try:
            if isinstance(frame, ttk.Frame):
                frame.configure(style="Alert.TFrame")

            for child in frame.winfo_children():
                if isinstance(child, tk.Frame):
                    child.configure(bg="red")
                    for g in child.winfo_children():
                        if isinstance(g, ttk.Label):
                            g.configure(style="Alert.TLabel")
                elif isinstance(child, ttk.Label):
                    child.configure(style="Alert.TLabel")
        except Exception as e:
            print("Alert square error:", e)

    def set_square_normal(self, frame):
        try:
            if isinstance(frame, ttk.Frame):
                frame.configure(style="TFrame")

            for child in frame.winfo_children():
                if isinstance(child, tk.Frame):
                    child.configure(bg="black")
                    for g in child.winfo_children():
                        if isinstance(g, ttk.Label):
                            g.configure(style="TLabel")
                elif isinstance(child, ttk.Label):
                    child.configure(style="TLabel")
        except Exception as e:
            print("Normal square error:", e)

    def get_unit_for_signal(self, sig):
        if sig in self.units:
            return self.units[sig]
        for ch, cfg in (
            self.can_config.get("inputs", {}).get("channels", {}).items()
        ):
            if cfg.get("name") == sig:
                return cfg.get("unit", "")
        if sig == "EGT":
            return "°C"
        if sig == "Odometer":
            return "km"
        if sig in ("G_Speed", "V_error"):
            return "km/h"
        if sig in ("Longitude", "Latitude"):
            return "°"
        return ""

    def toggle_warning_icon(self, name, active):
        label = self.warning_icon_refs.get(name)
        if label and name in self.warning_images:
            new_image = (
                self.warning_images[name]["active"]
                if active
                else self.warning_images[name]["inactive"]
            )
            label.config(image=new_image)
            label.image = new_image

    def toggle_view_mode(self, event=None):
        if self.view_mode == "normal":
            self.view_mode = "peak"
        else:
            self.view_mode = "normal"
        for key in self.peak_values:
            self.peak_values[key] = -9999.0

    def evaluate_condition(self, value, operator, threshold):
        try:
            if operator == "<":
                return value < threshold
            elif operator == "<=":
                return value <= threshold
            elif operator == ">":
                return value > threshold
            elif operator == ">=":
                return value >= threshold
            elif operator == "==":
                return value == threshold
            elif operator == "!=":
                return value != threshold
        except Exception:
            pass
        return False

    def update_can_data_thread(self):
        print("Started listening for CAN data...")
        last_rx = time.time()
        while True:
            if bus is None:
                time.sleep(1.0)
                continue

            try:
                message = bus.recv(timeout=0.01)
            except Exception as e:
                print(f"⚠️ CAN recv error: {e}")
                time.sleep(0.5)
                continue

            now = time.time()
            if message is not None:
                last_rx = now
                self.root.after(0, self.update_connection_status, True)
            elif now - last_rx > 2.0:
                self.root.after(0, self.update_connection_status, False)

            if message is None:
                continue

            can_id = message.arbitration_id
            data = message.data
            self.last_messages[can_id] = data

            for name, param_config in self.can_config["parameters"].items():
                if can_id != param_config["can_id"]:
                    continue

                index_str = param_config.get("index")
                if index_str is not None:
                    try:
                        expected_index = int(str(index_str), 0)
                        if len(data) < 1 or data[0] != expected_index:
                            continue
                    except ValueError:
                        print(f"⚠️ Invalid index for {name}: {index_str}")
                        continue

                try:
                    signal_data = data[2:] if index_str is not None else data
                    value = extract_can_signal(
                        signal_data,
                        start_bit=int(param_config.get("start_bit", 0)),
                        type_str=param_config.get("type", "UInt16"),
                        byte_order=param_config.get("byte_order", "LSB"),
                        adj_factor=param_config.get("adj_factor", ""),
                    )
                    self.ui_update_queue.put((name, value))
                except Exception as e:
                    print(f"❌ Error processing {name}: {e}")

    def extract_stream_signal(self, warn_cfg):
        try:
            can_id = warn_cfg.get("can_id")
            index = warn_cfg.get("index")
            start_bit = warn_cfg.get("start_bit", 0)
            type_str = warn_cfg.get("type", "UInt8")
            byte_order = warn_cfg.get("byte_order", "LSB")
            adj = warn_cfg.get("adj_factor", None)

            data = self.last_messages.get(can_id)
            if data is None:
                return None

            if index is not None:
                if len(data) == 0 or data[0] != index:
                    return None

            value = extract_can_signal(
                data=data,
                start_bit=start_bit,
                type_str=type_str,
                byte_order=byte_order,
                adj_factor=adj,
            )
            return value
        except Exception as e:
            print(f"⚠️ extract_stream_signal error: {e}")
            return None

    def process_ui_queue(self):
        processed = 0
        while not self.ui_update_queue.empty() and processed < 25:
            name, value = self.ui_update_queue.get()
            self.update_value(name, value)
            processed += 1

        for warn_key, cfg in self.can_config.get("warnings", {}).items():
            mode = cfg.get("mode")
            is_active = False

            if mode == "digital":
                pin = self.pin_map.get(cfg.get("channel"))
                if pin:
                    raw_state = GPIO.input(pin)
                    if cfg.get("logic") == "Normal":
                        is_active = raw_state == GPIO.HIGH
                    else:
                        is_active = raw_state == GPIO.LOW

            elif mode == "stream":
                v = self.extract_stream_signal(cfg)
                is_active = bool(v) if v is not None else False

            elif mode == "if/else":
                cur = self.sensor_values.get(cfg.get("signal"))
                r1 = (
                    self.evaluate_condition(
                        cur, cfg.get("operator"), cfg.get("value")
                    )
                    if cur is not None
                    else False
                )
                or_sig = cfg.get("or_signal")
                if or_sig:
                    cur2 = self.sensor_values.get(or_sig)
                    r2 = (
                        self.evaluate_condition(
                            cur2, cfg.get("or_operator"), cfg.get("or_value")
                        )
                        if cur2 is not None
                        else False
                    )
                    is_active = r1 or r2
                else:
                    is_active = r1

            self.toggle_warning_icon(warn_key, is_active)

        self.root.after(20, self.process_ui_queue)

    def update_connection_status(self, is_connected):
        self.can_connected = is_connected

        if not is_connected:
            self.connection_status_icon.config(image=self.disconnected_icon)
            self.connection_status_icon.unbind("<Button-1>")
            return

        try:
            current_limits = int(self.sensor_values.get("Limit_Flags", 0) or 0)
        except Exception:
            current_limits = 0

        has_fault = len(self.active_fault_codes) > 0
        has_limit = current_limits > 0

        if has_fault or has_limit:
            self.connection_status_icon.config(image=self.fault_icon)
            self.connection_status_icon.bind(
                "<Button-1>", self.open_diagnostics_popup
            )
        else:
            self.connection_status_icon.config(image=self.connected_icon)
            self.connection_status_icon.unbind("<Button-1>")

    def adc_ecu_worker_thread(self):
        # === Fuel smoothing buffers για Ch0 και Ch1 ===
        if not hasattr(self, "fuel_smooth_ch0"):
            self.fuel_smooth_ch0 = None
        if not hasattr(self, "fuel_smooth_ch1"):
            self.fuel_smooth_ch1 = None

        while True:
            # 1. ------------- READ ADC & FUEL LOGIC -------------
            channels = self.can_config["inputs"]["channels"]
            adc_raw = {}

            for ch, cfg in channels.items():
                if not cfg.get("enabled"):
                    continue

                try:
                    v = self.adc.get_voltage(int(ch))
                    mapped = self.map_adc_value(v, cfg)
                    name = cfg["name"]

                    # Smoothing Logic (Alpha Filter)
                    alpha = 0.005
                    if int(ch) == 0:
                        if self.fuel_smooth_ch0 is None:
                            self.fuel_smooth_ch0 = mapped
                        else:
                            self.fuel_smooth_ch0 = (
                                self.fuel_smooth_ch0 * (1 - alpha) + mapped * alpha
                            )
                        mapped = self.fuel_smooth_ch0
                    elif int(ch) == 1:
                        if self.fuel_smooth_ch1 is None:
                            self.fuel_smooth_ch1 = mapped
                        else:
                            self.fuel_smooth_ch1 = (
                                self.fuel_smooth_ch1 * (1 - alpha) + mapped * alpha
                            )
                        mapped = self.fuel_smooth_ch1

                    # Update Sensor Values (liters for every tank)
                    old = self.sensor_values.get(name)
                    if old != mapped:
                        self.sensor_values[name] = mapped
                        self.ui_update_queue.put((name, mapped))

                    adc_raw[int(ch)] = mapped

                except Exception:
                    continue

            # --- Total tank percentage % ---
            ch0_cfg = channels.get("0", {})
            ch1_cfg = channels.get("1", {})

            # Get liters from sensors
            curr_liters_main = self.sensor_values.get(
                ch0_cfg.get("name", "Fuel Main"), 0
            )
            curr_liters_sub = self.sensor_values.get(
                ch1_cfg.get("name", "Fuel Sub"), 0
            )

            cap_main = float(ch0_cfg.get("min_val", 0))
            cap_sub = float(ch1_cfg.get("min_val", 0))

            total_capacity = 0.0
            current_total_liters = 0.0

            # Calculation based on "Enabled"
            if ch0_cfg.get("enabled"):
                total_capacity += cap_main
                current_total_liters += curr_liters_main

            if ch1_cfg.get("enabled"):
                total_capacity += cap_sub
                current_total_liters += curr_liters_sub

            # Calculation %
            if total_capacity > 0:
                final_percent = (current_total_liters / total_capacity) * 100.0
            else:
                final_percent = 0.0

            # Safety margins 0-100%
            final_percent = max(0.0, min(final_percent, 100.0))

            # Display Fuel Level only if changed
            if self.sensor_values.get("Fuel Level") != final_percent:
                self.sensor_values["Fuel Level"] = final_percent
                self.ui_update_queue.put(("Fuel Level", final_percent))

            # 2. ------------- READ EGT (MAX31855) -------------
            try:
                egt_temp = self.egt_sensor.read_temp()

                if egt_temp is not None:
                    final_egt = round(egt_temp, 0)
                    self.sensor_values["EGT"] = final_egt
                    self.ui_update_queue.put(("EGT", final_egt))
                else:
                    self.sensor_values["EGT"] = 0
                    self.ui_update_queue.put(("EGT", 0))
            except Exception as e:
                print(f"⚠️ EGT Read Error: {e}")

            # 3. ------------- SEND TO ECU (CAN 20 Hz / 50ms) -------------
            if self.adc_to_ecu_enabled and bus is not None:
                try:
                    # --- Bytes 0 & 1: Final Fuel Level (%) ---
                    fuel_val = int(final_percent * 10)
                    fuel_val = max(0, min(fuel_val, 65535))  # UInt16 limits

                    fuel_hi = (fuel_val >> 8) & 0xFF
                    fuel_lo = fuel_val & 0xFF

                    # --- Bytes 2 & 3: Exhaust Gas Temperature (EGT °C) ---
                    egt_val = int(self.sensor_values.get("EGT", 0))
                    egt_val = max(0, min(egt_val, 65535))  # UInt16 limits

                    egt_hi = (egt_val >> 8) & 0xFF
                    egt_lo = egt_val & 0xFF

                    # --- Bytes 4 & 5: ADC Channel 2 (if enabled) ---
                    ch2_val = max(0, min(int(adc_raw.get(2, 0)), 5000))
                    ch2_hi = (ch2_val >> 8) & 0xFF
                    ch2_lo = ch2_val & 0xFF

                    # --- Bytes 6 & 7: ADC Channel 3 (if enabled) ---
                    ch3_val = max(0, min(int(adc_raw.get(3, 0)), 5000))
                    ch3_hi = (ch3_val >> 8) & 0xFF
                    ch3_lo = ch3_val & 0xFF

                    # Construct 8-byte payload for CAN ID 0x457
                    d457 = [
                        fuel_hi,
                        fuel_lo,
                        egt_hi,
                        egt_lo,
                        ch2_hi,
                        ch2_lo,
                        ch3_hi,
                        ch3_lo,
                    ]
                    bus.send(
                        can.Message(
                            arbitration_id=0x457, data=bytes(d457), is_extended_id=False
                        )
                    )

                    # If ADC channels 4-7 are active, broadcast on CAN ID 0x458
                    if any(
                        int(ch) >= 4 and cfg.get("enabled")
                        for ch, cfg in channels.items()
                    ):
                        d458 = []
                        for i in range(4, 8):
                            v_val = max(0, min(int(adc_raw.get(i, 0)), 5000))
                            hi = (v_val >> 8) & 0xFF
                            lo = v_val & 0xFF
                            d458.extend([hi, lo])
                        bus.send(
                            can.Message(
                                arbitration_id=0x458,
                                data=bytes(d458),
                                is_extended_id=False,
                            )
                        )

                except Exception as e:
                    print("ECU Send Error:", e)

            # Loop Rate (20Hz -> 50ms period)
            time.sleep(0.05)

    def map_adc_value(self, v, cfg):
        MinV = cfg.get("min_v", 0.0)
        MidV = cfg.get("mid_v", (MinV + cfg.get("max_v", 5.0)) / 2)
        MaxV = cfg.get("max_v", 5.0)

        MinVal = cfg.get("min_val", 0.0)
        MidVal = cfg.get("mid_val", (MinVal + cfg.get("max_val", 100.0)) / 2)
        MaxVal = cfg.get("max_val", 100.0)

        v = max(MinV, min(v, MaxV))

        if v <= MidV:
            if MidV == MinV:
                return MinVal
            return MinVal + ((v - MinV) / (MidV - MinV)) * (MidVal - MinVal)
        else:
            if MaxV == MidV:
                return MidVal
            return MidVal + ((v - MidV) / (MaxV - MidV)) * (MaxVal - MidVal)

    def toggle_lap_timer(self):
        if not self.is_lap_active:
            self.set_start_finish_line()
            self.lap_start_time = time.time()
            self.is_lap_active = True
            self.flag_icon_label.config(image=self.flag_active_icon)
            self.update_lap_time_display()
        else:
            lap_time = time.time() - self.lap_start_time
            self.lap_times.append(lap_time)
            self.is_lap_active = False
            self.flag_icon_label.config(image=self.flag_inactive_icon)
            self.display_lap_time(lap_time)

    def update_lap_time_display(self):
        if self.is_lap_active:
            current_time = time.time() - self.lap_start_time
            minutes, seconds = divmod(current_time, 60)
            self.lap_time_label.config(
                text=f"Lap Time: {int(minutes):02}:{seconds:.3f}"
            )
            self.root.after(100, self.update_lap_time_display)

    def display_lap_time(self, lap_time):
        minutes, seconds = divmod(lap_time, 60)
        self.lap_time_label.config(
            text=f"Time Lap: {int(minutes):02}:{seconds:.3f}"
        )

        if self.lap_times:
            best_time = min(self.lap_times)
            b_min, b_sec = divmod(best_time, 60)
            self.best_lap_label.config(text=f"Best Lap: {int(b_min):02}:{b_sec:.3f}")

    def set_start_finish_line(self):
        try:
            lat = float(self.gps_stream.TPV.get("lat", "n/a"))
            lon = float(self.gps_stream.TPV.get("lon", "n/a"))
            mode = int(float(self.gps_stream.TPV.get("mode", 1)))
            if mode >= 2:
                self.start_finish_line = {"latitude": lat, "longitude": lon}
                messagebox.showinfo(
                    "Start/Finish Line Set",
                    f"Set at Latitude: {lat:.6f}, Longitude: {lon:.6f}",
                )
            else:
                messagebox.showwarning(
                    "GPS Fix Unavailable", "No GPS fix available. Try again."
                )
        except Exception:
            messagebox.showerror("GPS Error", "Unable to get GPS data")

    def initialize_gpsd(self):
        try:
            self.gps_socket = gps3.GPSDSocket()
            self.gps_stream = gps3.DataStream()
            self.gps_socket.connect()
            self.gps_socket.watch()
            print("✅ GPSD connected using gps3.")
        except Exception as e:
            print(f"❌ Failed to initialize GPSD with gps3: {e}")

    def update_gps_status(self):
        try:
            mode_raw = self.gps_stream.TPV.get("mode", 1)
            try:
                fix_mode = int(float(mode_raw))
            except ValueError:
                fix_mode = 1

            if fix_mode != getattr(self, "current_gps_mode", None):
                self.current_gps_mode = fix_mode
                if fix_mode == 1:
                    self.gps_status_label.config(image=self.no_fix_img)
                elif fix_mode == 2:
                    self.gps_status_label.config(image=self.fix_2d_img)
                elif fix_mode == 3:
                    self.gps_status_label.config(image=self.fix_3d_img)
        except Exception as e:
            print(f"⚠️ GPS status error: {e}")
            self.gps_status_label.config(image=self.no_fix_img)

        self.root.after(1000, self.update_gps_status)

    def update_gps_data_thread(self):
        while True:
            try:
                if not hasattr(self, "gps_socket") or self.gps_socket is None:
                    self.initialize_gpsd()
                    time.sleep(1)
                    continue

                for new_data in self.gps_socket:
                    if new_data:
                        self.gps_stream.unpack(new_data)
                        mode_raw = self.gps_stream.TPV.get("mode", 1)

                        try:
                            fix_mode = int(float(mode_raw))
                        except (ValueError, TypeError):
                            fix_mode = 1

                        if fix_mode >= 2:
                            speed_raw = self.gps_stream.TPV.get("speed", 0.0)
                            lat_raw = self.gps_stream.TPV.get("lat", 0.0)
                            lon_raw = self.gps_stream.TPV.get("lon", 0.0)

                            if "n/a" in (speed_raw, lat_raw, lon_raw):
                                continue

                            try:
                                gps_speed = float(speed_raw) * 3.6
                                latitude = float(lat_raw)
                                longitude = float(lon_raw)
                            except (ValueError, TypeError):
                                continue

                            self.update_value("G_Speed", gps_speed)
                            self.update_value("Latitude", latitude)
                            self.update_value("Longitude", longitude)

                            if self.is_lap_active and self.start_finish_line:
                                sf_lat = self.start_finish_line.get("latitude")
                                sf_lon = self.start_finish_line.get("longitude")

                                if sf_lat and sf_lon:
                                    dist = haversine_distance(
                                        latitude, longitude, sf_lat, sf_lon
                                    )

                                    if dist < 12 and not self.last_position_crossed:
                                        self.last_position_crossed = True
                                        now = time.time()
                                        lap_time = now - self.lap_start_time
                                        self.lap_start_time = now
                                        self.lap_times.append(lap_time)

                                        self.root.after(
                                            0, lambda lt=lap_time: self.display_lap_time(lt)
                                        )

                                    elif dist >= 15:
                                        self.last_position_crossed = False

                        time.sleep(0.05)
            except Exception as e:
                print(f"❌ GPS Thread disconnect: {e}")
                self.gps_socket = None
                time.sleep(2)

    def test_mysql_ready(self, max_tries=3, delay=0.1):
        for _ in range(max_tries):
            try:
                conn = mysql.connector.connect(
                    host="localhost",
                    user="root",
                    unix_socket="/run/mysqld/mysqld.sock",
                    database="rx8data",
                    connection_timeout=2,
                )
                conn.close()
                return True
            except mysql.connector.Error as e:
                print(f"ℹ️ MariaDB not ready yet: {e}")
                time.sleep(delay)
        return False

    def ensure_db_connection(self):
        if (
            getattr(self, "db_connection", None)
            and self.db_connection.is_connected()
        ):
            return

        self.db_connection = mysql.connector.connect(
            host="localhost",
            user="root",
            unix_socket="/run/mysqld/mysqld.sock",
            database="rx8data",
        )
        self.cursor = self.db_connection.cursor()

    def sqlify_column_name(self, name: str) -> str:
        if not isinstance(name, str):
            name = str(name)
        col = name.strip().lower().replace(" ", "_")
        col = re.sub(r"[^0-9a-zA-Z_]", "", col)
        if not col:
            col = "col"
        if col[0].isdigit():
            col = "_" + col
        return col

    def get_log_table_name_for_day(self, day_str: str = None) -> str:
        if day_str is None:
            day_str = datetime.now().strftime("%Y%m%d")
        return f"sensor_log_{day_str}"

    def get_can_signal_names_for_logging(self):
        names = list(self.can_config.get("parameters", {}).keys())
        extras = [
            "G_Speed",
            "Longitude",
            "Latitude",
            "Odometer",
            "EGT",
            "V_error",
            "Fuel Level",
        ]
        for n in extras:
            if n not in names and n in self.sensor_values:
                names.append(n)
        return names

    def get_enabled_adc_names_for_logging(self):
        adc_names = []
        channels = self.can_config.get("inputs", {}).get("channels", {})
        for ch, cfg in channels.items():
            if cfg.get("enabled") and cfg.get("name"):
                adc_names.append(cfg["name"])
        return adc_names

    def ensure_log_table_for_day(self, day_str):
        table_name = self.get_log_table_name_for_day(day_str)
        can_names = self.get_can_signal_names_for_logging()
        adc_names = self.get_enabled_adc_names_for_logging()

        self.log_day = day_str
        self.log_table_name = table_name
        self.log_can_names = list(can_names)
        self.log_adc_names = list(adc_names)

        self.log_columns_sql = ["timestamp"] + [
            self.sqlify_column_name(n)
            for n in (self.log_can_names + self.log_adc_names)
        ]

        placeholders = ", ".join(["%s"] * len(self.log_columns_sql))
        cols_sql_str = ", ".join(f"`{c}`" for c in self.log_columns_sql)
        self.log_insert_query = (
            f"INSERT INTO `{self.log_table_name}` ({cols_sql_str}) "
            f"VALUES ({placeholders})"
        )

        if not self.test_mysql_ready():
            print(
                "⚠️ MariaDB not ready, will retry table creation from writer thread."
            )
            return

        try:
            self.ensure_db_connection()
            columns_defs = [
                "`id` BIGINT UNSIGNED NOT NULL AUTO_INCREMENT",
                "`timestamp` DATETIME NOT NULL",
            ]
            for sig in self.log_can_names + self.log_adc_names:
                col = self.sqlify_column_name(sig)
                columns_defs.append(f"`{col}` DOUBLE NULL")

            cols_sql = ", ".join(columns_defs)
            create_sql = (
                f"CREATE TABLE IF NOT EXISTS `{table_name}` ({cols_sql}, PRIMARY KEY"
                " (`id`)) ENGINE=InnoDB"
            )

            self.cursor.execute(create_sql)
            self.db_connection.commit()
        except mysql.connector.Error as e:
            print(f"❌ Error creating log table: {e}")

    def start_logging_session(self):
        day_str = datetime.now().strftime("%Y%m%d")
        self.ensure_log_table_for_day(day_str)

    def logging_sampler_loop(self):
        while True:
            if not self.logging_enabled:
                time.sleep(0.1)
                continue

            try:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                current_day = datetime.now().strftime("%Y%m%d")
                if self.log_day != current_day:
                    self.ensure_log_table_for_day(current_day)

                can_names = (
                    self.log_can_names or self.get_can_signal_names_for_logging()
                )
                adc_names = (
                    self.log_adc_names or self.get_enabled_adc_names_for_logging()
                )

                # Thread-safe snapshot of sensor_values
                sensor_snapshot = dict(self.sensor_values)

                row = [timestamp]
                for name in can_names:
                    v = sensor_snapshot.get(name)
                    if isinstance(v, (int, float)):
                        row.append(float(v))
                    else:
                        try:
                            row.append(float(v))
                        except (TypeError, ValueError):
                            row.append(None)

                for name in adc_names:
                    v = sensor_snapshot.get(name)
                    if isinstance(v, (int, float)):
                        row.append(float(v))
                    else:
                        try:
                            row.append(float(v))
                        except (TypeError, ValueError):
                            row.append(None)

                self.log_queue.put(row)
            except Exception as e:
                print(f"⚠️ logging_sampler_loop error: {e}")

            time.sleep(0.1)

    def process_log_queue(self):
        while True:
            if not self.logging_enabled:
                if getattr(self, "db_connection", None):
                    try:
                        if self.cursor:
                            self.cursor.close()
                        if self.db_connection.is_connected():
                            self.db_connection.close()
                    except Exception:
                        pass
                    self.db_connection = None
                    self.cursor = None
                time.sleep(0.2)
                continue

            if not self.test_mysql_ready():
                time.sleep(0.5)
                continue

            if self.log_day is None or self.log_insert_query is None:
                self.ensure_log_table_for_day(datetime.now().strftime("%Y%m%d"))

            batch = []
            try:
                while len(batch) < 200:
                    try:
                        row = self.log_queue.get(timeout=0.05)
                        batch.append(row)
                    except queue.Empty:
                        break

                if not batch:
                    time.sleep(0.05)
                    continue

                self.batch_insert_to_db(batch)
            except Exception as e:
                print(f"⚠️ process_log_queue error: {e}")
                if getattr(self, "db_connection", None):
                    try:
                        if self.cursor:
                            self.cursor.close()
                        if self.db_connection.is_connected():
                            self.db_connection.close()
                    except Exception:
                        pass
                    self.db_connection = None
                    self.cursor = None
                time.sleep(0.5)

    def batch_insert_to_db(self, batch_rows):
        if not batch_rows or not self.log_insert_query:
            return
        try:
            self.ensure_db_connection()
            self.cursor.executemany(self.log_insert_query, batch_rows)
            self.db_connection.commit()
        except mysql.connector.Error as e:
            print(f"❌ Error in batch insert: {e}")
            raise

    def rename_adc_column_in_db(self, old_name, new_name):
        old_col = self.sqlify_column_name(old_name)
        new_col = self.sqlify_column_name(new_name)
        if old_col == new_col:
            return

        if not self.test_mysql_ready():
            print("⚠️ MariaDB not ready, skipping column rename for now.")
            return

        try:
            self.ensure_db_connection()
            cur = self.db_connection.cursor()
            cur.execute("""
                SELECT TABLE_NAME 
                FROM INFORMATION_SCHEMA.TABLES 
                WHERE TABLE_SCHEMA = DATABASE() 
                AND TABLE_NAME LIKE 'sensor_log_%';
            """)
            tables = [row[0] for row in cur.fetchall()]

            for table in tables:
                cur.execute(
                    """
                    SELECT COUNT(*) 
                    FROM INFORMATION_SCHEMA.COLUMNS 
                    WHERE TABLE_SCHEMA = DATABASE() 
                    AND TABLE_NAME = %s 
                    AND COLUMN_NAME = %s;
                    """,
                    (table, old_col),
                )
                if cur.fetchone()[0]:
                    cur.execute(
                        f"ALTER TABLE `{table}` CHANGE `{old_col}` `{new_col}` DOUBLE"
                        " NULL;"
                    )

            self.db_connection.commit()
            cur.close()
        except mysql.connector.Error as e:
            print(f"❌ Rename column error: {e}")

    def toggle_logging(self, event):
        self.logging_enabled = not self.logging_enabled
        if self.logging_enabled:
            self.v_error_sum_sq = 0.0
            self.v_error_count = 0
            self.logging_status_icon.config(image=self.logging_enabled_icon)
            manage_service("mariadb", "start")
            self.start_logging_session()
        else:
            self.logging_status_icon.config(image=self.logging_disabled_icon)
            manage_service("mariadb", "stop")

            if self.v_error_count > 0:
                rmse = math.sqrt(self.v_error_sum_sq / self.v_error_count)
                print(
                    f"📊 Session RMSE V_error: {rmse:.3f} km/h (Samples:"
                    f" {self.v_error_count})"
                )

    def on_closing(self):
        try:
            if getattr(self, "cursor", None):
                self.cursor.close()
            if (
                getattr(self, "db_connection", None)
                and self.db_connection.is_connected()
            ):
                self.db_connection.close()
        except Exception:
            pass
        self.root.quit()

    def process_odometer_queue(self):
        while True:
            time.sleep(10)
            while not self.odometer_queue.empty():
                last_value = self.odometer_queue.get()
                self.can_config["inputs"]["odometer"] = last_value
                save_can_config(self.can_config)

    def wifi_status_loop(self):
        try:
            wifi_online, _ = self.get_wifi_status()
            self.wifi_icon_label.config(
                image=self.wifi_online_icon if wifi_online else self.wifi_offline_icon
            )
        except Exception as e:
            print(f"⚠️ WiFi status error: {e}")
        self.root.after(1500, self.wifi_status_loop)

    def get_wifi_status(self):
        try:
            ip = os.popen("hostname -I").read().strip()
            if ip:
                return True, ip.split()[0]
        except Exception:
            pass
        return False, None

    def is_wifi_disabled_in_boot(self):
        try:
            with open("/boot/config.txt", "r") as f:
                for line in f:
                    if line.strip() == "dtoverlay=disable-wifi":
                        return True
        except Exception:
            pass
        return False

    def set_wifi_enabled(self, enable: bool):
        try:
            with open("/boot/config.txt", "r") as f:
                lines = f.readlines()

            new_lines = []
            found = False
            for line in lines:
                if "dtoverlay=disable-wifi" in line:
                    found = True
                    new_lines.append(
                        "#dtoverlay=disable-wifi\n"
                        if enable
                        else "dtoverlay=disable-wifi\n"
                    )
                else:
                    new_lines.append(line)

            if not found and not enable:
                new_lines.append("\ndtoverlay=disable-wifi\n")

            with open("/boot/config.txt", "w") as f:
                f.writelines(new_lines)
        except Exception as e:
            print(f"WiFi toggle error: {e}")

    def is_mariadb_running(self):
        return os.system("systemctl is-active --quiet mariadb") == 0

    def toggle_mariadb(self):
        if self.is_mariadb_running():
            manage_service("mariadb", "stop")
        else:
            manage_service("mariadb", "start")

    def reboot_system(self):
        os.system("sudo reboot")

    def open_wifi_popup(self):
        if getattr(self, "wifi_popup_open", False):
            return
        self.wifi_popup_open = True

        popup = tk.Toplevel(self.root)
        popup.overrideredirect(True)
        popup.title("WiFi Config")
        popup.configure(
            bg="black", highlightbackground="black", highlightthickness=2
        )
        popup.geometry("320x200")
        popup.resizable(False, False)
        popup.protocol("WM_DELETE_WINDOW", lambda: None)

        wifi_online, ip = self.get_wifi_status()
        wifi_disabled = self.is_wifi_disabled_in_boot()
        mariadb_on = self.is_mariadb_running()

        popup.grid_columnconfigure(0, weight=1)
        popup.grid_columnconfigure(1, weight=1)

        def toggle_wifi_and_refresh():
            self.set_wifi_enabled(enable=wifi_disabled)
            messagebox.showinfo("WiFi", "WiFi Changed.\nPlease Reboot.")
            self.open_wifi_popup()

        tk.Button(
            popup,
            text=ip if wifi_online else "OFFLINE",
            fg="white",
            bg="green" if wifi_online else "red",
            activebackground="green" if wifi_online else "red",
            width=16,
            command=toggle_wifi_and_refresh,
        ).grid(row=0, column=0, padx=10, pady=15, sticky="w")

        tk.Button(
            popup,
            text="REBOOT",
            fg="white",
            bg="red",
            width=14,
            command=self.reboot_system,
        ).grid(row=0, column=1, padx=15, pady=(5, 15), sticky="e")

        def toggle_mariadb_and_refresh():
            self.toggle_mariadb()
            popup.destroy()
            self.wifi_popup_open = False
            self.open_wifi_popup()

        tk.Button(
            popup,
            text="Maria DB ON" if mariadb_on else "Maria DB OFF",
            fg="white",
            bg="green" if mariadb_on else "red",
            activebackground="green" if mariadb_on else "red",
            width=16,
            command=toggle_mariadb_and_refresh,
        ).grid(row=1, column=0, padx=10, pady=10, sticky="w")

        tk.Button(
            popup,
            text="CLOSE",
            fg="white",
            bg="#444444",
            activebackground="#666666",
            width=16,
            command=lambda: (
                popup.destroy(),
                setattr(self, "wifi_popup_open", False),
            ),
        ).grid(row=1, column=1, padx=10, pady=10, sticky="e")

    def open_diagnostics_popup(self, event=None):
        if hasattr(self, "diagnostics_popup_open") and self.diagnostics_popup_open:
            return
        self.diagnostics_popup_open = True

        popup = tk.Toplevel(self.root)
        popup.title("System Diagnostics")
        popup.geometry("600x400")
        popup.configure(bg="black")
        popup.resizable(False, False)

        def on_close():
            self.diagnostics_popup_open = False
            popup.destroy()

        popup.protocol("WM_DELETE_WINDOW", on_close)

        tk.Label(
            popup,
            text="Active Diagnostics (History)",
            font=("Helvetica", 16, "bold"),
            fg="red",
            bg="black",
        ).pack(pady=10)
        content_frame = tk.Frame(popup, bg="black")
        content_frame.pack(fill="both", expand=True, padx=10)

        left_frame = tk.Frame(content_frame, bg="black", width=280)
        left_frame.pack(side="left", fill="both", expand=True, padx=5)
        tk.Label(
            left_frame,
            text="ECU Faults",
            font=("Helvetica", 14, "bold", "underline"),
            fg="white",
            bg="black",
        ).pack(pady=5)

        if not self.active_fault_codes:
            tk.Label(
                left_frame, text="No Faults", font=("Helvetica", 12), fg="green", bg="black"
            ).pack()
        else:
            for code in self.active_fault_codes:
                tk.Label(
                    left_frame,
                    text=f"⚠ Code: {code}",
                    font=("Helvetica", 12),
                    fg="orange",
                    bg="black",
                ).pack()

        ttk.Separator(content_frame, orient="vertical").pack(
            side="left", fill="y", padx=10
        )

        right_frame = tk.Frame(content_frame, bg="black", width=280)
        right_frame.pack(side="right", fill="both", expand=True, padx=5)
        tk.Label(
            right_frame,
            text="Triggered Limits",
            font=("Helvetica", 14, "bold", "underline"),
            fg="white",
            bg="black",
        ).pack(pady=5)

        limit_defs = self.can_config.get("limit_definitions", {})
        current_limit_val = self.latched_limit_flags
        active_limits = []

        for bit in range(16):
            if (current_limit_val >> bit) & 1:
                name = limit_defs.get(str(bit), f"Limit Bit {bit}")
                active_limits.append(name)

        if not active_limits:
            tk.Label(
                right_frame,
                text="No Limits Triggered",
                font=("Helvetica", 12),
                fg="green",
                bg="black",
            ).pack()
        else:
            for limit in active_limits:
                tk.Label(
                    right_frame,
                    text=f"⚠ {limit}",
                    font=("Helvetica", 12, "bold"),
                    fg="orange",
                    bg="black",
                ).pack()

        button_frame = tk.Frame(popup, bg="black")
        button_frame.pack(pady=20, fill="x")

        def reset_all():
            self.active_fault_codes.clear()
            self.latched_limit_flags = 0
            self.update_connection_status(self.can_connected)
            self.toggle_warning_icon("check", False)

            if "ECU_Fault" in self.label_references:
                lbl = self.label_references["ECU_Fault"]
                lbl.config(text="0")
                self.set_square_normal(lbl.master)
            if "Limit_Flags" in self.label_references:
                lbl = self.label_references["Limit_Flags"]
                lbl.config(text="0x0000")
                self.set_square_normal(lbl.master)

            on_close()

        tk.Button(
            button_frame,
            text="Reset History",
            command=reset_all,
            width=15,
            bg="#333",
            fg="white",
        ).pack(side="left", padx=20)
        tk.Button(
            button_frame,
            text="Close",
            command=on_close,
            width=15,
            bg="#333",
            fg="white",
        ).pack(side="right", padx=20)

    def open_settings_popup(self, event=None):
        if hasattr(self, "settings_popup_open") and self.settings_popup_open:
            return
        self.settings_popup_open = True

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()

        popup = tk.Toplevel(self.root)
        popup.withdraw()
        popup.configure(highlightbackground="black", highlightthickness=2)
        popup.title("CAN Bus Settings")
        popup.configure(bg="#181818")
        popup.geometry(f"{sw}x{sh}+0+0")
        popup.attributes("-fullscreen", True)
        popup.bind("<Escape>", lambda e: popup.destroy())

        BG_MAIN = "#181818"
        BG_ENTRY = "#333333"
        FG_TEXT = "white"
        FG_LBL = "#FFD700"
        FONT_HEAD = ("Helvetica", 12, "bold")
        FONT_BODY = ("Helvetica", 10, "bold")

        container = tk.Frame(popup, bg=BG_MAIN)
        container.pack(fill="both", expand=True)

        canvas = tk.Canvas(
            container, borderwidth=0, bg=BG_MAIN, highlightthickness=0
        )
        v_scroll = tk.Scrollbar(
            container, orient="vertical", command=canvas.yview, bg=BG_MAIN
        )
        canvas.configure(yscrollcommand=v_scroll.set)

        canvas.pack(side="left", fill="both", expand=True)
        v_scroll.pack(side="right", fill="y")

        scroll_frame = tk.Frame(canvas, bg=BG_MAIN)
        scroll_window = canvas.create_window(
            0, 0, window=scroll_frame, anchor="nw", width=sw
        )

        def configure_canvas(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(scroll_window, width=event.width)

        canvas.bind("<Configure>", configure_canvas)

        for i in range(10):
            scroll_frame.grid_columnconfigure(i, weight=1)

        headers = [
            "Parameter",
            "CAN ID",
            "Index",
            "Start Bit",
            "Byte Order",
            "Type",
            "Adj Factor",
            "Limit Low",
            "Limit High",
            "Unit",
        ]
        for col, header in enumerate(headers):
            tk.Label(
                scroll_frame, text=header, font=FONT_HEAD, fg="#00aaff", bg=BG_MAIN
            ).grid(row=0, column=col, padx=2, pady=10)

        self.entries = {}
        exclude_params = {"G_Speed"}

        for i, (param, cfg) in enumerate(
            self.can_config.get("parameters", {}).items(), start=1
        ):
            if param in exclude_params:
                continue
            self.entries[param] = {}

            tk.Label(
                scroll_frame,
                text=param,
                font=("Helvetica", 11, "bold"),
                fg=FG_LBL,
                bg=BG_MAIN,
            ).grid(row=i, column=0, padx=5, pady=4)

            def entry(col, key, val, width):
                e = tk.Entry(
                    scroll_frame,
                    width=width,
                    font=FONT_BODY,
                    justify="center",
                    bg=BG_ENTRY,
                    fg=FG_TEXT,
                    insertbackground="white",
                    bd=0,
                )
                e.insert(0, val)
                e.grid(row=i, column=col, padx=2, pady=2, ipady=3)
                self.entries[param][key] = e

            entry(1, "can_id", cfg.get("can_id", ""), 10)
            index_val = cfg.get("index")
            idx_display = f"{int(index_val)}" if index_val is not None else ""
            entry(2, "index", idx_display, 10)
            entry(3, "start_bit", cfg.get("start_bit", 0), 10)

            byte_cb = ttk.Combobox(
                scroll_frame,
                values=["LSB", "MSB"],
                width=10,
                font=FONT_BODY,
                justify="center",
            )
            byte_cb.set(cfg.get("byte_order", "LSB"))
            byte_cb.grid(row=i, column=4, padx=2, pady=2, ipady=3)
            self.entries[param]["byte_order"] = byte_cb

            type_cb = ttk.Combobox(
                scroll_frame,
                values=["UInt8", "Int8", "UInt16", "Int16", "Bit"],
                width=14,
                font=FONT_BODY,
                justify="center",
            )
            type_cb.set(cfg.get("type", "UInt16"))
            type_cb.grid(row=i, column=5, padx=2, pady=2, ipady=3)
            self.entries[param]["type"] = type_cb

            entry(6, "adj_factor", cfg.get("adj_factor", ""), 30)
            entry(7, "limit_low", cfg.get("limit_low", 0), 12)
            entry(8, "limit_high", cfg.get("limit_high", 100), 12)
            entry(9, "unit", cfg.get("unit", ""), 8)

        def save():
            for param, widgets in self.entries.items():
                try:
                    def safe_int(val_str):
                        s = val_str.strip()
                        if not s:
                            return None
                        return int(s, 0)

                    cid_val = safe_int(widgets["can_id"].get()) or 0
                    idx_val = safe_int(widgets["index"].get())

                    self.can_config["parameters"][param] = {
                        "can_id": cid_val,
                        "index": idx_val,
                        "start_bit": (
                            int(widgets["start_bit"].get())
                            if widgets["start_bit"].get().strip().isdigit()
                            else 0
                        ),
                        "byte_order": widgets["byte_order"].get() or "LSB",
                        "type": widgets["type"].get() or "UInt16",
                        "adj_factor": widgets["adj_factor"].get(),
                        "limit_low": float(widgets["limit_low"].get()),
                        "limit_high": float(widgets["limit_high"].get()),
                        "unit": widgets["unit"].get(),
                    }
                except Exception:
                    pass

            save_can_config(self.can_config)
            if save_btn.winfo_exists():
                save_btn.config(text="✔ Saved", bg="#006600")
            self.root.after(
                1500,
                lambda: save_btn.winfo_exists()
                and save_btn.config(text="⛁ Save", bg="#008800"),
            )

        btn_font = ("Helvetica", 12, "bold")
        button_holder = tk.Frame(scroll_frame, bg=BG_MAIN)
        button_holder.grid(
            row=len(self.entries) + 2, column=0, columnspan=10, pady=(5, 30)
        )

        tk.Button(
            button_holder,
            text="⚠️ Warnings",
            font=btn_font,
            command=lambda: not self.warnings_popup_open
            and self.open_warnings_popup(),
            width=12,
            bg="#333",
            fg="white",
        ).pack(side="left", padx=10)
        tk.Button(
            button_holder,
            text="⚙ Inputs",
            font=btn_font,
            command=lambda: not self.inputs_popup_open and self.open_inputs_popup(),
            width=12,
            bg="#333",
            fg="white",
        ).pack(side="left", padx=10)
        tk.Button(
            button_holder,
            text="⚅ Display",
            font=btn_font,
            command=lambda: not self.display_popup_open
            and self.open_display_popup(),
            width=12,
            bg="#333",
            fg="white",
        ).pack(side="left", padx=10)

        save_btn = tk.Button(
            button_holder,
            text="⛁ Save",
            font=btn_font,
            width=12,
            bg="#008800",
            fg="white",
            command=save,
        )
        save_btn.pack(side="left", padx=10)
        tk.Button(
            button_holder,
            text="✗ Close",
            font=btn_font,
            command=lambda: (
                popup.destroy(),
                setattr(self, "settings_popup_open", False),
            ),
            width=12,
            bg="#880000",
            fg="white",
        ).pack(side="left", padx=10)

        popup.update_idletasks()
        popup.deiconify()

    def open_warnings_popup(self):
        if self.warnings_popup_open:
            return
        self.warnings_popup_open = True

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()

        popup = tk.Toplevel(self.root)
        popup.withdraw()
        popup.configure(highlightbackground="black", highlightthickness=2)
        popup.title("Warning Settings")
        popup.configure(bg="#181818")
        popup.geometry(f"{sw}x{sh}+0+0")
        popup.attributes("-fullscreen", True)
        popup.bind("<Escape>", lambda e: popup.destroy())

        BG_MAIN = "#181818"
        BG_ENTRY = "#333333"
        FG_LBL = "#FFD700"
        FONT_HEAD = ("Helvetica", 11, "bold")
        FONT_BODY = ("Helvetica", 10, "bold")
        BG_DISABLED = "#CCCCCC"
        FG_DISABLED = "#AAAAAA"

        container = tk.Frame(popup, bg=BG_MAIN)
        container.pack(fill="both", expand=True)

        canvas = tk.Canvas(
            container, borderwidth=0, bg=BG_MAIN, highlightthickness=0
        )
        v_scroll = tk.Scrollbar(
            container, orient="vertical", command=canvas.yview, bg=BG_MAIN
        )
        canvas.configure(yscrollcommand=v_scroll.set)

        canvas.pack(side="left", fill="both", expand=True)
        v_scroll.pack(side="right", fill="y")

        scroll_frame = tk.Frame(canvas, bg=BG_MAIN)
        scroll_window = canvas.create_window(
            0, 0, window=scroll_frame, anchor="nw", width=sw
        )

        def configure_canvas(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(scroll_window, width=event.width)

        canvas.bind("<Configure>", configure_canvas)

        for i in range(16):
            scroll_frame.grid_columnconfigure(i, weight=1)

        headers = [
            "Key",
            "Mode",
            "Signal",
            "Op",
            "Val",
            "Or Sig",
            "Op",
            "Or Val",
            "CanID",
            "S_Bit",
            "Type",
            "Order",
            "Idx",
            "Channel",
            "Logic",
            "Pull",
        ]
        for col, text in enumerate(headers):
            pad_left = 1
            if col == 2:
                pad_left = 25
            if col == 8:
                pad_left = 25
            if col == 13:
                pad_left = 25
            tk.Label(
                scroll_frame, text=text, font=FONT_HEAD, fg="#00aaff", bg=BG_MAIN
            ).grid(row=0, column=col, padx=(pad_left, 0), pady=5)

        self.warning_entries = {}
        available_signals = list(self.sensor_values.keys())

        def update_field_states(key):
            w = self.warning_entries[key]
            m = w["mode"].get()

            def set_state(widgets, state):
                for widget in widgets:
                    if state == "normal":
                        widget.configure(
                            state=(
                                "normal"
                                if isinstance(widget, tk.Entry)
                                else "readonly"
                            )
                        )
                    else:
                        widget.configure(state="disabled")

            if_else = [
                w["signal"],
                w["operator"],
                w["value"],
                w["or_signal"],
                w["or_operator"],
                w["or_value"],
            ]
            stream = [
                w["can_id"],
                w["start_bit"],
                w["type"],
                w["byte_order"],
                w["index"],
            ]
            digital = [w["channel"], w["logic"], w["pull"]]

            if m == "if/else":
                set_state(if_else, "normal")
                set_state(stream, "disabled")
                set_state(digital, "disabled")
            elif m == "stream":
                set_state(if_else, "disabled")
                set_state(stream, "normal")
                set_state(digital, "disabled")
            elif m == "digital":
                set_state(if_else, "disabled")
                set_state(stream, "disabled")
                set_state(digital, "normal")

        for i, (key, cfg) in enumerate(
            self.can_config.get("warnings", {}).items(), start=1
        ):
            self.warning_entries[key] = {}
            if key in self.warning_images:
                photo = self.warning_images[key]["active"]
                lbl = tk.Label(scroll_frame, image=photo, bg=BG_MAIN)
                lbl.image = photo
                lbl.grid(row=i, column=0, padx=1, pady=1)
            else:
                tk.Label(
                    scroll_frame,
                    text=key,
                    font=("Helvetica", 11, "bold"),
                    fg=FG_LBL,
                    bg=BG_MAIN,
                ).grid(row=i, column=0, padx=1, pady=1)

            def grid_widget(widget, col):
                pad_left = 1
                if col == 2:
                    pad_left = 25
                if col == 8:
                    pad_left = 25
                if col == 13:
                    pad_left = 25
                widget.grid(row=i, column=col, padx=(pad_left, 0), pady=1, ipady=3)

            def mk_entry(col, k, val, w=5):
                e = tk.Entry(
                    scroll_frame,
                    width=w,
                    font=FONT_BODY,
                    justify="center",
                    bg=BG_ENTRY,
                    fg="white",
                    insertbackground="white",
                    bd=0,
                    disabledbackground=BG_DISABLED,
                    disabledforeground=FG_DISABLED,
                )
                e.insert(0, str(val))
                grid_widget(e, col)
                self.warning_entries[key][k] = e

            def mk_combo(col, k, val, vals, w=6):
                cb = ttk.Combobox(
                    scroll_frame,
                    values=vals,
                    width=w,
                    font=FONT_BODY,
                    justify="center",
                )
                cb.set(str(val))
                grid_widget(cb, col)
                self.warning_entries[key][k] = cb

            mk_combo(
                1,
                "mode",
                cfg.get("mode", "stream"),
                ["stream", "if/else", "digital"],
                8,
            )
            mk_combo(2, "signal", cfg.get("signal") or "", available_signals, 15)
            mk_combo(
                3, "operator", cfg.get("operator") or "", ["<", ">", "==", "!="], 3
            )
            mk_entry(4, "value", cfg.get("value", ""), 8)
            mk_combo(
                5, "or_signal", cfg.get("or_signal") or "", available_signals, 15
            )
            mk_combo(
                6,
                "or_operator",
                cfg.get("or_operator") or "",
                ["<", ">", "==", "!="],
                3,
            )
            mk_entry(7, "or_value", cfg.get("or_value", ""), 8)
            mk_entry(8, "can_id", cfg.get("can_id", ""), 7)
            mk_entry(9, "start_bit", cfg.get("start_bit", ""), 6)
            mk_combo(
                10,
                "type",
                cfg.get("type", "Uint8"),
                ["Uint8", "Uint16", "Bit"],
                6,
            )
            mk_combo(
                11,
                "byte_order",
                cfg.get("byte_order", "MSB"),
                ["MSB", "LSB"],
                5,
            )
            mk_entry(12, "index", cfg.get("index", ""), 6)
            mk_combo(
                13,
                "channel",
                cfg.get("channel", "ch1"),
                ["ch1", "ch2", "ch3", "ch4"],
                5,
            )
            mk_combo(
                14,
                "logic",
                cfg.get("logic", "Normal"),
                ["Normal", "Inverted"],
                10,
            )
            mk_combo(15, "pull", cfg.get("pull", "UP"), ["UP", "DOWN"], 8)

            self.warning_entries[key]["mode"].bind(
                "<<ComboboxSelected>>", lambda e, k=key: update_field_states(k)
            )
            update_field_states(key)

        def save_warning_settings():
            for key, widgets in self.warning_entries.items():
                try:
                    m = widgets["mode"].get()
                    cfg = {"mode": m}

                    def get_safe_val(val, to_type=float):
                        s = val.strip()
                        if not s or s.lower() in ["none", "null"]:
                            return None
                        try:
                            return to_type(s)
                        except Exception:
                            return None

                    if m == "if/else":
                        cfg.update({
                            "signal": widgets["signal"].get() or None,
                            "operator": widgets["operator"].get() or None,
                            "value": get_safe_val(widgets["value"].get()),
                            "or_signal": widgets["or_signal"].get() or None,
                            "or_operator": widgets["or_operator"].get() or None,
                            "or_value": get_safe_val(widgets["or_value"].get()),
                        })
                    elif m == "digital":
                        cfg.update({
                            "channel": widgets["channel"].get() or "ch1",
                            "logic": widgets["logic"].get() or "Normal",
                            "pull": widgets["pull"].get() or "UP",
                        })
                    else:
                        cfg.update({
                            "can_id": (
                                int(widgets["can_id"].get())
                                if widgets["can_id"].get().strip().isdigit()
                                else 0
                            ),
                            "start_bit": (
                                int(widgets["start_bit"].get())
                                if widgets["start_bit"].get().strip().isdigit()
                                else 0
                            ),
                            "type": widgets["type"].get() or "Uint8",
                            "byte_order": widgets["byte_order"].get() or "MSB",
                            "index": (
                                int(widgets["index"].get())
                                if widgets["index"].get().strip().isdigit()
                                else None
                            ),
                        })
                    self.can_config["warnings"][key] = cfg
                except Exception:
                    pass

            with open(
                "/root/DashDisplay/can_config.json", "w", encoding="utf-8"
            ) as f:
                json.dump(self.can_config, f, indent=4)
            if save_btn.winfo_exists():
                save_btn.config(text="✔ Saved", bg="#006600")

            def reset_btn_color():
                try:
                    if save_btn.winfo_exists():
                        save_btn.config(text="⛁ Save", bg="#008800")
                except Exception:
                    pass

            self.root.after(1500, reset_btn_color)

        btn_font = ("Helvetica", 14, "bold")
        button_holder = tk.Frame(scroll_frame, bg=BG_MAIN)
        button_holder.grid(
            row=len(self.warning_entries) + 1, column=0, columnspan=16, pady=30
        )

        save_btn = tk.Button(
            button_holder,
            text="⛁ Save",
            font=btn_font,
            width=12,
            bg="#008800",
            fg="white",
            command=save_warning_settings,
        )
        save_btn.pack(side="left", padx=20)
        tk.Button(
            button_holder,
            text="✗ Close",
            font=btn_font,
            width=12,
            bg="#880000",
            fg="white",
            command=lambda: (
                popup.destroy(),
                setattr(self, "warnings_popup_open", False),
            ),
        ).pack(side="left", padx=20)

        popup.update_idletasks()
        popup.deiconify()

    def open_inputs_popup(self):
        if getattr(self, "inputs_popup_open", False):
            return
        self.inputs_popup_open = True

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()

        popup = tk.Toplevel(self.root)
        popup.withdraw()
        popup.configure(highlightbackground="black", highlightthickness=2)
        popup.title("Inputs Calibration")
        popup.configure(bg="#181818")
        popup.geometry(f"{sw}x{sh}+0+0")
        popup.attributes("-fullscreen", True)
        popup.bind("<Escape>", lambda e: popup.destroy())

        BG_MAIN = "#181818"
        BG_ENTRY = "#333333"
        FG_TEXT = "white"
        FG_LBL = "#FFD700"
        FONT_HEAD = ("Helvetica", 14, "bold")
        FONT_BODY = ("Helvetica", 12, "bold")

        container = tk.Frame(popup, bg=BG_MAIN)
        container.pack(fill="both", expand=True)

        canvas = tk.Canvas(
            container, borderwidth=0, bg=BG_MAIN, highlightthickness=0
        )
        v_scroll = tk.Scrollbar(
            container, orient="vertical", command=canvas.yview, bg=BG_MAIN
        )
        canvas.configure(yscrollcommand=v_scroll.set)

        canvas.pack(side="left", fill="both", expand=True)
        v_scroll.pack(side="right", fill="y")

        scroll_frame = tk.Frame(canvas, bg=BG_MAIN)
        scroll_window = canvas.create_window(
            0, 0, window=scroll_frame, anchor="nw", width=sw
        )

        def configure_canvas(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(scroll_window, width=event.width)

        canvas.bind("<Configure>", configure_canvas)

        for i in range(10):
            scroll_frame.grid_columnconfigure(i, weight=1)

        headers = [
            "Channel",
            "Enable",
            "Name",
            "Unit",
            "Min Volt",
            "Mid Volt",
            "Max Volt",
            "Min Value",
            "Mid Value",
            "Max Value",
        ]
        for col, h in enumerate(headers):
            pad_left = 2
            if col == 4:
                pad_left = 25
            if col == 7:
                pad_left = 25
            if col in [5, 6, 8, 9]:
                pad_left = 1
            tk.Label(
                scroll_frame, text=h, fg="#00aaff", bg=BG_MAIN, font=FONT_HEAD
            ).grid(row=0, column=col, padx=(pad_left, 2), pady=20)

        self.input_widgets = {}
        row = 1

        for ch in range(8):
            ch = str(ch)
            cfg = self.can_config["inputs"]["channels"].get(ch, {})
            w = {}
            self.input_widgets[ch] = w

            tk.Label(
                scroll_frame,
                text=f"CH {ch}",
                fg=FG_LBL,
                bg=BG_MAIN,
                font=("Helvetica", 13, "bold"),
            ).grid(row=row, column=0, pady=5)

            w["enabled"] = tk.IntVar(value=1 if cfg.get("enabled") else 0)
            btn = tk.Button(
                scroll_frame, width=6, font=("Helvetica", 10, "bold"), bd=0
            )

            def toggle(var=w["enabled"], b=btn):
                if var.get() == 1:
                    var.set(0)
                    b.config(text="OFF", bg="#444444", fg="gray")
                else:
                    var.set(1)
                    b.config(text="ON", bg="#00AA00", fg="white")

            if w["enabled"].get():
                btn.config(text="ON", bg="#00AA00", fg="white", command=toggle)
            else:
                btn.config(text="OFF", bg="#444444", fg="gray", command=toggle)
            btn.grid(row=row, column=1, padx=5, pady=5)

            def mk_entry(col, val, width=8, disable=False):
                e = tk.Entry(
                    scroll_frame,
                    width=width,
                    font=FONT_BODY,
                    justify="center",
                    bg=BG_ENTRY,
                    fg=FG_TEXT,
                    insertbackground="white",
                    bd=0,
                )
                e.insert(0, str(val))
                if disable:
                    e.config(state="disabled", fg="gray")
                pad_left = 2
                if col == 4:
                    pad_left = 25
                if col == 7:
                    pad_left = 25
                if col in [5, 6, 8, 9]:
                    pad_left = 1
                e.grid(row=row, column=col, padx=(pad_left, 2), pady=5, ipady=5)
                return e

            w["name"] = mk_entry(
                2, cfg.get("name", f"CH{ch}"), 15, disable=(ch in ["0", "1"])
            )
            unit_val = "Lt" if ch in ["0", "1"] else cfg.get("unit", "")
            w["unit"] = mk_entry(3, unit_val, 8, disable=(ch in ["0", "1"]))

            w["min_v"] = mk_entry(4, cfg.get("min_v", 0.0))
            w["mid_v"] = mk_entry(5, cfg.get("mid_v", 1.6))
            w["max_v"] = mk_entry(6, cfg.get("max_v", 3.0))

            w["min_val"] = mk_entry(7, cfg.get("min_val", 0.0))
            w["mid_val"] = mk_entry(8, cfg.get("mid_val", 50.0))
            w["max_val"] = mk_entry(9, cfg.get("max_val", 100.0))

            row += 1

        tk.Label(
            scroll_frame,
            text="Odometer (km)",
            fg="#00aaff",
            bg=BG_MAIN,
            font=FONT_HEAD,
        ).grid(row=row, column=0, columnspan=2, pady=40, sticky="e")
        odo_var = tk.StringVar(
            value=str(self.can_config["inputs"].get("odometer", 0.0))
        )
        tk.Entry(
            scroll_frame,
            textvariable=odo_var,
            width=15,
            font=FONT_BODY,
            justify="left",
            bg=BG_ENTRY,
            fg=FG_TEXT,
            insertbackground="white",
            bd=0,
        ).grid(row=row, column=2, padx=10, ipady=5)

        # Info Text
        info_text = (
            "Fuel Calibration: Fuel Table is INVERTED ( MinValue = Max Capacity"
            " of each tank in Lt )\nADC → ECU: MSB - UInt16 - 50ms (20 Hz) | "
            " FUEL EGT ADC2 ADC3 → ID 0x457 (1111) | ADC4 – ADC7 → ID 0x458"
            " (1112) "
        )
        tk.Label(
            scroll_frame,
            text=info_text,
            font=("Helvetica", 10, "italic"),
            fg="gray",
            bg=BG_MAIN,
            justify="left",
        ).grid(row=row, column=3, columnspan=7, sticky="w", padx=20)

        def save_inputs():
            old_channels = (
                self.can_config.get("inputs", {}).get("channels", {}).copy()
            )
            try:
                self.can_config["inputs"]["odometer"] = float(odo_var.get())
            except Exception:
                self.can_config["inputs"]["odometer"] = 0.0

            for ch, w in self.input_widgets.items():
                old_name = old_channels.get(ch, {}).get("name")
                new_name = w["name"].get()
                if old_name and new_name and old_name != new_name:
                    self.rename_adc_column_in_db(old_name, new_name)

                try:
                    self.can_config["inputs"]["channels"][ch] = {
                        "enabled": bool(w["enabled"].get()),
                        "name": new_name,
                        "unit": w["unit"].get(),
                        "min_v": float(w["min_v"].get()),
                        "mid_v": float(w["mid_v"].get()),
                        "max_v": float(w["max_v"].get()),
                        "min_val": float(w["min_val"].get()),
                        "mid_val": float(w["mid_val"].get()),
                        "max_val": float(w["max_val"].get()),
                    }
                except Exception:
                    pass

            save_can_config(self.can_config)
            if save_btn.winfo_exists():
                save_btn.config(text="✔ Saved", bg="#006600")

            def reset_btn():
                try:
                    if save_btn.winfo_exists():
                        save_btn.config(text="⛁ Save", bg="#008800")
                except Exception:
                    pass

            self.root.after(1500, reset_btn)

        def toggle_adc_output():
            self.adc_to_ecu_enabled = not self.adc_to_ecu_enabled
            self.can_config["inputs"]["adc_to_ecu_enabled"] = (
                self.adc_to_ecu_enabled
            )
            save_can_config(self.can_config)
            update_adc_btn()

        def update_adc_btn():
            if self.adc_to_ecu_enabled:
                adc_btn.config(
                    text="ADC → ECU ON", bg="#006400", activebackground="#008F00"
                )
            else:
                adc_btn.config(
                    text="ADC → ECU OFF", bg="#8B0000", activebackground="#A40000"
                )

        btn_font = ("Helvetica", 14, "bold")
        button_holder = tk.Frame(scroll_frame, bg=BG_MAIN)
        button_holder.grid(row=row + 1, column=0, columnspan=10, pady=30)

        adc_btn = tk.Button(
            button_holder,
            text="ADC → ECU OFF",
            font=btn_font,
            width=16,
            fg="white",
            command=toggle_adc_output,
        )
        update_adc_btn()
        adc_btn.pack(side="left", padx=20)

        save_btn = tk.Button(
            button_holder,
            text="⛁ Save",
            font=btn_font,
            width=12,
            bg="#008800",
            fg="white",
            command=save_inputs,
        )
        save_btn.pack(side="left", padx=20)

        tk.Button(
            button_holder,
            text="✗ Close",
            font=btn_font,
            width=12,
            bg="#880000",
            fg="white",
            command=lambda: (
                popup.destroy(),
                setattr(self, "inputs_popup_open", False),
            ),
        ).pack(side="left", padx=20)

        popup.update_idletasks()
        popup.deiconify()

    def open_display_popup(self):
        if getattr(self, "display_popup_open", False):
            return
        self.display_popup_open = True

        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()

        popup = tk.Toplevel(self.root)
        popup.withdraw()
        popup.configure(highlightbackground="black", highlightthickness=2)
        popup.title("Display Layout")
        popup.configure(bg="#181818")
        popup.geometry(f"{sw}x{sh}+0+0")
        popup.attributes("-fullscreen", True)
        popup.bind("<Escape>", lambda e: popup.destroy())

        BG_MAIN = "#181818"
        BG_ENTRY = "#333333"
        FG_TEXT = "white"
        FG_LBL = "#FFD700"
        FONT_HEAD = ("Helvetica", 14, "bold")
        FONT_BODY = ("Helvetica", 12, "bold")

        container = tk.Frame(popup, bg=BG_MAIN)
        container.pack(fill="both", expand=True)

        canvas = tk.Canvas(
            container, borderwidth=0, bg=BG_MAIN, highlightthickness=0
        )
        v_scroll = tk.Scrollbar(
            container, orient="vertical", command=canvas.yview, bg=BG_MAIN
        )
        canvas.configure(yscrollcommand=v_scroll.set)

        canvas.pack(side="left", fill="both", expand=True)
        v_scroll.pack(side="right", fill="y")

        scroll_frame = tk.Frame(canvas, bg=BG_MAIN)
        scroll_window = canvas.create_window(
            0, 0, window=scroll_frame, anchor="nw", width=sw
        )

        def configure_canvas(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(scroll_window, width=event.width)

        canvas.bind("<Configure>", configure_canvas)

        for i in range(3):
            scroll_frame.grid_columnconfigure(i, weight=1)

        available_signals = []
        available_signals.extend(
            list(self.can_config.get("parameters", {}).keys())
        )
        fuel_possible = False
        for ch, cfg in (
            self.can_config.get("inputs", {}).get("channels", {}).items()
        ):
            if ch in ["0", "1"]:
                if cfg.get("enabled"):
                    fuel_possible = True
                continue
            if cfg.get("enabled"):
                name = cfg.get("name")
                if name and name not in available_signals:
                    available_signals.append(name)
        if fuel_possible and "Fuel Level" not in available_signals:
            available_signals.append("Fuel Level")
        for extra in [
            "EGT",
            "Odometer",
            "G_Speed",
            "V_error",
            "Longitude",
            "Latitude",
        ]:
            if extra not in available_signals:
                available_signals.append(extra)
        available_signals = sorted(available_signals)

        display_cfg = self.can_config.get("display", {})
        columns_cfg = display_cfg.get("columns", {})
        cols_order = ["left", "middle", "right"]
        col_titles = {"left": "Left", "middle": "Middle", "right": "Right"}
        self.display_entries = {}

        def build_column(col_key, parent, col_index):
            col_cfg = columns_cfg.get(col_key, {})
            mode_val = col_cfg.get("mode", "6small")
            tiles = col_cfg.get("tiles", [])

            col_frame = tk.LabelFrame(
                parent,
                text=f"{col_titles[col_key]} Column",
                font=FONT_HEAD,
                fg="#00aaff",
                bg=BG_MAIN,
                bd=2,
            )
            col_frame.grid(row=1, column=col_index, padx=10, pady=20, sticky="nsew")

            entries = {}
            self.display_entries[col_key] = entries

            tk.Label(
                col_frame, text="Mode:", fg=FG_TEXT, bg=BG_MAIN, font=FONT_BODY
            ).grid(row=0, column=0, padx=5, pady=10, sticky="w")
            mode_cb = ttk.Combobox(
                col_frame,
                values=["2big", "6small"],
                width=10,
                state="readonly",
                font=FONT_BODY,
                justify="center",
            )
            if mode_val not in ("2big", "6small"):
                mode_val = "6small"
            mode_cb.set(mode_val)
            mode_cb.grid(row=0, column=1, padx=5, pady=10, sticky="w")
            entries["mode"] = mode_cb

            big_frame = tk.Frame(col_frame, bg=BG_MAIN)
            entries["big_frame"] = big_frame
            small_frame = tk.Frame(col_frame, bg=BG_MAIN)
            entries["small_frame"] = small_frame
            entries["big"] = []
            entries["small"] = []

            big_tiles = [t for t in tiles if t.get("type") == "big"]
            for i in range(2):
                s1 = big_tiles[i].get("signal", "") if i < len(big_tiles) else ""
                type_cb = ttk.Combobox(
                    big_frame,
                    values=["empty", "big"],
                    width=8,
                    state="readonly",
                    font=FONT_BODY,
                )
                sig1_cb = ttk.Combobox(
                    big_frame, values=available_signals, width=15, font=FONT_BODY
                )
                type_cb.set("big" if s1 else "empty")
                sig1_cb.set(s1)
                entries["big"].append({"type": type_cb, "sig1": sig1_cb})

            def rebuild_big():
                for w in big_frame.winfo_children():
                    w.grid_forget()
                for i in range(2):
                    tk.Label(
                        big_frame,
                        text=f"B{i+1}",
                        fg=FG_LBL,
                        bg=BG_MAIN,
                        font=FONT_BODY,
                    ).grid(row=i, column=0, padx=5, pady=10)
                    entries["big"][i]["type"].grid(row=i, column=1, padx=5, pady=10)
                    entries["big"][i]["sig1"].grid(row=i, column=2, padx=5, pady=10)

            for i in range(2):

                def chg(e, idx=i):
                    if entries["big"][idx]["type"].get() == "empty":
                        entries["big"][idx]["sig1"].set("")
                    rebuild_big()

                entries["big"][i]["type"].bind("<<ComboboxSelected>>", chg)
            rebuild_big()

            small_tiles = [
                t for t in tiles if t.get("type") in ("small", "split")
            ]
            for i in range(6):
                t = small_tiles[i] if i < len(small_tiles) else {}
                ttype = t.get("type", "small") if t else "empty"
                s1 = (
                    t.get("signal", "")
                    if ttype == "small"
                    else t.get("signal1", "")
                )
                s2 = t.get("signal2", "") if ttype == "split" else ""

                type_cb = ttk.Combobox(
                    small_frame,
                    values=["empty", "small", "split"],
                    width=8,
                    state="readonly",
                    font=FONT_BODY,
                )
                sig1_cb = ttk.Combobox(
                    small_frame, values=available_signals, width=15, font=FONT_BODY
                )
                sig2_cb = ttk.Combobox(
                    small_frame, values=available_signals, width=15, font=FONT_BODY
                )

                type_cb.set(ttype)
                sig1_cb.set(s1)
                sig2_cb.set(s2)
                entries["small"].append(
                    {"type": type_cb, "sig1": sig1_cb, "sig2": sig2_cb}
                )

            def rebuild_small():
                for w in small_frame.winfo_children():
                    w.grid_forget()
                r = 0
                for i in range(6):
                    tk.Label(
                        small_frame,
                        text=f"S{i+1}",
                        fg=FG_LBL,
                        bg=BG_MAIN,
                        font=FONT_BODY,
                    ).grid(row=r, column=0, padx=5, pady=5)
                    slot = entries["small"][i]
                    slot["type"].grid(row=r, column=1, padx=5, pady=5)
                    slot["sig1"].grid(row=r, column=2, padx=5, pady=5)
                    if slot["type"].get() == "split":
                        slot["sig2"].grid(row=r + 1, column=2, padx=5, pady=5)
                        r += 2
                    else:
                        r += 1

            for i in range(6):

                def chg_sm(e, idx=i):
                    if entries["small"][idx]["type"].get() != "split":
                        entries["small"][idx]["sig2"].set("")
                    rebuild_small()

                entries["small"][i]["type"].bind("<<ComboboxSelected>>", chg_sm)
            rebuild_small()

            def apply_mode(e=None):
                if mode_cb.get() == "2big":
                    small_frame.grid_forget()
                    big_frame.grid(
                        row=1, column=0, columnspan=3, padx=5, pady=5
                    )
                else:
                    big_frame.grid_forget()
                    small_frame.grid(
                        row=1, column=0, columnspan=3, padx=5, pady=5
                    )

            mode_cb.bind("<<ComboboxSelected>>", apply_mode)
            apply_mode()

        for idx, col_key in enumerate(cols_order):
            build_column(col_key, scroll_frame, idx)

        def save_display():
            new_display = {"columns": {}}
            for col_key, entries in self.display_entries.items():
                mode = entries["mode"].get()
                col_tiles = []
                if mode == "2big":
                    for slot in entries["big"]:
                        if slot["type"].get() == "big" and slot["sig1"].get():
                            col_tiles.append({"type": "big", "signal": slot["sig1"].get()})
                else:
                    for slot in entries["small"]:
                        t = slot["type"].get()
                        if t == "small" and slot["sig1"].get():
                            col_tiles.append(
                                {"type": "small", "signal": slot["sig1"].get()}
                            )
                        elif t == "split" and slot["sig1"].get():
                            col_tiles.append({
                                "type": "split",
                                "signal1": slot["sig1"].get(),
                                "signal2": slot["sig2"].get(),
                            })
                new_display["columns"][col_key] = {"mode": mode, "tiles": col_tiles}

            self.can_config["display"] = new_display
            with open(CONFIG_PATH, "w") as f:
                json.dump(self.can_config, f, indent=4)
            if save_btn.winfo_exists():
                save_btn.config(text="✔ Saved", bg="#006600")
            self.root.after(
                1500,
                lambda: save_btn.winfo_exists()
                and save_btn.config(text="⛁ Save", bg="#008800"),
            )
            self.label_references.clear()
            self.create_squares([
                {
                    "name": k,
                    "number": self.sensor_values.get(k, 0),
                    "unit": self.get_unit_for_signal(k),
                }
                for k in self.sensor_values.keys()
            ])

        btn_font = ("Helvetica", 14, "bold")
        button_holder = tk.Frame(scroll_frame, bg=BG_MAIN)
        button_holder.grid(row=2, column=0, columnspan=3, pady=40)

        tk.Button(
            button_holder,
            text="✗ Close",
            font=btn_font,
            width=12,
            bg="#880000",
            fg="white",
            command=lambda: (
                popup.destroy(),
                setattr(self, "display_popup_open", False),
            ),
        ).pack(side="right", padx=20)
        save_btn = tk.Button(
            button_holder,
            text="⛁ Save",
            font=btn_font,
            width=12,
            bg="#008800",
            fg="white",
            command=save_display,
        )
        save_btn.pack(side="right", padx=20)

        popup.update_idletasks()
        popup.deiconify()

    def toggle_grafana_db(self, event=None):
        current = getattr(self, "grafana_db_active", False)
        new_state = not current
        self.grafana_db_active = new_state

        if new_state:
            manage_service("mariadb", "start")
            self.grafana_icon_label.config(image=self.grafana_connected_icon)
            self.grafana_icon_label.image = self.grafana_connected_icon
            try:
                ip = os.popen("hostname -I").read().strip().split()[0]
            except Exception:
                ip = "UNKNOWN"

            messagebox.showinfo(
                "Grafana / MariaDB",
                "Η MariaDB είναι τώρα ενεργή.\n\n"
                "Στο Grafana (PC) βάλε datasource MariaDB:\n"
                f"Host: {ip}:3306\n"
                "Database: rx8data\n"
                "User / Password: αυτά που έχεις ορίσει\n",
            )
            print("✅ MariaDB for Grafana → ENABLED")
        else:
            manage_service("mariadb", "stop")
            self.grafana_icon_label.config(image=self.grafana_disconnected_icon)
            self.grafana_icon_label.image = self.grafana_disconnected_icon
            print("⏹ MariaDB for Grafana → DISABLED")


def main():
    data = [
        {"name": "Speed", "number": 0},
        {"name": "ECT", "number": 0},
        {"name": "IAT", "number": 0},
        {"name": "Oil Press", "number": 0},
        {"name": "Oil Temp", "number": 0},
        {"name": "Fuel Press", "number": 0},
        {"name": "Fuel Level", "number": 0},
        {"name": "Boost", "number": 0},
        {"name": "AFR", "number": 0},
        {"name": "TPS", "number": 0},
        {"name": "Ign Tim", "number": 0},
        {"name": "Inj DC", "number": 0},
        {"name": "Gear", "number": 0},
        {"name": "Volt", "number": 0},
        {"name": "Limit_Flags", "number": 0},
        {"name": "ECU_Fault", "number": 0},
    ]

    root = tk.Tk()
    app = CanDashboard(root, data)
    root.mainloop()


if __name__ == "__main__":
    main()
