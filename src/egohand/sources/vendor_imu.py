"""Direct VS030 LSM6DSV16X IIO acquisition over ADB.

Raw device timestamps are preserved with an explicit, unverified clock-domain tag.
This adapter deliberately does NOT claim camera/IMU synchronization or feed ORB-SLAM3.
"""
from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import queue
import struct
import subprocess
import threading
import time
from pathlib import Path

from .vendor_placeholder import ADB, CAMERA_SERIAL

log = logging.getLogger(__name__)
GYRO = "/sys/bus/iio/devices/iio:device1"
ACCEL = "/sys/bus/iio/devices/iio:device2"
RECORD = struct.Struct("<hhh2xq")  # scan indices 0..2 + 64-bit aligned timestamp
GYRO_SCALE = 0.000610865          # rad/s per LSB; read from device sysfs
ACCEL_SCALE = 0.002392822         # m/s^2 per LSB; read from device sysfs


@dataclass(frozen=True)
class RawIMUSample:
    sensor: str
    timestamp_ns: int
    xyz_si: tuple[float, float, float]
    clock_domain: str = "vs030_iio_realtime_unverified"
    timestamp_unit: str = "ns"


class VendorIMUSource:
    """Bounded direct IIO streams; restores every touched sysfs setting on stop."""

    def __init__(self, odr_hz: int = 480, queue_size: int = 8192):
        if odr_hz not in (120, 240, 480, 960):
            raise ValueError("odr_hz must be one of 120, 240, 480, 960")
        self.odr_hz = odr_hz
        self.samples: queue.Queue[RawIMUSample] = queue.Queue(maxsize=queue_size)
        self.processes: dict[str, subprocess.Popen] = {}
        self.threads: list[threading.Thread] = []
        self.stop_event = threading.Event()
        self.restore_script: str | None = None
        self.last_timestamp = {"gyro": None, "accel": None}
        self.stats = {"gyro": 0, "accel": 0, "dropped": 0, "non_monotonic": 0}
        self.running = False
        self.scales = {'gyro':GYRO_SCALE,'accel':ACCEL_SCALE}
        self.clock_domains = {'gyro':'vs030_iio_realtime_unverified','accel':'vs030_iio_realtime_unverified'}

    def _adb(self, *args, timeout=8, check=True):
        return subprocess.run([str(ADB), "-s", CAMERA_SERIAL, *args],
                              capture_output=True, timeout=timeout, check=check)

    def _configure(self):
        if not ADB.is_file():
            raise FileNotFoundError(ADB)
        # 每次打开都读取实际尺度、时钟和扫描布局；不得把旧固件的数值当作实时参数。
        for name,dev,prefix,scale_name in (('gyro',GYRO,'in_anglvel','in_anglvel_x_scale'),
                                            ('accel',ACCEL,'in_accel','in_accel_x_scale')):
            paths=[f'{dev}/{scale_name}',f'{dev}/current_timestamp_clock']
            for axis in ('x','y','z'):
                paths.extend([f'{dev}/scan_elements/{prefix}_{axis}_index',f'{dev}/scan_elements/{prefix}_{axis}_type'])
            paths.extend([f'{dev}/scan_elements/in_timestamp_index',f'{dev}/scan_elements/in_timestamp_type'])
            result=self._adb('shell','cat '+' '.join(paths))
            lines=[line.replace('\x00','').strip() for line in result.stdout.decode('utf8','strict').splitlines()]
            if len(lines)!=10:raise RuntimeError(f'Cannot read {name} IIO timestamp/scale/scan layout')
            scale=float(lines[0]);clock=lines[1]
            if not math.isfinite(scale) or scale<=0:raise RuntimeError(f'Invalid {name} IIO scale')
            if clock not in ('realtime','monotonic','monotonic_raw','boottime'):
                raise RuntimeError(f'Unsupported {name} IIO clock: {clock}')
            if lines[2:]!=['0','le:s16/16>>0','1','le:s16/16>>0','2','le:s16/16>>0','3','le:s64/64>>0']:
                raise RuntimeError(f'Unsupported {name} IIO scan layout: {lines[2:]}')
            self.scales[name]=scale;self.clock_domains[name]=f'vs030_iio_{clock}_unverified'
        if self.clock_domains['gyro']!=self.clock_domains['accel']:
            raise RuntimeError('IMU accelerometer and gyroscope clock domains differ')
        state = self._adb("shell",
            f"for d in {GYRO} {ACCEL}; do "
            "echo BEGIN:$d; cat $d/sampling_frequency $d/buffer/length $d/buffer/enable; "
            "for f in $d/scan_elements/*_en; do echo $(basename $f):$(cat $f); done; done")
        lines = state.stdout.decode("utf-8", "replace").splitlines()
        saved = {}
        current = None
        for line in lines:
            if line.startswith("BEGIN:"):
                current = line[6:]
                saved[current] = {"scan": {}}
            elif current and line.startswith("in_") and ":" in line:
                k, v = line.split(":", 1); saved[current]["scan"][k] = v
            elif current:
                saved[current].setdefault("base", []).append(line.strip())
        if set(saved) != {GYRO, ACCEL} or any(len(v.get("base", [])) != 3 for v in saved.values()):
            raise RuntimeError("Could not snapshot IIO settings; refusing to modify sensor")
        if any(v['base'][2] != '0' for v in saved.values()):
            raise RuntimeError('IMU IIO 缓冲区正在使用，拒绝中断现有采集')
        self.restore_script = self._settings_script(saved, restore=True)
        enable = [
            f"for d in {GYRO} {ACCEL}; do echo 0 > $d/buffer/enable; done",
            f"for d in {GYRO} {ACCEL}; do echo {self.odr_hz} > $d/sampling_frequency; echo 4096 > $d/buffer/length; done",
            f"for f in {GYRO}/scan_elements/*_en {ACCEL}/scan_elements/*_en; do echo 0 > $f; done",
            f"for f in {GYRO}/scan_elements/in_anglvel_x_en {GYRO}/scan_elements/in_anglvel_y_en {GYRO}/scan_elements/in_anglvel_z_en {GYRO}/scan_elements/in_timestamp_en "
            f"{ACCEL}/scan_elements/in_accel_x_en {ACCEL}/scan_elements/in_accel_y_en {ACCEL}/scan_elements/in_accel_z_en {ACCEL}/scan_elements/in_timestamp_en; do echo 1 > $f; done",
            f"for d in {GYRO} {ACCEL}; do echo 1 > $d/buffer/enable; done",
        ]
        self._adb("shell", "; ".join(enable))

    @staticmethod
    def _settings_script(saved, restore=False):
        commands = []
        for dev, state in saved.items():
            base = state["base"]
            freq, length, enabled = base[:3]
            commands += [f"echo 0 > {dev}/buffer/enable",
                         f"echo {freq} > {dev}/sampling_frequency",
                         f"echo {length} > {dev}/buffer/length"]
            for name, value in state["scan"].items():
                commands.append(f"echo {value} > {dev}/scan_elements/{name}")
            commands.append(f"echo {enabled} > {dev}/buffer/enable")
        return "; ".join(commands)

    def start(self):
        if self.running:
            return
        self.stop_event.clear()
        self.last_timestamp={'gyro':None,'accel':None}
        self.stats={'gyro':0,'accel':0,'dropped':0,'non_monotonic':0}
        while True:
            try:self.samples.get_nowait()
            except queue.Empty:break
        try:
            self._configure()
            for sensor, dev in (("gyro", GYRO), ("accel", ACCEL)):
                proc = subprocess.Popen([str(ADB), "-s", CAMERA_SERIAL, "exec-out", "cat", f"/dev/{dev.rsplit('/',1)[-1]}"],
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
                self.processes[sensor] = proc
                thread = threading.Thread(target=self._reader, args=(sensor, proc),
                                          name=f"VS030-{sensor}-IIO", daemon=True)
                self.threads.append(thread)
                thread.start()
            self.running = True
        except Exception:
            self.stop()
            raise

    def _parse_record(self, sensor, raw):
        if sensor not in ("gyro", "accel") or len(raw) != RECORD.size:
            raise ValueError("invalid IIO sensor or record length")
        x, y, z, stamp = RECORD.unpack(raw)
        previous = self.last_timestamp[sensor]
        if stamp <= 0 or (previous is not None and stamp <= previous):
            self.stats["non_monotonic"] += 1
            return None
        self.last_timestamp[sensor] = int(stamp)
        scale = self.scales[sensor]
        return RawIMUSample(sensor, int(stamp), (x*scale, y*scale, z*scale),self.clock_domains[sensor])

    def _reader(self, sensor, proc):
        try:
            while not self.stop_event.is_set():
                raw = proc.stdout.read(RECORD.size)
                if not raw:
                    if not self.stop_event.is_set():
                        log.warning("VS030 %s IIO stream ended", sensor)
                    break
                if len(raw) != RECORD.size:
                    self.stats["dropped"] += 1
                    break
                sample = self._parse_record(sensor, raw)
                if sample is None:
                    continue
                try:
                    self.samples.put_nowait(sample)
                    self.stats[sensor] += 1
                except queue.Full:
                    self.stats["dropped"] += 1
        except (OSError, ValueError) as exc:
            if not self.stop_event.is_set():
                log.warning("VS030 %s IIO read failed: %s", sensor, exc)

    def read(self, timeout=0.0):
        try:
            return self.samples.get(timeout=timeout) if timeout else self.samples.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self.stop_event.set()
        for proc in self.processes.values():
            if proc.poll() is None:
                proc.terminate()
        for proc in self.processes.values():
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait(timeout=2)
        self.processes.clear()
        for thread in self.threads:
            thread.join(timeout=1)
        self.threads.clear()
        if self.restore_script:
            try:
                self._adb("shell", self.restore_script)
            except Exception:
                log.exception("Failed to restore VS030 IIO settings")
            self.restore_script = None
        self.running = False

    def close(self):
        self.stop()
