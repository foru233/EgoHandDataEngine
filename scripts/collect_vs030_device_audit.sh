#!/bin/sh
# Read-only VS030 runtime audit. Does not write sysfs, change config, or restart services.
set +e

section() { echo; echo "========== $1 =========="; }

section "DEVICE / CLOCK"
uname -a
[ -r /etc/os-release ] && cat /etc/os-release
echo "date:"; date
echo "epoch_ns:"; date +%s%N
echo "uptime:"; cat /proc/uptime

section "CAMERA FSYNC SCRIPT"
if [ -r /root/scripts/cam_fsync.sh ]; then
  sed -n '1,240p' /root/scripts/cam_fsync.sh
else
  echo "/root/scripts/cam_fsync.sh not found/readable"
fi

section "CAMERA / IMU / SYNC PROCESSES"
ps 2>/dev/null | grep -Ei 'cam|imu|iio|sync|xd|ptp|ntp|time' | grep -v grep

section "IIO DEVICES AND SCALE"
found=0
for d in /sys/bus/iio/devices/iio:device*; do
  [ -d "$d" ] || continue
  found=1
  echo "--- $d ---"
  for f in name sampling_frequency in_accel_scale in_anglvel_scale in_temp_scale current_timestamp_clock; do
    if [ -r "$d/$f" ]; then printf '%s=' "$f"; cat "$d/$f"; fi
  done
  for f in "$d"/in_accel_*_raw "$d"/in_anglvel_*_raw "$d"/scan_elements/*_en "$d"/scan_elements/*_index "$d"/scan_elements/*_type; do
    [ -r "$f" ] || continue
    printf '%s=' "\${f##*/}"; cat "$f"
  done
done
[ "$found" -eq 0 ] && echo "No /sys/bus/iio/devices/iio:device* entries"

section "DEVICE NODES / PTP CLOCKS"
ls -l /dev/iio:device* /dev/ptp* 2>/dev/null
for d in /sys/class/ptp/ptp*; do
  [ -d "$d" ] || continue
  echo "--- $d ---"
  for f in clock_name max_adjustment; do
    [ -r "$d/$f" ] && { printf '%s=' "$f"; cat "$d/$f"; }
  done
done

section "XDYN CONFIG (ONLY TIMING-RELATED KEYS; SECRETS REDACTED)"
if [ -r /root/xdyn.json ] && command -v jq >/dev/null 2>&1; then
  jq -r '
    paths(scalars) as $p
    | ($p | map(tostring) | join(".")) as $k
    | select($k | test("sync|fsync|imu|camera|exposure|fps|timestamp|clock|trigger"; "i"))
    | if ($k | test("password|passwd|secret|token|ssid|key"; "i"))
      then "\($k)=<redacted>"
      else "\($k)=\(getpath($p))"
      end
  ' /root/xdyn.json
else
  echo "xdyn.json unavailable or jq not installed; full config intentionally not printed"
fi

section "RECENT RELEVANT LOG FILES (NAMES / TIMES ONLY)"
for d in /root/log/time_service /root/log/cs /root/log/client_inter /root/log/ver_mng /root/scripts /root/imudata /root/imgdata; do
  [ -d "$d" ] || continue
  echo "--- $d ---"
  ls -lt "$d" 2>/dev/null | head -n 15
done

section "KERNEL IMU / CAMERA / SYNC MESSAGES"
dmesg 2>/dev/null | grep -Ei 'imu|iio|fsync|sync|camera|sensor|ptp|clock' | tail -n 100

section "END"
echo "Read-only audit finished. No device settings were changed."
