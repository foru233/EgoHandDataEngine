import time, json
from egohand.sources.vendor_imu import VendorIMUSource
s=VendorIMUSource(odr_hz=480)
try:
    s.start()
    t=time.monotonic()+2.0
    seen={"gyro":0,"accel":0};first={};last={}
    while time.monotonic()<t:
        x=s.read(timeout=0.05)
        if x is None: continue
        seen[x.sensor]+=1
        first.setdefault(x.sensor,x.timestamp_ns);last[x.sensor]=x.timestamp_ns
    print(json.dumps({"samples":seen,"first":first,"last":last,"stats":s.stats,"clock_domain":"vs030_iio_realtime_unverified"},indent=2))
finally:
    s.stop()
