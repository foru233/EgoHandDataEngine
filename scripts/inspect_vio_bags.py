"""只读检查 ROS1 bag 连接与传感器消息；不借用 GT 进行定位。"""
from pathlib import Path
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore
import json,sys
root=Path('samples/date/VIO')
store=get_typestore(Stores.ROS1_NOETIC)
for name,path in [('euroc',root/'EuRoC/MH_01_easy/MH_01_easy.bag'),('tumvi',root/'TUM-VI/room4/dataset-room4_512_16.bag')]:
 with Reader(path) as reader:
  print(name,'duration_ns',reader.duration,'connections',flush=True)
  for c in reader.connections:print(c.topic,c.msgtype,c.msgcount,flush=True)
  seen=set()
  for c,t,raw in reader.messages():
   if c.topic in seen:continue
   seen.add(c.topic)
   try:
    m=store.deserialize_ros1(raw,c.msgtype)
    if c.msgtype=='sensor_msgs/msg/Image':print('IMAGE',c.topic,'bag_ns',t,'encoding',m.encoding,'size',m.width,m.height,'step',m.step,'header',m.header.stamp.sec,m.header.stamp.nanosec,flush=True)
    elif c.msgtype=='sensor_msgs/msg/Imu':print('IMU',c.topic,'bag_ns',t,'header',m.header.stamp.sec,m.header.stamp.nanosec,'accel',m.linear_acceleration,'gyro',m.angular_velocity,flush=True)
    else:print('OTHER',c.topic,c.msgtype,flush=True)
   except Exception as e:print('ERROR',c.topic,repr(e),flush=True)
   if len(seen)==len(reader.connections):break
