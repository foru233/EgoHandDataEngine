"""ROS1 bag 只读转换：原始纳秒时间戳、无损灰度 PNG、真实 IMU。"""
import argparse,csv,json,time
from pathlib import Path
import cv2,numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore

ROOT=Path('samples/date/VIO')
DATA={'euroc':ROOT/'EuRoC/MH_01_easy/MH_01_easy.bag','tumvi':ROOT/'TUM-VI/room4/dataset-room4_512_16.bag'}
def ns(stamp):return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
def convert(name):
 source=DATA[name];out=ROOT/'converted'/('EuRoC/MH_01_easy' if name=='euroc' else 'TUM-VI/room4')
 if out.exists():raise FileExistsError('目标已存在，拒绝覆盖：'+str(out))
 for side in ('cam0','cam1'):(out/'mav0'/side/'data').mkdir(parents=True)
 (out/'mav0/imu0').mkdir(parents=True)
 store=get_typestore(Stores.ROS1_NOETIC);times={k:[] for k in ('cam0','cam1','imu0')}; meta={};started=time.perf_counter()
 with (out/'mav0/imu0/data.csv').open('x',encoding='utf8',newline='') as csvfile,Reader(source) as reader:
  writer=csv.writer(csvfile,lineterminator='\n');writer.writerow(['#timestamp [ns]','w_RS_S_x [rad s^-1]','w_RS_S_y','w_RS_S_z','a_RS_S_x [m s^-2]','a_RS_S_y','a_RS_S_z'])
  selected=[c for c in reader.connections if c.topic in ('/cam0/image_raw','/cam1/image_raw','/imu0')]
  for c,bag_ts,raw in reader.messages(connections=selected):
   msg=store.deserialize_ros1(raw,c.msgtype);key=c.topic.split('/')[1];t=ns(msg.header.stamp)
   if times[key] and t<=times[key][-1]:raise ValueError(f'{key} 传感器时间戳不严格递增：{t}')
   times[key].append(t)
   if key=='imu0':
    g=msg.angular_velocity;a=msg.linear_acceleration;writer.writerow((t,g.x,g.y,g.z,a.x,a.y,a.z));continue
   enc=msg.encoding;bits={'mono8':8,'mono16':16}.get(enc)
   if bits is None:raise ValueError('不支持的图像格式 '+enc)
   dtype=np.dtype('u1' if bits==8 else ('>u2' if msg.is_bigendian else '<u2'))
   data=np.asarray(msg.data,dtype=np.uint8);rows=data.reshape(msg.height,msg.step)
   image=rows[:,:msg.width*(bits//8)].copy().view(dtype).reshape(msg.height,msg.width).astype(np.uint8 if bits==8 else np.uint16)
   dest=out/'mav0'/key/'data'/f'{t}.png'
   if dest.exists() or not cv2.imwrite(str(dest),image):raise IOError('PNG 写入失败或重名 '+str(dest))
   if key not in meta:meta[key]={'encoding':enc,'width':msg.width,'height':msg.height,'bit_depth':bits,'header_to_bag_ns_first':int(bag_ts)-t}
  stats={'dataset':name,'source':str(source),'topics':{k:{'count':len(v),'first_ns':v[0] if v else None,'last_ns':v[-1] if v else None,'min_dt_ns':min(np.diff(v).tolist()) if len(v)>1 else None,'max_dt_ns':max(np.diff(v).tolist()) if len(v)>1 else None} for k,v in times.items()},'image':meta}
  common=sorted(set(times['cam0'])&set(times['cam1']));stats['paired_exact_timestamps']=len(common);stats['left_only']=len(set(times['cam0'])-set(times['cam1']));stats['right_only']=len(set(times['cam1'])-set(times['cam0']))
  if not common:raise ValueError('没有精确同步的双目时间戳，不允许猜测配对')
  (out/'timestamps.txt').write_text(''.join(str(t)+'\n' for t in common),encoding='utf8')
  checks={}
  for label,index in [('first',0),('middle',len(common)//2),('last',len(common)-1)]:
   t=common[index];checks[label]={}
   for side in ('cam0','cam1'):
    p=out/'mav0'/side/'data'/f'{t}.png';raw=cv2.imread(str(p),cv2.IMREAD_UNCHANGED);gray=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
    checks[label][side]={'timestamp_ns':t,'dtype':str(raw.dtype),'raw_min':int(raw.min()),'raw_max':int(raw.max()),'gray8_min':int(gray.min()),'gray8_max':int(gray.max()),'gray8_std':float(gray.std())}
  stats['decoded_samples']=checks;stats['tumvi_16bit_policy']='保留原始16位无损PNG；官方示例使用OpenCV IMREAD_GRAYSCALE读取为8位' if name=='tumvi' else None
  stats['elapsed_s']=time.perf_counter()-started;(out/'conversion_report.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(stats,ensure_ascii=False,indent=2),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('dataset',choices=list(DATA));convert(p.parse_args().dataset)
