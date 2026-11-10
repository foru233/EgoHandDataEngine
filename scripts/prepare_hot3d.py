"""Inspect HOT3D-Clips archives without extracting images or trusting folder names."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys
import tarfile
import subprocess
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from egohand.benchmarks.provenance import sha256

REVISION='30fe9674782f32e1e5edba98476b6ff4300132c5'


def inspect_archive(path, official_records=(), output=None):
    # 先对整包 SHA256 查官方 split，再检查成员完整性；不信任本地 train/test 文件夹名称。
    digest=sha256(path)
    official=next((r for r in official_records if r.get('lfs',{}).get('oid')==digest),None)
    report={'path':str(path.resolve()),'size_bytes':path.stat().st_size,'sha256':digest,
        'official_hash_verified':official is not None,
        'official_path':official['path'] if official else None,'revision':REVISION,
        'selected_streams':['1201-1','1201-2'],'excluded_rgb_stream':'214-1',
        'sync_quality':'DATASET_DOCUMENTED','hardware_sync_precision_verified':False}
    offsets=[];models=set();devices=set();missing=[];previous={};hashes={}
    with tarfile.open(path) as archive:
        members=archive.getmembers(); names=[m.name for m in members]
        if len(set(names))!=len(names):raise ValueError('Duplicate archive member')
        for m in members:
            p=PurePosixPath(m.name)
            if p.is_absolute() or '..' in p.parts or not m.isfile():
                raise ValueError('Unsafe archive member: '+m.name)
            # 逐成员完整读取，连未参与双目的 RGB 负载也检查截断；不代表把 RGB 配入双目。
            # Read every member fully: truncated payloads must fail even for unused RGB.
            payload=archive.extractfile(m).read()
            if len(payload)!=m.size:raise ValueError('Truncated member: '+m.name)
            if m.name.endswith('.json') or m.name=='__hand_shapes.json__':json.loads(payload)
        frames=sorted(n.split('.')[0] for n in names if n.endswith('.info.json'))
        has_shape='__hand_shapes.json__' in names
        if has_shape:
            shape=archive.extractfile('__hand_shapes.json__').read()
            hashes['__hand_shapes.json__']=hashlib.sha256(shape).hexdigest()
            if output:
                target=Path(output)/path.stem/'annotations';target.mkdir(parents=True,exist_ok=True)
                (target/'__hand_shapes.json__').write_bytes(shape)
        gt_frames=0
        for frame in frames:
            required=[frame+s for s in ('.cameras.json','.info.json','.image_1201-1.jpg','.image_1201-2.jpg')]
            missing.extend(n for n in required if n not in names)
            if any(n not in names for n in required):continue
            info=json.load(archive.extractfile(frame+'.info.json'))
            cams=json.load(archive.extractfile(frame+'.cameras.json'))
            devices.add(info['device'])
            timestamps=info['image_timestamps_ns']
            offsets.append(abs(timestamps['1201-1']-timestamps['1201-2']))
            for stream in ('1201-1','1201-2'):
                if stream in previous and timestamps[stream]<=previous[stream]:raise ValueError('Non-monotonic sensor timestamp')
                previous[stream]=timestamps[stream]
                calibration=cams[stream]['calibration'];models.add(calibration['projection_model_type'])
                payload=archive.extractfile(frame+'.image_'+stream+'.jpg').read()
                image=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_UNCHANGED)
                if image is None or image.ndim!=2 or image.shape!=(calibration['image_height'],calibration['image_width']):
                    raise ValueError('Expected native monochrome stream dimensions: '+stream)
            name=frame+'.hands.json'
            if name in names:
                payload=archive.extractfile(name).read();hands=json.loads(payload)
                if any(h and 'umetrack_pose' in h for h in hands.values()):gt_frames+=1
                hashes[name]=hashlib.sha256(payload).hexdigest()
                if output:
                    target=Path(output)/path.stem/'annotations';target.mkdir(parents=True,exist_ok=True)
                    (target/name).write_bytes(payload)
        if missing:raise ValueError('Missing required stereo members: '+repr(missing[:5]))
    report.update(member_count=len(members),frames=len(frames),gt_hand_frames=gt_frames,
        has_hand_shape=has_shape,camera_models=sorted(models),devices=sorted(devices),
        timestamp_offset_ns={'min':min(offsets) if offsets else None,'max':max(offsets) if offsets else None},
        annotation_sha256=hashes,structure_status='SOFTWARE_VERIFIED',
        gt_status='AVAILABLE_NATIVE_MODEL_PARAMETERS' if gt_frames and has_shape else 'BLOCKED_BY_DATA',
        raw_data_modified=False)
    if official and official['path'].startswith('test_'):
        report['gt_restriction']='Official test split: withheld hands.json; no absolute 3D evaluation'
    return report


def main():
    # 参数入口：数据目录、独立输出目录、已有 Torch 解释器和官方模型代码目录。
    # Torch 只在转换子进程中使用，不向主 .venv 自动安装大型模型依赖。
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset-root',type=Path,default=Path('samples/date/hot3d'))
    p.add_argument('--output',type=Path)
    p.add_argument('--torch-python',type=Path)
    p.add_argument('--toolkit-root',type=Path)
    args=p.parse_args();out=args.output or args.dataset_root/'prepared';out.mkdir(parents=True,exist_ok=True)
    metadata=args.dataset_root/'official_file_metadata.json'
    official=json.loads(metadata.read_text(encoding='utf8')) if metadata.exists() else []
    records=[]
    for archive in sorted(args.dataset_root.rglob('clip-*.tar')):
        try:
            result=inspect_archive(archive,official,out)
            if args.torch_python and args.toolkit_root and result['gt_hand_frames']:
                command=[str(args.torch_python),str(Path(__file__).with_name('convert_hot3d_umetrack.py')),
                    '--toolkit-root',str(args.toolkit_root),'--archive',str(archive),
                    '--output',str(out/archive.stem/'umetrack_world.json')]
                subprocess.run(command,check=True,timeout=180)
            records.append(result)
        except Exception as e:records.append({'path':str(archive),'status':'BLOCKED_BY_DATA','reason':str(e)})
    (out/'inspection.json').write_text(json.dumps(records,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps([{k:r.get(k) for k in ['path','frames','gt_hand_frames','official_path','official_hash_verified','gt_status','reason']} for r in records],indent=2))
    return int(any('reason' in r for r in records))


if __name__=='__main__':sys.exit(main())
