# 结果导出必须保留坐标系/单位/valid/status/来源；不能只把 XYZ 数组当作完整训练数据。

import csv
import json
from pathlib import Path
import numpy as np
from ..core.types import serializable, result_from_dict,SCHEMA_VERSION

# 严格JSON，非有限坐标序列化为null，新旧字段保留各自schema版本。
def export_json(results,path):
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    Path(path).write_text(json.dumps(serializable(list(results)),ensure_ascii=False,allow_nan=False,indent=2),encoding="utf-8")

def load_json(path):
    return [result_from_dict(d) for d in json.loads(Path(path).read_text(encoding="utf-8"))]

# 一行一个关节；无效坐标留空，新不确定性字段追加在旧列后，便于旧工具兼容。
def export_csv(results,path):
    names=["schema_version","session_id","frame_id","timestamp_ns","source_type","calibration_id","coordinate_frame","unit","reconstruction_status","track_id","handedness","joint_id","joint_name","x_m","y_m","z_m","valid","status","epipolar_error_px","reprojection_error_px","derived_quality_score","depth_std_m","position_std_m","position_covariance_m2","uncertainty_status"]
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    with Path(path).open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=names);w.writeheader()
        for r in results:
            for h in r.hands:
                for j in h.joints:
                    row={k:getattr(r,k) for k in names[:9]}
                    row.update(track_id=h.track_id,handedness=h.handedness,joint_id=j.joint_id,joint_name=j.joint_name,valid=j.valid,status=str(j.status),epipolar_error_px=j.epipolar_error_px,reprojection_error_px=j.reprojection_error_px,derived_quality_score=j.quality_score)
                    row.update(zip(("x_m","y_m","z_m"),j.xyz_m if j.valid else (None,None,None)))
                    row.update(depth_std_m=j.depth_std_m,position_std_m=j.position_std_m,uncertainty_status=j.uncertainty_status,position_covariance_m2=json.dumps(serializable(j.position_covariance_m2),allow_nan=False) if j.position_covariance_m2 is not None else None)
                    w.writerow(row)

# 不使用pickle；H是帧内手槽位而不是跨帧身份，必须按 session_id+track_id 跟踪。
def export_npz(results,path):
    results=list(results)
    n=len(results); slots=max(2,max((len(r.hands) for r in results),default=0))
    xyz=np.full((n,slots,21,3),np.nan)
    valid=np.zeros((n,slots,21),dtype=bool)
    statuses=np.full((n,slots,21),"UNMATCHED",dtype="U32")
    tracks=np.full((n,slots),-1,dtype=np.int64)
    epi=np.full((n,slots,21),np.nan);repro=epi.copy();quality=epi.copy()
    depth_std=epi.copy();position_std=epi.copy();covariance=np.full((n,slots,21,3,3),np.nan)
    uncertainty_status=np.full((n,slots,21),'NOT_COMPUTED',dtype='U48')
    for i,r in enumerate(results):
        for k,h in enumerate(r.hands):
            tracks[i,k]=h.track_id
            for j in h.joints:
                xyz[i,k,j.joint_id]=j.xyz_m
                valid[i,k,j.joint_id]=j.valid
                statuses[i,k,j.joint_id]=j.status
                epi[i,k,j.joint_id]=j.epipolar_error_px if j.epipolar_error_px is not None else np.nan
                repro[i,k,j.joint_id]=j.reprojection_error_px if j.reprojection_error_px is not None else np.nan
                quality[i,k,j.joint_id]=j.quality_score if j.quality_score is not None else np.nan
                depth_std[i,k,j.joint_id]=j.depth_std_m if j.depth_std_m is not None else np.nan
                position_std[i,k,j.joint_id]=j.position_std_m if j.position_std_m is not None else np.nan
                if j.position_covariance_m2 is not None:covariance[i,k,j.joint_id]=j.position_covariance_m2
                uncertainty_status[i,k,j.joint_id]=j.uncertainty_status or 'NOT_COMPUTED'
    Path(path).parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(path,xyz_m=xyz,valid=valid,status=statuses,track_id=tracks,epipolar_error_px=epi,reprojection_error_px=repro,derived_quality_score=quality,depth_std_m=depth_std,position_std_m=position_std,position_covariance_m2=covariance,uncertainty_status=uncertainty_status,frame_id=np.array([r.frame_id for r in results],dtype=np.int64),timestamp_ns=np.array([r.timestamp_ns for r in results],dtype=np.int64),schema_version=np.array(SCHEMA_VERSION),result_schema_version=np.array([r.schema_version for r in results]),coordinate_frame=np.array("camera_left_optical"),unit=np.array("m"),source_type=np.array([r.source_type for r in results]),calibration_id=np.array([r.calibration_id or "" for r in results]),session_id=np.array([r.session_id for r in results]))

# 创建新的导出目录，避免无意覆盖上一次结果；JSON/CSV/NPZ由同一批结果生成。
def export_all(results,directory):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=False)
    results=list(results)
    export_json(results,directory/"results.json")
    export_csv(results,directory/"joints.csv")
    export_npz(results,directory/"joints.npz")
    # 世界系结果独立保存，旧相机系 CSV / NPZ 不被覆盖。
    world=[r.world_hands for r in results if r.world_hands is not None]
    if world:
        (directory/"world_hands.json").write_text(json.dumps(serializable(world),ensure_ascii=False,allow_nan=False,indent=2),encoding="utf-8")
    return directory
