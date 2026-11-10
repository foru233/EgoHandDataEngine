# 覆盖率与误差必须并列；缺失/全部拒绝不等于0mm，指标N/A应保留原因。

"""Coverage-aware statistics; unavailable metrics have explicit reasons."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment


def write_csv(path,records,fields=None):
    fields=fields or list(dict.fromkeys(k for r in records for k in r)) or ['reason']
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(records)


# 只统计有限有效误差；没有样本时均值/分位数为 None，不能返回0制造好成绩。
def distribution(values):
    v=np.asarray(values,dtype=float);v=v[np.isfinite(v)]
    return {'count':len(v),'mean':float(v.mean()) if len(v) else None,
        'median':float(np.median(v)) if len(v) else None,
        'p90':float(np.percentile(v,90)) if len(v) else None,
        'p95':float(np.percentile(v,95)) if len(v) else None,
        'max':float(v.max()) if len(v) else None}


# 评测参数位于函数签名前部：80px最大匹配距离、10px替代解间隔；不是重建引擎阈值。
def match_2d(detections,truth_uv,max_error_px=80.,margin_px=10.):
    """Evaluation-only matching in one view. Never uses predicted 3D or GT labels.

    Ambiguous pixel assignments stay unmatched. This matching is not used to
    improve the system's stereo pairing, tracking, or coordinates.
    """
    sides=list(truth_uv)
    if not detections or not sides:return {},{}
    costs=np.full((len(detections),len(sides)),1e6)
    for i,d in enumerate(detections):
        for j,side in enumerate(sides):
            uv,mask=truth_uv[side]
            valid=mask&d.valid&np.isfinite(d.pixel_landmarks).all(axis=1)
            if valid.sum()>=4:costs[i,j]=np.median(np.linalg.norm(d.pixel_landmarks[valid]-uv[valid],axis=1))
    n,m=costs.shape
    def solve(C):
        padded=np.full((n+m,n+m),1e6);padded[:n,:m]=C
        for i in range(n):padded[i,m+i]=max_error_px
        for j in range(m):padded[n+j,j]=0
        padded[n:,m:]=0;r,c=linear_sum_assignment(padded)
        return {(int(i),int(j)) for i,j in zip(r,c) if i<n and j<m and C[i,j]<max_error_px},float(padded[r,c].sum())
    pairs,total=solve(costs);accepted={};errors={}
    for i,j in pairs:
        alternate=costs.copy();alternate[i,j]=1e6;_,cost=solve(alternate)
        if cost-total>=margin_px:accepted[i]=sides[j];errors[i]=float(costs[i,j])
    return accepted,errors


# 每组分母包括漏检和被拒绝的可比较 GT，误差只对有效点计算，不能隐藏低覆盖率。
def grouped_joint_errors(rows,key):
    groups={}
    for row in rows:groups.setdefault(row[key],[]).append(row)
    return [{key:k,'gt_joints':len(v),'valid_joints':sum(r['valid'] for r in v),
        'coverage':sum(r['valid'] for r in v)/len(v),
        **{'error_mm_'+a:b for a,b in distribution([r['error_mm'] for r in v if r['error_mm'] is not None]).items()}}
        for k,v in sorted(groups.items(),key=lambda x:str(x[0]))]


# Coverage–Error 是标准差阈值的事后筛选，不代表重新运行整套关联后的成功概率。
def save_plots(output,rows,metrics):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output=Path(output)
    errors=[r['error_mm'] for r in rows if r['valid'] and r['error_mm'] is not None]
    fig,ax=plt.subplots(figsize=(7,4))
    if errors:ax.hist(errors,bins=30);ax.set_xlabel('Absolute 3D joint error (mm)');ax.set_ylabel('Valid comparable joints')
    else:ax.text(.5,.5,'N/A: no verified comparable valid 3D joints',ha='center',va='center',transform=ax.transAxes)
    ax.set_title('Error distribution — valid joints only; see coverage');fig.tight_layout();fig.savefig(output/'error_distribution.png',dpi=130);plt.close(fig)
    points=[]
    for threshold in (.002,.005,.01,.02,.03,.05,.1,1e9):
        selected=[r for r in rows if r['valid'] and r.get('position_std_m') is not None and r['position_std_m']<=threshold]
        values=[r['error_mm'] for r in selected if r['error_mm'] is not None]
        points.append({'position_std_threshold_m':None if threshold==1e9 else threshold,
            'coverage_all_gt':len(selected)/len(rows) if rows else 0.,
            'valid_comparable_joints':len(selected),'mpjpe_mm':float(np.mean(values)) if values else None})
    write_csv(output/'coverage_error.csv',points)
    fig,ax=plt.subplots(figsize=(7,4));p=[p for p in points if p['mpjpe_mm'] is not None]
    if p:ax.plot([100*x['coverage_all_gt'] for x in p],[x['mpjpe_mm'] for x in p],'o-')
    else:ax.text(.5,.5,'N/A: verified GT and accepted joints required',ha='center',va='center',transform=ax.transAxes)
    ax.set_xlabel('Coverage of ALL comparable GT joints (%)');ax.set_ylabel('MPJPE (mm)')
    ax.set_title('Post-hoc uncertainty threshold sensitivity (not calibrated probability)')
    fig.tight_layout();fig.savefig(output/'coverage_error.png',dpi=130);plt.close(fig)
