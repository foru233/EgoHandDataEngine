import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from egohand.benchmarks.injections import run_matrix,measure_case
from egohand.benchmarks.metrics import write_csv


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path('benchmark-results/sync_injection'));args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True);rows=run_matrix()
    (args.output/'metrics.json').write_text(json.dumps(rows,indent=2,allow_nan=False),encoding='utf8');write_csv(args.output/'metrics.csv',rows)
    sensitivity=[measure_case(0,0,'horizontal',sigma) for sigma in (.5,1.,2.,4.)]
    write_csv(args.output/'pixel_noise_sensitivity.csv',sensitivity)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,axis in zip(axes,('horizontal','vertical','depth')):
        for speed in (0.,.05,.3,1.):
            a=[r for r in rows if r['axis']==axis and r['speed_m_s']==speed]
            ax.plot([r['offset_ms'] for r in a],[r['raw_geometric_error_mm'] for r in a],'o-',label=f'{speed} m/s')
        ax.set_title(axis);ax.set_xlabel('Right image time offset (ms)');ax.set_ylabel('Accepted geometric 3D error (mm)');ax.legend()
    fig.tight_layout();fig.savefig(args.output/'sync_error.png',dpi=140);plt.close(fig)
    print(json.dumps({'cases':len(rows),'horizontal_high_5ms':next(r for r in rows if r['axis']=='horizontal' and r['speed_m_s']==1. and r['offset_ms']==5)},indent=2))


if __name__=='__main__':main()
