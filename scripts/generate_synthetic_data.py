import argparse
from pathlib import Path
from egohand.sources.synthetic import SyntheticStereoSource,SCENARIOS
from egohand.storage.recorder import Recorder

def main():
    p=argparse.ArgumentParser();p.add_argument("--frames",type=int,default=12);p.add_argument("--output",default="samples/synthetic")
    a=p.parse_args()
    for scenario in SCENARIOS:
        source=SyntheticStereoSource(scenario,frames=a.frames);source.open();source.start()
        path=Path(a.output)/scenario
        if path.exists():print(f"Already exists, preserved: {path}");continue
        writer=Recorder(path,source.get_metadata(),source.get_calibration(),{"scenario":scenario,"seed":42})
        try:
            for _ in range(a.frames):writer.write_raw(source.read())
        finally:writer.close();source.close()
        print(path)

if __name__=="__main__":main()
