"""从公开镜像下载 FreiHAND 评测集均匀采样图片；标注另取官方源。"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import json

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'samples/date/RGB_Hands/FreiHAND/mirror_eval_rgb'
COUNT = 200
TOTAL = 3960
URL = 'https://huggingface.co/datasets/nalbikyungmoon/FreiHAND_pub_v2_evaluation/resolve/main/'

def download(index):
    name = f'{index:08d}.jpg'
    dest = OUT / name
    if dest.exists() and dest.stat().st_size > 1000:
        return index, True
    result = subprocess.run(['curl.exe','--silent','--show-error','--fail','--location','--max-time','90', '--output',str(dest),URL+name], capture_output=True)
    return index, result.returncode == 0 and dest.exists() and dest.stat().st_size > 1000

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    indices = [round(i*(TOTAL-1)/(COUNT-1)) for i in range(COUNT)]
    with ThreadPoolExecutor(max_workers=16) as pool:
        result = list(pool.map(download, indices))
    (OUT.parent/'mirror_manifest.json').write_text(json.dumps({'source': URL, 'sampling': 'uniform across 3960 evaluation indices', 'requested': indices, 'succeeded': [i for i,ok in result if ok], 'failed': [i for i,ok in result if not ok]}, indent=2), encoding='utf8')
    print('downloaded',sum(ok for _,ok in result),'of',len(result),flush=True)

if __name__ == '__main__':
    main()
