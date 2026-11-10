"""读取已下载的官方 ZIP 中央目录，不执行解压。"""
import struct
import zlib
from pathlib import Path

path = Path(__file__).resolve().parents[1]/'samples/date/RGB_Hands/FreiHAND/zip_directory.bin'
data = path.read_bytes()
i = 0
checks = {}
while data[i:i+4] == b'PK\x01\x02':
    header = struct.unpack_from('<4s6H3I5H2I', data, i)
    name_len, extra_len, comment_len = header[10:13]
    name = data[i+46:i+46+name_len].decode('utf8', errors='replace')
    if name.startswith('evaluation/rgb/') and name.endswith('.jpg'):
        checks[Path(name).name] = header[7]
    i += 46+name_len+extra_len+comment_len
images=list((path.parent/'mirror_eval_rgb').glob('*.jpg'))
bad=[p.name for p in images if zlib.crc32(p.read_bytes()) != checks.get(p.name)]
print('images',len(images),'matched',len(images)-len(bad),'mismatched',bad)
