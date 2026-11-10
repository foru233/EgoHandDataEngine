"""校验并提取官方 ZIP 范围下载的标注条目。"""
import struct
import zlib
from pathlib import Path

root=Path(__file__).resolve().parents[1]/'samples/date/RGB_Hands/FreiHAND'
for name,expected_size,expected_crc in [('evaluation_K',337775,0x4c4504dc),('evaluation_xyz',5506793,0x90ce7f70)]:
    data=(root/(name+'.entry')).read_bytes()
    header=struct.unpack_from('<4s5H3I2H',data)
    if header[0]!=b'PK\x03\x04' or header[3]!=8:
        raise ValueError('非法 ZIP 本地头或压缩格式')
    offset=30+header[-2]+header[-1]
    output=zlib.decompress(data[offset:],-15)
    if len(output)!=expected_size or zlib.crc32(output)!=expected_crc:
        raise ValueError('官方标注 CRC 或大小不匹配')
    (root/(name+'.json')).write_bytes(output)
    print(name,len(output),'CRC OK')
