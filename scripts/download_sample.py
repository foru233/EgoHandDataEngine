"""Public real photo for 2D inference only. Never a calibrated stereo pair."""
import hashlib
import json
from datetime import datetime,timezone
from pathlib import Path
from urllib.request import Request,urlopen

URL="https://c.pxhere.com/photos/2c/2d/hand_palm_open_give_take-990733.jpg!d"
PAGE="https://pxhere.com/en/photo/990733"

def main():
    root=Path("samples/public");root.mkdir(parents=True,exist_ok=True)
    with urlopen(Request(URL,headers={"User-Agent":"EgoHandDataEngine/0.1"}),timeout=60) as f: data=f.read()
    (root/"hand.jpg").write_bytes(data)
    (root/"provenance.json").write_text(json.dumps({"source_page":PAGE,"download_url":URL,"license":"CC0 per source page","license_url":"https://creativecommons.org/publicdomain/zero/1.0/","author":"source page does not name a photographer","downloaded_at":datetime.now(timezone.utc).isoformat(),"sha256":hashlib.sha256(data).hexdigest(),"purpose":"Real Image Detection Mode, monocular 2D only","modifications":"none"},indent=2),encoding="utf-8")
    print(root/"hand.jpg")

if __name__=="__main__": main()
