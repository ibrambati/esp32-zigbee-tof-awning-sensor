#!/usr/bin/env python3
"""
Crea l'immagine OTA Zigbee (.zigbee) e l'indice locale per Zigbee2MQTT a partire dal .bin del firmware.

Il formato del file e' quello dello standard Zigbee (header OTA + sotto-elemento 0x0000 "upgrade image"),
lo stesso prodotto da image_builder_tool.py di esp-zigbee-sdk (CC0).

Uso (dalla radice del repo, dopo `arduino-cli compile --export-binaries arduino/esp32_zigbee_awning`):

  python3 z2m/ota/make_ota.py \
      --bin arduino/esp32_zigbee_awning/build/esp32.esp32.esp32h2/esp32_zigbee_awning.ino.bin \
      --version 0x00010001

--manuf-id, --image-type e --model-id devono coincidere con le costanti del firmware
(OTA_MANUFACTURER, OTA_IMAGE_TYPE) e con il modello del converter ("AwningToF").
--version deve essere PIU' ALTA di quella gia' installata sul dispositivo (FW_VERSION nello sketch).
"""
import argparse
import hashlib
import json
import os
import struct
import sys


def build_ota(app: bytes, manuf: int, image_type: int, version: int, header_string: str) -> bytes:
    hs = header_string.encode("ascii")
    if len(hs) > 32:
        raise ValueError("header string: max 32 byte")
    hs = hs.ljust(32, b"\x00")
    element = struct.pack("<HI", 0x0000, len(app)) + app            # tag 0x0000 = upgrade image
    header_len = 4 + 2 + 2 + 2 + 2 + 2 + 4 + 2 + 32 + 4               # 56, nessun campo opzionale
    total = header_len + len(element)
    header = struct.pack("<IHHHHHIH32sI", 0x0BEEF11E, 0x0100, header_len, 0, manuf, image_type, version, 0x0002, hs, total)
    return header + element


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", required=True, help="firmware applicativo (.ino.bin, NON il merged)")
    ap.add_argument("--version", required=True, type=lambda x: int(x, 0), help="file version, es. 0x00010001")
    ap.add_argument("--manuf-id", type=lambda x: int(x, 0), default=0x1001)
    ap.add_argument("--image-type", type=lambda x: int(x, 0), default=0x1011)
    ap.add_argument("--model-id", default="AwningToF")
    ap.add_argument("--out-dir", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--base-url", default="https://raw.githubusercontent.com/ibrambati/esp32-zigbee-tof-awning-sensor/main/z2m/ota",
                    help="URL pubblico della cartella che conterra' index.json e i .zigbee (o un percorso locale)")
    a = ap.parse_args()

    app = open(a.bin, "rb").read()
    if len(app) < 1024 or app[0] != 0xE9:       # ogni immagine app ESP inizia con il magic byte 0xE9
        print("ERRORE: il file non sembra un'immagine app ESP32 (magic 0xE9 mancante). Usa il .ino.bin, non il merged.", file=sys.stderr)
        return 1

    ota = build_ota(app, a.manuf_id, a.image_type, a.version, "AwningToF OTA")
    name = f"{a.manuf_id:04X}-{a.image_type:04X}-{a.version:08X}-awning_tof.zigbee"
    os.makedirs(a.out_dir, exist_ok=True)
    path = os.path.join(a.out_dir, name)
    with open(path, "wb") as f:
        f.write(ota)

    entry = {
        "fileName": name,
        "fileVersion": a.version,
        "fileSize": len(ota),
        "url": a.base_url.rstrip("/") + "/" + name,
        "imageType": a.image_type,
        "manufacturerCode": a.manuf_id,
        "sha512": hashlib.sha512(ota).hexdigest(),
        "modelId": a.model_id,
    }
    index_path = os.path.join(a.out_dir, "index.json")
    index = []
    if os.path.exists(index_path):
        index = [e for e in json.load(open(index_path)) if e.get("fileVersion") != a.version]
    index.append(entry)
    index.sort(key=lambda e: e["fileVersion"])
    with open(index_path, "w") as f:
        json.dump(index, f, indent=2)
        f.write("\n")

    print(f"OTA:   {path} ({len(ota)} byte, firmware {len(app)} byte)")
    print(f"Indice: {index_path} ({len(index)} voci, ultima versione 0x{a.version:08X})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
