# OTA firmware updates (Arduino firmware, over Zigbee2MQTT)

Step-by-step guide to build a new firmware of the [Arduino sketch](../arduino/README.md) and install it on the sensor **over the air** through Zigbee2MQTT, without touching the board. It covers the files involved, what goes in the repository, what does not, and how to publish a release.

> The first flash of a board with an OTA-capable firmware always goes over USB. After that, every update can be done over Zigbee.

## How it works

1. The sketch contains the Zigbee **OTA Upgrade client** (cluster `0x0019`, endpoint 10). At boot, and then every hour, the sensor asks the coordinator whether a newer image exists.
2. Zigbee2MQTT looks the answer up in an **OTA index** (`index.json`): a list of images with version, size, SHA-512 and download URL. Here the index and the images are served straight from this GitHub repository.
3. When you press *Update* (or schedule it), Zigbee2MQTT downloads the image and sends it to the sensor in small blocks. The Arduino core writes it to the inactive app slot, validates it and only then switches boot partition and restarts.
4. Settings (NVS) and the Zigbee pairing are in their own partitions and survive the update.

Identity of the device, used to match image and sensor (all three must agree):

| Value | Where | Current |
| :--- | :--- | :--- |
| Manufacturer code | `OTA_MANUFACTURER` in the sketch, `manufacturerCode` in `index.json` | `0x1001` |
| Image type | `OTA_IMAGE_TYPE` in the sketch, `imageType` in `index.json` | `0x1011` |
| Model | `setManufacturerAndModel(..., "AwningToF")` in the sketch, `zigbeeModel` in the converter, `modelId` in `index.json` | `AwningToF` |

Do not change them between releases: the sensor would stop recognising the images.

## Files involved

| Path | What it is | In git? |
| :--- | :--- | :--- |
| `arduino/esp32_zigbee_awning/esp32_zigbee_awning.ino` | Firmware source. `FW_VERSION` is the version Zigbee2MQTT compares | Yes |
| `arduino/esp32_zigbee_awning/sketch.yaml` | Default board options (`PartitionScheme=zigbee`: two OTA slots) | Yes |
| `arduino/esp32_zigbee_awning/build/` | Compiler output: `.ino.bin`, `.merged.bin`, `.elf`, `.map`, bootloader, partitions | **No** (ignored by `arduino/.gitignore`) |
| `z2m/awning_tof.mjs` | Zigbee2MQTT external converter, declares `ota: true` | Yes |
| `z2m/ota/make_ota.py` | Wraps the `.ino.bin` into a Zigbee OTA file and updates the index | Yes |
| `z2m/ota/<manuf>-<type>-<version>-awning_tof.zigbee` | The OTA image served to the sensor | Yes (Zigbee2MQTT downloads it from GitHub) |
| `z2m/ota/index.json` | OTA index read by Zigbee2MQTT | Yes |

What you never commit: everything under `build/` (the `*.bin`, `*.elf` and `*.map` files are also ignored), and any `*.patch` or temporary file.

The OTA file is the only binary in the repository. It is about 745 KB per release, so the repository grows by that much each time. If that becomes a problem, keep only the last releases in `z2m/ota/` (delete old `.zigbee` files and their index entries) or publish the images as GitHub Release assets and point the `url` fields of the index to them.

## One-time setup

1. **Converter**: copy `z2m/awning_tof.mjs` into the Zigbee2MQTT `external_converters` folder (next to `configuration.yaml`). With Zigbee2MQTT 2.11 or newer set `enable_external_js: true` under `advanced`. Restart Zigbee2MQTT. After changing the converter or the clusters of the firmware, remove the device in Zigbee2MQTT and pair it again.
2. **OTA index**: in the Zigbee2MQTT `configuration.yaml` add the index URL and restart:

   ```yaml
   ota:
     zigbee_ota_override_index_location: https://raw.githubusercontent.com/ibrambati/esp32-zigbee-tof-awning-sensor/main/z2m/ota/index.json
   ```

   With Home Assistant OS there is no direct access to the Zigbee2MQTT data folder, so the URL form is the practical one. Use `.../feat/<branch>/z2m/ota/index.json` while testing a branch. Only use sources you trust: a custom index bypasses several of the regular OTA checks, and a wrong image can brick a device.
3. **Tools** on the PC that builds releases: Arduino IDE 2 or `arduino-cli` with the `esp32:esp32` core, and Python 3 (only the standard library is used).

## Release procedure

All commands are run from the repository root. Forward slashes work in PowerShell, cmd and bash.

### 1. Raise the version

In the sketch, increase `FW_VERSION` (format `0xMMmmpppp`, for example `0x00010001` = 0.1.1). Zigbee2MQTT only offers an image whose version is **higher** than the installed one, and cannot reinstall the same version. If you want to recognise the update, also change a serial message.

### 2. Compile and export the binary

- Arduino IDE 2: *Sketch → Export Compiled Binary*.
- `arduino-cli`: `arduino-cli compile --export-binaries arduino/esp32_zigbee_awning`.

Both write into `arduino/esp32_zigbee_awning/build/esp32.esp32.esp32h2/`. The file you need is **`esp32_zigbee_awning.ino.bin`**. Do not use `*.merged.bin`, `*_flashed.bin`, the bootloader or the partitions file: those are for USB flashing.

### 3. Create the OTA image and the index

```bash
python z2m/ota/make_ota.py \
    --bin arduino/esp32_zigbee_awning/build/esp32.esp32.esp32h2/esp32_zigbee_awning.ino.bin \
    --version 0x00010001
```

(single line on Windows). `--version` must be the same value as `FW_VERSION`. The script refuses a file that is not an ESP application image, writes the `.zigbee` file into `z2m/ota/` and adds or replaces the entry for that version in `z2m/ota/index.json`.

By default the index URLs point to the `main` branch of this repository. To test from another branch add `--base-url https://raw.githubusercontent.com/ibrambati/esp32-zigbee-tof-awning-sensor/<branch>/z2m/ota`, then regenerate with the default URL before merging (running the script again with the same version replaces the entry).

### 4. Commit and push

```bash
git add arduino/esp32_zigbee_awning/esp32_zigbee_awning.ino z2m/ota
git status          # must NOT list anything under build/
git commit -m "ota: release 0.1.1"
git push
```

For a release the files must end up on `main` (merge the pull request), because that is where the default index URL points. Check in a browser that the `index.json` URL opens before going on.

### 5. Install on the sensor

1. Open the device page in Zigbee2MQTT, tab **OTA**, press *Check for updates*. `update.latest_version` should show the new version.
2. Press *Update*, or *Schedule* if the sensor is asleep or busy: the update then starts at its next OTA check (a minute after every boot, then every hour).
3. Keep the board powered and, for the first tests, connected over USB with the serial monitor at 115200. You should see `OTA: trasferimento avviato`, later `OTA: trasferimento terminato`, then the reboot and `Firmware 0x00010001`.

Expect a long transfer: about 745 KB in blocks of 50 to 64 bytes with a 250 ms pause by default is in the order of an hour (an estimate, not a measurement). Zigbee2MQTT can shorten the pause (`ota: image_block_response_delay`, minimum 50 ms) but a low value needs a stable network. Keep the battery well charged: some devices refuse updates when it is low.

While an update is running the sensor stays awake: the night deep sleep is suspended during the transfer and for 10 minutes after every boot (`OTA_WINDOW_MS`), and the BOOT-button factory reset is ignored. A sensor in deep sleep cannot receive anything: update it in daytime or use *Schedule*.

## Troubleshooting

| Symptom | Likely cause |
| :--- | :--- |
| No *Update* button, no OTA entity | The converter is not loaded or lacks `ota: true`; the device was paired before the OTA cluster existed: remove it and pair it again |
| *No image available* | `manufacturerCode`, `imageType` or `modelId` in the index differ from the firmware, or the index version is not higher than the installed one |
| *Check for updates* shows nothing new | Index URL not reachable or not updated yet (open it in a browser), or Zigbee2MQTT was not restarted after editing `configuration.yaml` |
| Update starts, then times out | Weak link or a very low block delay: raise `image_block_response_delay`, move the sensor closer to a router |
| `make_ota.py` says the file is not an ESP application image | You selected the `merged` or another `.bin`: use `*.ino.bin` |
| Sensor does not answer at all after an update | Reflash over USB: *Upload* from the IDE, or `arduino-cli upload` |

The Zigbee2MQTT side follows its public documentation and the zigbee-herdsman-converters types. Not every case above has been reproduced on the real board yet (see *Open points* in the [Arduino README](../arduino/README.md)).
