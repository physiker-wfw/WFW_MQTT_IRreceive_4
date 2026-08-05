# OTA Updater

This document describes how to run the OTA updater test script and what the expected output looks like.

## Usage

```powershell
py test\ota_updater.py -l 192.168.178.26 -t /sensor/ -i ircontrol .pio\build\nodemcuv2\firmware.bin

-l : MQTT broker address
-t : MQTT topic prefix
-i : device identifier
final argument : path to firmware binary

## Example Output

# Run 1: Firmware Already Up to Date

Connecting to mqtt broker 192.168.178.26 on port 1883
Connected with result code 0
Waiting for device to come online...
Waiting for device info...
Device firmware already up to date with md5 checksum: 5bff58eab5b4f56b0c354b2076d2fdbe

# Run 2: Firmware Upload

Connecting to mqtt broker 192.168.178.26 on port 1883
Connected with result code 0
Waiting for device to come online...
Waiting for device info...
Publishing new firmware with checksum 5bff58eab5b4f56b0c354b2076d2fdbe
Checksum accepted
[++++++++++++++++++++++++++++++] 475968
[++++++++++++++++++++++++++++++] 475968
Firmware uploaded successfully. Waiting for device to come back online.
Device back online. Update Successful!


# Notes
The script waits for the device to come online and retrieve its current firmware info.
If the firmware checksum already matches, no upload occurs.
If the firmware is different, the script publishes the new firmware and waits for the device to reconnect after the update.