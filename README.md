# smartbms-venus

123\SmartBMS driver for Victron VenusOS.

## Victron Cerbo/Venus

[Download Cerbo/Venus installer file](https://123electric.eu/downloads/123smartbmstousb/venus-data.tar.gz)

Installation is simple: just plug the [123\SmartBMS to USB cable](https://123electric.eu/products/123smartbms-to-usb) into the 123\SmartBMS End Board Ext Out and the USB into the Victron Cerbo/Venus. Download venus-data.tar.gz and put it on a usb drive or SD card (do not extract). Plug this USB drive/SD card into the Cerbo/Venus and reboot the Cerbo/Venus. The software is now automatically copied. Reboot again for installation. If you do not see the BMS appear on the Cerbo, reboot again. You can remove the USB/SD card.

## Important

This project requires VenusOS 2.80 or higher. Make sure you update to the latest release (candidate) to get v2.80 or higher.
The USB cable can only be plugged into the two USB ports on the edge. The third USB port is a power port and cannot be used.

## Raspberry Pi projects

To speed up development of custom Raspberry Pi projects, we have developed code for the Raspberry Pi. This code connects to the BMS via the 123\SmartBMS to USB cable and can upload the BMS data to Thingspeak. For more installation instructions, please see our [GitHub repository](https://github.com/123electric/smartbms-thingspeak).

## Manual

Coming soon.
