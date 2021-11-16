# smartbms-venus

123\SmartBMS driver for Victron VenusOS.

## Victron Cerbo/Venus

### Important

* This project requires VenusOS 2.80 or higher. Make sure you update to the latest release (candidate) to get v2.80 or higher.
* Cerbo - The USB cable can only be plugged into the two USB ports on the edge. The third USB port is a power port and cannot be used.

### Hardware Installation

Plug the [123\SmartBMS to USB cable](https://123electric.eu/products/123smartbms-to-usb) into the 123\SmartBMS End Board Ext Out and the USB into the Victron Cerbo/Venus.

### Software Installation or Upgrade via SD Card

1. Download [venus-data.tar.gz](https://123electric.eu/downloads/123smartbmstousb/venus-data.tar.gz) and put it on a usb drive or SD card (do not extract).
2. Plug this USB drive/SD card into the Cerbo/Venus and reboot the Cerbo/Venus. The software is now automatically copied.
3. Reboot again for installation.
4. If you do not see the BMS appear on the Cerbo, reboot again.
5. You can remove the USB/SD card.

### Software Installation or Upgrade via SSH

1. Ensure SSH is enabled and the root password has been set via the VenusOS display:
    1. Select Menu then Settings then General.
    2. Select "Access Level", hold the ">" RIGHT arrow button until "Superuser" appears then select it.
    3. Select "Set root password" then ENTER.
    4. Enter a secure root password.
    5. Enable the SSH on LAN option.
2. SSH into the device as the root user, e.g. `ssh root@raspberrypi2` where for example `raspberrypi2` is the hostname of the device.
3. Enter these commands to update the software:
```
    cd /data
    wget -c https://123electric.eu/downloads/123smartbmstousb/venus-data.tar.gz -O - | tar -xz
    smartbms/reinstall
```
4. Reboot.

## Raspberry Pi projects

To speed up development of custom Raspberry Pi projects, we have developed code for the Raspberry Pi. This code connects to the BMS via the 123\SmartBMS to USB cable and can upload the BMS data to Thingspeak. For more installation instructions, please see our [GitHub repository](https://github.com/123electric/smartbms-thingspeak).

## Manual

Coming soon.
