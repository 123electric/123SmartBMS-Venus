# Architecture

## Hardware path

The 123\SmartBMS to USB cable converts the UART of the 123\SmartBMS to a virtual COM port with an FT232 chip. udev reports the cable with `ID_MODEL=SmartBMSToUSB`. The serial link runs at 9600 baud.

## Processes on the GX

| Process | Started by | D-Bus service | Device instance | Log |
|---|---|---|---|---|
| `smartbms.py`, one worker per cable | Victron's `serial-starter` | `com.victronenergy.battery.ttyUSB<n>` | 288 + n | `/var/log/smartbms-dbus.ttyUSB<n>/current` |
| `smartbms_manager.py`, one per system | daemontools service `123SmartBMS-Venus` | `com.victronenergy.battery.smartBMSManager` | 287 | `/var/log/123SmartBMS-Venus/current` |

`velib_python` is not shipped with the package. Both scripts load the system copy from `/opt/victronenergy/dbus-systemcalc-py/ext/velib_python`.

### How a worker gets started

1. `setup` adds a udev rule to `/etc/udev/rules.d/serial-starter.rules` that sets `ENV{VE_SERVICE}="smartbms"` for the cable, and adds `service smartbms smartbms-dbus` to `/etc/venus/serial-starter.conf`.
2. When the cable is plugged in, serial-starter reads `VE_SERVICE=smartbms` and creates a service for that tty from the template `/opt/victronenergy/service-templates/smartbms-dbus`. That template is a copy of `workerService/`, with the placeholder `TTY` replaced by the tty name.
3. The service runs `start-smartbms.sh`, which sources `/opt/victronenergy/serial-starter/run-service.sh` and starts `/data/123SmartBMS-Venus/smartbms.py -d /dev/<tty>`.

### smartbms.py (worker)

- `SmartBMSSerial` polls the serial port and parses the BMS packets.
- From BMS firmware v3.3.6 the BMS also sends key-value pairs, one per second (SoH, charge cycles, total charged and discharged kWh, firmware version). `KeyValuePairGuard` tracks each value and when it was last received.
- `SmartBMSToDbus` publishes the data with `VeDbusService`. Product name `123SmartBMS`, product id `0xB050`. A one-second GLib timer pushes the updates. After startup the script waits 8 seconds before the first update, so the filters have data.

### smartbms_manager.py (manager)

- `SmartBMSManagerDbus` uses `DbusMonitor` to watch all `com.victronenergy.battery` services. A service counts as a SmartBMS when `/ProductName` is `123SmartBMS`, `/Connected` is 1 and `/UpdateTimestamp` exists. A BMS that has not been seen for 60 seconds is dropped.
- A BMS whose custom name starts with `*` is ignored. Of the remaining BMSes, only those with the highest cell count are managed.
- The manager combines the managed BMSes (minimum and maximum cell voltage and temperature, SoC, SoH, current, power and more) and calculates the charge and discharge limits (CVL, CCL, DCL) in `_calculate_current_limits`. Its timer runs every 200 ms.

## Repo layout

| File or folder | Role |
|---|---|
| `setup` | SetupHelper install/uninstall script (bash). See [install-and-update.md](install-and-update.md). |
| `start-smartbms.sh` | Worker start script, called by serial-starter through `run-service.sh`. |
| `workerService/` | Service template for serial-starter (placeholder `TTY`), logs to `/var/log/smartbms-dbus.TTY`. |
| `service/` | Manager service, starts `smartbms_manager.py`, logs to `/var/log/123SmartBMS-Venus`. |
| `smartbms.py` | Worker: serial polling, parsing and D-Bus publication. |
| `smartbms_manager.py` | Manager: aggregation over all SmartBMSes and the charge and discharge limits. |
| `version` | Package version, must start with `v`. |
| `firstCompatibleVersion` | Oldest supported Venus OS version (`v2.8~10`). |
| `gitHubInfo` | `123electric:latest`, the GitHub user and ref that PackageManager downloads from. |
| `changelog.txt`, `README.md`, `LICENSE` | Public files, also shipped to the GX. |
| `CLAUDE.md`, `docs/`, `.gitattributes` | Developer files, kept out of the GitHub archive with `export-ignore`. |
