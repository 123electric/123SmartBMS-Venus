# Open points

Observations that still need a decision or a test on a device. Remove an item once it is resolved, and move what was learned to the relevant doc.

## Updates do not arrive through the combined archive when SetupHelper is installed

Our package sits in the same archive as SetupHelper's `rc/` hooks. On a system that already has the same SetupHelper version, `pre-hook.sh` exits non-zero and the whole archive is skipped, so a new 123SmartBMS-Venus version does not arrive this way. Not decided yet how to solve this.

PackageManager can also take over separate package archives (`*.tar.gz`) from USB/SD, but `transferPackage` derives the package name with `os.path.basename(path).split('-', 1)[0]`. For `123SmartBMS-Venus-...tar.gz` that gives `123SmartBMS`, so that route does not seem usable without a change, because of the hyphen in our package name.

## Logging of the blind-install hooks

In `pre-hook.sh` and `post-hook.sh` (upstream SetupHelper), `if ! [ "$logDir" ]` is always false and `mkdir -P` is not a valid option. If `/var/log/PackageManager` does not exist yet (fresh install), the hook lines do not show up in the log, even when the hooks run. Check for `/data/SetupHelper` and `/etc/venus/installedVersion-SetupHelper` instead.

## Changelog is missing v1.15

`version` is `v1.15`, but `changelog.txt` starts at `v1.14`.

## README install instructions

The install instructions in `README.md` point to `venus-data.tar.gz` on 123electric.eu and mention two reboots. With the post-hook of SetupHelper v9.x and the no-reboot route in `setup` (Venus OS v2.90~3 and later), one reboot may be enough now. Verify on a device before changing the README.

## Relation between main and v1

`main` and `v1` are identical since 2026-10-08. Not decided yet whether new work goes to `main` first and is then brought to `v1` before a release, or the other way around.
