# Install and update

How the package gets onto a GX device and stays installed. Unless noted otherwise, everything here was checked against the actual scripts: Venus OS `update-data.sh` and SetupHelper v9.4.

## What setup does

`setup` is a SetupHelper install/uninstall script. It sources `/data/SetupHelper/HelperResources/IncludeHelpers`.

On install:

- Adds the udev rule (`ENV{VE_SERVICE}="smartbms"`) to `/etc/udev/rules.d/serial-starter.rules` and `service smartbms smartbms-dbus` to `/etc/venus/serial-starter.conf`. Both go through `updateActiveFile`, so uninstall can restore the originals with `restoreActiveFile`.
- Copies `workerService/` to `/opt/victronenergy/service-templates/smartbms-dbus`.
- Installs the manager service (`installService`).
- Removes legacy paths: `/data/smartbms`, `/data/smartbms-venus`, and the old lines in `/data/rc.local` and `/data/rcS.local`.
- From Venus OS v2.90~3 no reboot is needed: it stops the SmartBMS ttys, clears the serial-starter cache `/data/var/lib/serial-starter`, runs `udevadm trigger --action=add` and `svc -t /service/serial-starter`, and starts the ttys again. On older versions it asks for a reboot.

Uninstall restores both files, removes the service template and the manager service, and restarts serial-starter.

## Venus OS updates

Venus OS has two rootfs partitions and image-based updates. A firmware update overwrites the rootfs completely. Only `/data` is kept. Everything `setup` changes in `/etc` and `/opt` has to be applied again after a Venus OS update. SetupHelper does that with a reinstall.

To reinstall the same Venus OS image (handy to test the reinstall-after-update path):

```bash
/opt/victronenergy/swupdate-scripts/check-updates.sh -update -force
```

## Blind install from USB stick or SD card

Source: `meta-venus/recipes-core/initscripts/files/update-data.sh` in `victronenergy/meta-victronenergy`.

At boot the script looks on `/media/*` for `venus-data.{tar.gz,tgz,zip}` and `venus-data-*.{tar.gz,tgz,zip}`. A dot as separator (`venus-data.something.tgz`) does not match. A hyphen does.

For each archive:

```sh
tar xzf "$archive" -C "$tmp" rc            # 1. only rc/ to a temp folder
sh "$tmp/rc/pre-hook.sh" || return         # 2. non-zero exit: the archive is NOT extracted
tar xzf "$archive" -C /data --exclude rc   # 3. everything else to /data
sh "$tmp/rc/post-hook.sh" success          # 4. or "extraction-failed"
```

**Pitfall:** step 1 asks for the member `rc` literally. If the paths in the archive start with `./` (`./rc/pre-hook.sh`), tar finds nothing and the pre- and post-hook are skipped, but step 3 still extracts the rest to `/data` (the exclude matches on every path component). Symptom: the files are in `/data`, but SetupHelper is never installed. The `./` prefix comes from `tar -czf archive.tgz .`.

### SetupHelper blind install

Archive layout: `SetupHelper-blind/` (a full SetupHelper), `rc/` (`pre-hook.sh`, `post-hook.sh`, `blindInstall.sh`, `SetupHelperVersion`, `rcS.localForUninstall`) and our `123SmartBMS-Venus/`. The `rc/` files are identical to `SetupHelper-blind/blindInstall/`.

- `pre-hook.sh` compares `rc/SetupHelperVersion` with `/etc/venus/installedVersion-SetupHelper`. If the same SetupHelper version is already installed, it exits non-zero and the whole archive is skipped (see [open-points.md](open-points.md)).
- `post-hook.sh` starts `blindInstall.sh` in the background. That waits until `com.victronenergy.settings` is on D-Bus, moves `/data/SetupHelper-blind` to `/data/SetupHelper` and runs `setup install auto`.
- After that, PackageManager adds packages in `/data` automatically (`AddStoredPackages`) when the folder has a `setup` and a `version` starting with `v`, the name is not in `rejectNames`, and the name contains none of the `rejectStrings` (among others `-blind`, `-test`, `-beta`, `-temp`, `-debug`, `-latest`, `-main`, `-current`, `-0` to `-9`, and spaces). `123SmartBMS-Venus` passes. Never rename the package folder with such a suffix.
- `ONE_TIME_INSTALL` in the package folder makes PackageManager install the package once, even when auto-install is off. The file is removed afterwards. It is not in the repo; it is added to `123SmartBMS-Venus/` when the blind-install archive is built.

### Building the blind-install archive

From the staging folder:

```bash
tar -czf ../venus-data-123SmartBMS.tgz --owner=root --group=root SetupHelper-blind 123SmartBMS-Venus rc
tar -tzf ../venus-data-123SmartBMS.tgz | head   # paths must start with rc/ or SetupHelper-blind/, not ./
```

Fill the package folder in the staging folder from git, so it gets the same files as a GitHub download (`export-ignore` applies):

```bash
git archive --prefix=123SmartBMS-Venus/ latest | tar -x -C staging
```

## PackageManager updates from GitHub

PackageManager downloads `https://github.com/<gitHubUser>/<package>/archive/<branch>.tar.gz`. Here that is `123electric/123SmartBMS-Venus` with ref `latest` from `gitHubInfo`. It extracts the archive to `/data/PmDownloadTemp`, takes the first folder that contains a `version` file, renames the existing `/data/123SmartBMS-Venus` out of the way and moves the new folder in its place. So the package folder is replaced as a whole.

- **Automatic downloads:** if the branch starts with `v` (a version tag), PackageManager downloads when the GitHub version differs from the stored version. For any other branch, such as `latest`, it only downloads when the GitHub version is newer.
- **Manual download:** in the GUI (Settings > Package Manager > Active packages > the package) the branch can be changed and the Download button works whenever GitHub returns a valid version, also when it is not newer. This is the way to try out a test branch on a device. Set the branch back to `latest` afterwards.

### Keeping files off the GX

SetupHelper has no exclude mechanism of its own: everything in the GitHub archive ends up on the GX. GitHub builds these archives with `git archive`, which leaves out paths that have the `export-ignore` attribute in `.gitattributes`. `AGENTS.md`, `docs/` and `.gitattributes` itself are excluded that way. Keep `LICENSE` in the archive, since the MIT license asks for the license text to be included with copies.

`export-ignore` only takes effect for the commit the ref points to, so for customers it applies once the `latest` tag is moved to a commit that has it. To check what a ref will deliver:

```bash
git archive --prefix=123SmartBMS-Venus/ latest | tar -t
```

## On the device

- SSH: enable "SSH on LAN" in Settings > General and log in as `root`. Keys in `~/.ssh/authorized_keys` survive Venus OS updates.
- Logs: `/var/log/PackageManager/current`, `/var/log/123SmartBMS-Venus/current`, `/var/log/smartbms-dbus.ttyUSB*/current`.
- Installed SetupHelper: `/data/SetupHelper` and `/etc/venus/installedVersion-SetupHelper` exist. That is a more reliable check than the PackageManager log on a fresh install (see [open-points.md](open-points.md)).
