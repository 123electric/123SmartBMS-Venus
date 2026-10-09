# 123SmartBMS-Venus

Driver package that publishes 123\SmartBMS data on D-Bus on a Victron GX device (Venus OS). It is installed and updated through kwindrem's SetupHelper (PackageManager). `README.md` is for customers and dealers. This file and `docs/` are for developers and coding agents such as Claude Code, and are kept off the GX with `export-ignore` in `.gitattributes`.

## Documentation

- [docs/architecture.md](docs/architecture.md): what runs on the GX, the D-Bus services and the repo layout.
- [docs/install-and-update.md](docs/install-and-update.md): what `setup` does, blind install from USB/SD, PackageManager downloads from GitHub, building the install archive, keeping files off the GX, useful commands on the device.
- [docs/open-points.md](docs/open-points.md): observations that still need a decision or a test on a device.

Read the relevant file before changing `setup`, the service files or the release process. When you learn something about Venus OS, SetupHelper or this package that is not written down yet, add it to `docs/`, so it survives between sessions and machines.

## Branches and releases

- Customers get whatever the `latest` tag points to. `gitHubInfo` is `123electric:latest`, so PackageManager downloads that ref.
- `main` and `v1` were made identical on 2026-10-08 (v1.15, the commit `latest` points to). Keep `v1` as a separate branch.
- `archive/main-2024-08` holds an unreleased prototype (Vfull, watchdogs, per-cell data for GuiMods) that made systems crash at random. It is kept for reference only. Do not build on it.
- The version number is in three places: `version`, and the `'version'` entry of `_info` in both `smartbms.py` and `smartbms_manager.py`. Update all of them and add an entry to `changelog.txt`. SetupHelper requires `version` to start with `v`.

## Working rules

- Do not commit, push or move tags unless the maintainer asks for it.
- Coding agents have no access to a GX device. Report changes as not tested on a device until the maintainer confirms a test on real hardware.
- Feature pull requests from outside contributors are not merged. If a feature is wanted, it is built in-house so it can be tested on GX hardware first. Per-cell voltages and temperatures on D-Bus are on hold until Victron shows per-cell data in the GX GUI.
- Keep the indentation style of each file. `setup` uses tabs.
- A file that must not end up on the GX needs an `export-ignore` line in `.gitattributes`. See [docs/install-and-update.md](docs/install-and-update.md).
- README, changelog and release notes are read by customers and dealers. Write them in English and follow the style rules below. The same rules apply to `docs/`.
  - No em dashes. Use a comma, a hyphen, a colon or parentheses, or split the sentence.
  - No constructions like "it's not just X, it's Y".
  - No fixed rhythm of lists with exactly three items.
  - Avoid words like "crucial", "seamless", "robust", "dive into", "game-changer" and "in today's landscape".
  - No emoji.
