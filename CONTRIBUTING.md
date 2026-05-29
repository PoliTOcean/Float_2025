# Contributing — Float 2025

Team PoliTOcean @ Politecnico di Torino
Maintainers: Colabella Davide, Benevenga Filippo

## Git workflow

The project follows a **trunk-based workflow**: `master` is the single source of truth and must always build. Everything else lives on short-lived feature branches.

### Branch model

- **`master`** — always green. Protected. Direct pushes are forbidden; changes land via pull request.
- **`feature/<short-name>`** — short-lived branches off `master` for a specific change (one feature, one fix, one refactor). Aim to merge within a few days.
- **`fix/<short-name>`** — same as feature but for bug fixes (purely a label convention, behaviour is identical).
- **`hotfix/<short-name>`** — emergency fix on top of the latest release tag; same PR flow.

Tags `v*` (e.g. `v11.2.0`) mark released firmware versions and trigger the [release workflow](.github/workflows/release.yml).

### Standard cycle

```bash
# 1. Start from an up-to-date master
git checkout master
git pull --ff-only

# 2. Create a feature branch
git checkout -b feature/surface-offset-runtime

# 3. Make commits — small, focused, with descriptive messages
git add include/config.h lib/sensors/...
git commit -m "feat: runtime-tunable surface target offset"

# 4. Push and open a PR
git push -u origin feature/surface-offset-runtime
# then open PR via GitHub UI or:  gh pr create --fill

# 5. Wait for CI to pass and for at least one reviewer to approve
# 6. Merge via the GitHub UI (squash recommended)
# 7. Delete the branch (GitHub does this automatically if configured)
```

### Commit message style

Optional but recommended: prefix the subject with a short type tag for fast scanning.

| Prefix | When |
|--------|------|
| `feat:` | a new user-visible feature or command |
| `fix:` | a bug fix |
| `refactor:` | internal change with no behavioural difference |
| `docs:` | README, comments, this file |
| `test:` | adding or fixing tests |
| `chore:` | dependencies, CI, build config |

Subject in imperative ("add X", not "added X" / "adds X"), <72 chars. Body explains *why* if not obvious from the diff.

### Pull request expectations

- The branch must be **rebased on `master`** before merging (or at least up-to-date) — the PR UI will warn if not.
- All [CI checks](.github/workflows/ci.yml) must be green: `build (espA)`, `build (espB)`, `build (espA_pool)`.
- At least **one approving review** from another maintainer.
- Description should state *what* changes and *why*, plus a manual test plan if the change touches motion/PID/comms (CI does not run hardware tests).

## CI

`.github/workflows/ci.yml` runs on every push to any branch and on every pull request to `master`. It compiles all three PlatformIO environments in parallel:

- `espA` — float controller firmware
- `espB` — communication bridge firmware
- `espA_pool` — float controller with shallow-pool profile

Caching of PlatformIO core and build artifacts keeps a typical run under 2 minutes after the first warm-up.

**Hardware tests are not run by CI.** The `test/unit_hw/` and `test/integration/` suites need real ESP32 boards plus motor / TOF / Bar02. Run them locally on a bench setup — see the *Development and Testing* section in [README.md](README.md).

### Releases

Push a tag like `v11.3.0` and the [release workflow](.github/workflows/release.yml) builds all three environments and publishes a GitHub Release with the `firmware.bin` and `firmware.elf` for each one attached.

```bash
# After the change is merged to master:
git checkout master
git pull --ff-only
git tag -a v11.3.0 -m "Release v11.3.0: <one-line summary>"
git push origin v11.3.0
```

GitHub auto-generates release notes from the merged PRs since the previous tag; edit them on GitHub after the workflow finishes if you want a curated changelog.

## Branch protection (one-time setup, by repo admin)

Branch protection rules cannot be applied from CI — a repo admin enables them once on GitHub:

**Settings → Branches → Add rule → Branch name pattern: `master`**

Enable:
- Require a pull request before merging
  - Require approvals: **1**
  - Dismiss stale pull request approvals when new commits are pushed
- Require status checks to pass before merging
  - Require branches to be up to date before merging
  - Status checks: `Build espA`, `Build espB`, `Build espA_pool`
- Do not allow bypassing the above settings (recommended)
- Restrict who can push to matching branches (admin only — for emergency hotfixes)

Or via `gh` CLI:

```bash
gh api -X PUT repos/:owner/:repo/branches/master/protection \
  -F required_status_checks.strict=true \
  -F 'required_status_checks.contexts[]=Build espA' \
  -F 'required_status_checks.contexts[]=Build espB' \
  -F 'required_status_checks.contexts[]=Build espA_pool' \
  -F enforce_admins=false \
  -F required_pull_request_reviews.required_approving_review_count=1 \
  -F required_pull_request_reviews.dismiss_stale_reviews=true \
  -F restrictions=
```

(Run from a clone with `gh` authenticated; replace `:owner/:repo` if `gh` doesn't auto-detect.)

## Code style

The firmware is C++17 over Arduino/ESP-IDF. There is no clang-format config yet — match the surrounding style:

- 4-space indent, no tabs.
- Braces on the same line for `if`/`for`/function definitions.
- Comments explain *why*, not *what*; the code says what.
- New constants belong in [`include/config.h`](include/config.h) (firmware tuning) or [`include/float_common.h`](include/float_common.h) (shared protocol). Don't sprinkle magic numbers.
- New commands: append to the end of the `FloatCommand` enum (never insert in the middle — breaks compatibility with older ESPB/GUI) and add a matching `CMDxx_ACK` define, an entry in `PROTOCOL_COMMANDS` in [`lib/espb_bridge_core/src/espb_bridge_core.cpp`](lib/espb_bridge_core/src/espb_bridge_core.cpp), and a case in the ESPA main switch.

## Reporting issues

Open a GitHub issue with: what you expected, what happened, the steps to reproduce, the relevant ESP-NOW log snippet from `DebugSerial`, and the ESPA/ESPB firmware version (commit hash or tag).
