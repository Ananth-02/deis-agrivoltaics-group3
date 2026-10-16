# Contributing

Thank you for helping with the project. These rules keep five people working on one code base without breaking each other's parts.

## Workflow

1. Pull the latest `main`: `git pull origin main`.
2. Create a branch named after your area and task, for example `drone/preflight-fix`, `link/netem-runner`, `vision/mask-threshold`.
3. Commit small, focused changes with a clear message (see below).
4. Run the tests before you push: `python -m pytest -q`. All tests must pass.
5. Push your branch and open a pull request into `main`. Fill in the template.
6. One other team member reviews it. Merge after the review and a green CI run.

Do not push directly to `main`.

## Areas

- Keep changes inside your own area (`agri_loop/drone`, `agri_loop/link`, `agri_loop/vision`, `ros2_ws`). If you need a change in someone else's area, ask the owner or tag them in the pull request.
- Shared interfaces need agreement from everyone who uses them:
    - the message set in `agri_loop/link/messages.py`,
    - the `EventLog` format in `agri_loop/link/eventlog.py`,
    - the C1 frame folder format (PNG plus `index.csv` with `idx,t,file`).

## Code

- Python 3.9 compatible: no `match`, no `X | Y` type unions at runtime (use `from __future__ import annotations`).
- Follow the style of the surrounding file: module docstring explaining what the file does, short comments only where the reason is not obvious.
- New behaviour comes with a test in `tests/`. Hardware code needs a simulated path (see `agri_loop/drone/sim.py`) so it can be tested without the drone.
- Do not add packages to `requirements.txt` without telling the team. Never install cflib with its dependencies (it downgrades numpy); use `pip install cflib==0.1.27 --no-deps`.

## Data and files

- Run outputs go in `data/`, which Git ignores. Share real flight data through the group drive, not Git.
- Figures for the report or slides that must be versioned go in `docs/img/`, kept under 1 MB each.
- Do not commit papers, datasets, videos, `.venv`, or personal details (emails, phone numbers).
- Label simulated or synthetic results as such everywhere they appear.

## Commit messages

- First line: what the change does, in the imperative and under 72 characters, for example `Add pipelined stream option to the AI deck recorder`.
- Optional body: why, and what was tested.
- British English in messages and documentation (colour, behaviour, organise).

## Hardware safety

Code that commands the drone must keep the existing safety checks in `agri_loop/drone/mission.py` (battery, tilt, geofence, link loss, Ctrl+C). Changes to those checks need a review from the drone owner and a test flight at low height before they are merged.
