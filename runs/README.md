# runs/

Raw logs, world manifests, maps, result tables and figures. Every script in `analysis/` reads from this folder and writes its results back here.

## File types

| File | Written by | Content |
|---|---|---|
| `world_<seed>.csv` | `isaac/generate_world.py` | World manifest, the ground truth for every metric. Rows: `box`, `cyl`, `door,name,x,y,width,axis,yaw`, `person`, `path,person_i,x,y,seq,speed,pause_s` and `xdoor` (the door of the cross-room walker). The 7th `path` column (`pause_s`) and the `xdoor` row exist from generator v6 on. |
| `run_<seed>_<YYYYmmdd_HHMMSS>.csv` | `isaac/metrics_logger.py` | One file per Play/Stop, about 10 Hz in sim time. Line 1: seed, wall-clock start (local time) and format tag. Columns `sim_t,x,y,yaw_deg`, and format v2 adds `p0x,p0y,p1x,p1y,p2x,p2y` (the three people). |
| `odom_<seed>_<ts>.csv` | `isaac/odom_publisher.py` | Experiment 3 on: line 1 is the active noise tier, then `ox,oy,oyaw,tx,ty,tyaw` (odometry vs truth) at about 10 Hz. |
| `missions_<seed>_<ts>.csv` | `ros/mission_runner.py` | One row per goal attempt of a Nav2 tour: goal, result, wall times. |
| `world_<seed>.pgm` / `.yaml` | `ros/slam_batch.sh` (slam_toolbox) | Saved occupancy maps, 0.05 m per cell. |
| `batch_log.csv` | `isaac/batch_runner.py` | One `seed,end_reason,wall_seconds` row per batch run (layout below). |
| `nav_<ts>.log` | `ros/yolo_navigator_v2.py` | Navigator console logs of the development runs and the batches of experiments 1 and 2. |
| `*.json` | `analysis/` | Result tables (see the header of each script). In them, `contacts` counts the times a walker pushed the robot (`analysis/contacts.py`) and `dance` counts back-and-forth episodes (60 sim-s with more than 2.5 m walked but under 0.35 m net). |
| `*.png` | `analysis/` | Figures, described in `docs/FIGURES.md`. |

Positions are the robot's lidar origin, which sits at the chassis centre, and the people's root positions, so every robot to person distance in the analysis is centre to centre.

## Worlds

- `20260723001` to `20260723025`: the 25 benchmark worlds of all five experiments.
- `659927`: the single world of the walking vs frozen maps and the odometry-noise ablation (experiment 3). Its maps: `world_659927` (walking people, 3%/5% noise), `_static` (people frozen), `_n00` and `_n00b` (no noise, two runs), `_n810` (8%/10% noise).
- `983120`, `759639`, `192691`, `59259`: early exploration and development worlds.

## Cohorts

File names carry the local wall-clock start time. The scripts take the newest file per seed inside a window:

| Cohort | Files from | Files until | Used for |
|---|---|---|---|
| static, v2.9.1 | | 23/7 19:00 | experiment 1, and the baseline of experiment 2 |
| v1 baseline | 23/7 19:00 | 24/7 02:00 | experiment 1 |
| first dynamic batch | 24/7 15:00 | 24/7 22:00 | experiment 2 (watchdog forensics) |
| second dynamic batch | 24/7 22:00 | 26/7 | experiment 2 |
| cross-room walker | 26/7 14:00 | 27/7 | experiment 2 |
| mapping batch | 28/7 | 29/7 | experiment 3 |
| missions, AMCL v8 | 30/7 12:30 | 30/7 22:00 | experiment 4 |
| missions, AMCL v9 | 31/7 | 1/8 | experiment 5 |

## batch_log.csv layout

181 rows in order: v1 baseline (rows 1-10, seeds 016-025), first dynamic batch (11-35), second dynamic batch (36-60), cross-room batch (61-85), a first start of the mapping batch that was restarted (86-88), the mapping batch (89-113) and the mission runs of experiments 4 and 5 (114-181, some seeds more than once).
