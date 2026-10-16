# runs/

Raw logs, world manifests, maps, result tables and figures. Every script in `analysis/` reads from this folder and writes its results back here.

## File types

| File | Written by | Content |
|---|---|---|
| `world_<seed>.csv` | `isaac/generate_world.py` | World manifest, the ground truth for every metric. Rows: `box`, `cyl`, `door,name,x,y,width,axis,yaw`, `person`, `path,person_i,x,y,seq,speed,pause_s` and `xdoor` (the door of the cross-room walker). The 7th `path` column (`pause_s`) and the `xdoor` row exist from generator v6 on. |
| `run_<seed>_<YYYYmmdd_HHMMSS>.csv` | `isaac/metrics_logger.py` | One file per Play/Stop, about 10 Hz in sim time. Line 1: seed, wall-clock start (local time) and format tag. Columns `sim_t,x,y,yaw_deg`, and format v2 adds `p0x,p0y,p1x,p1y,p2x,p2y` (the three people). |
| `odom_<seed>_<ts>.csv` | `isaac/odom_publisher.py` | Experiment 3 on: line 1 is the active noise tier, then `ox,oy,oyaw,tx,ty,tyaw` (odometry vs truth) at about 10 Hz. |
| `missions_<seed>_<ts>.csv` | `ros/mission_runner.py` | One row per goal attempt of a Nav2 tour: goal, result, wall times. From experiment 6 on also `sim_start,sim_end` (simulated seconds from the run CSV), `config` (parameter file name), `map` (`slam` or `truth`) and, from the second pilot of the revised configuration on, `params_md5` (first 8 hex digits of the parameter file's md5). |
| `amcl_<seed>_<ts>.csv` | `ros/mission_runner.py` | Experiment 6 on: every AMCL pose estimate, `epoch,sim_t,x,y,yaw_deg,cov_xx,cov_yy,cov_aa`, in the AMCL base frame (`base_axle` for the revised configuration, 0.10 m behind the chassis centre). |
| `cmdvel_<seed>_<ts>.csv` | `ros/mission_runner.py` | Experiment 6 on: velocity commands at three stages (`controller`, `smoother`, `monitor`), sampled at 2 Hz and at every change between moving and stopped. |
| `truth_<seed>.pgm` / `.yaml` | `analysis/make_truth_map.py` | Perfect maps rendered from the manifest, same resolution (0.05 m) and frame as the SLAM maps. Cells outside the arena are unknown. |
| `world_<seed>.pgm` / `.yaml` | `ros/slam_batch.sh` (slam_toolbox) | Saved occupancy maps, 0.05 m per cell. |
| `nav2logs/nav_<seed>_*.log` | `ros/nav_batch.sh` | Nav2 console logs of the tours that show the start-up bug (README). |
| `batch_log.csv` | `isaac/batch_runner.py` | One `seed,end_reason,wall_seconds` row per batch run (layout below). |
| `nav_<ts>.log` | `ros/yolo_navigator_v2.py` | Navigator console logs of the development runs and the batches of experiments 1 and 2. |
| `*.json` | `analysis/` | Result tables (see the header of each script). In them, `contacts` counts the times a walker pushed the robot (`analysis/contacts.py`) and `dance` counts back-and-forth episodes (60 sim-s with more than 2.5 m walked but under 0.35 m net). |
| `*.png` | `analysis/` | Figures, described in `docs/FIGURES.md`. |

Positions are the robot's lidar origin, which sits at the chassis centre, and the people's root positions, so every robot to person distance in the analysis is centre to centre.

## Worlds

- `20260723001` to `20260723025`: the 25 benchmark worlds of all mission experiments.
- `20261012001` to `20261012006`: development worlds, used only for the pilots of the revised configuration (perfect maps).
- `20261015001` to `20261015010`: new worlds of the held-out check, generated after the revised configuration was frozen (perfect maps).
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
| v9, perfect maps | 12/10 | 12/10 | experiment 6 |
| pilots, development worlds | 12/10 | 13/10 | v9 baseline, then three pilots of the revised configuration (`params_md5` 1a4e33df for pilot 2, 9f696af1 for pilot 3) |
| revised, perfect maps | 13/10 | 14/10 | experiment 8 (`params_md5` 9f696af1) |
| revised and v9, SLAM maps | 14/10 | 14/10 | experiments 7 and 5b, alternating world by world (`params_md5` 9f696af1 and b7f0bb38) |
| revised and without 7a, perfect maps | 15/10 | 15/10 | the checks of README section 8 (`params_md5` 9f696af1 and a5e38846) |
| experiment 9, SLAM maps | 15/10 | 15/10 | experiment 9 (`params_md5` 13931c6d), stopped by choice after worlds 001-003. The partial tour of world 004 is not included. |

From experiment 6 on the scripts select a cohort by the `config`, `map` and `params_md5` columns as well as the date.

## batch_log.csv layout

Rows in order: v1 baseline (rows 1-10, seeds 016-025), first dynamic batch (11-35), second dynamic batch (36-60), cross-room batch (61-85), a first start of the mapping batch that was restarted (86-88), the mapping batch (89-113), the mission runs of experiments 4 and 5 (114-181, some seeds more than once), experiment 6 (182-206), the pilots on the development worlds (207-218), experiment 8 (219-243), experiments 7 and 5b (244-293, two rows per world), the checks of README section 8 (294-329, two rows per world) and experiment 9 (330-333, the last one stopped during world 004).
