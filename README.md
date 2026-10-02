# Indoor navigation among walking people: a measured pipeline in Isaac Sim

This repository contains a simulation benchmark I built to answer one question in stages: what does it take for a wheeled robot to move with purpose through cluttered rooms while people walk around it?

Everything runs in NVIDIA Isaac Sim 6.0 with ROS 2 Jazzy on WSL2. The robot is a 0.42 m differential-drive platform with a 360-ray lidar and a camera. Worlds are generated procedurally from a seed: four rooms, doors of controlled width, random furniture, and three people, who stand still or walk scripted routes. Every claim below is checked against simulator ground truth, not against the robot's own estimates. The full pipeline ran on a single RTX 2080 Ti, below the simulator's minimum spec, at real-time factors between 0.35 and 0.55.

Author: Dimitrios Gkiokas (dimitris.gkiokas@outlook.com)

Six-page technical report: [`report_gkiokas_2026.pdf`](docs/report_gkiokas_2026.pdf) (also attached to [Release v1.0](../../releases/tag/v1.0)) · All figures with captions: [FIGURES.md](docs/FIGURES.md)

<p align="center"><img src="runs/still_robot_person.png" width="420" alt="The robot and a walker in one of the generated worlds, path-traced in Isaac Sim"></p>

<p align="center"><em>The robot and one of the walkers, path-traced in Isaac Sim. The people are unanimated meshes moved along scripted routes. The camera sees the character mesh, the lidar a 0.25 m cylinder that moves with each person.</em></p>

<p align="center"><img src="runs/still_world_overview.png" width="560" alt="One generated world seen from above, path-traced"></p>

<p align="center"><em>One world from above, paused mid-run: four rooms, doors in the partition walls, seeded furniture, three people on their routes. The robot is mid-crossing in the central doorway.</em></p>

## Demo

https://github.com/user-attachments/assets/2eda9605-6167-463c-b3ef-dac336a959df

The reactive navigator coming through a doorway, recorded live in the simulator. The footage plays at 1.5× to offset the below-real-time simulation rate on this GPU. The clip also lives in the repository as [`docs/demo.mp4`](docs/demo.mp4).

## Results

Five experiments, each building on the previous one. All plots and numbers are produced by the scripts in `analysis/` from the logs in `runs/`. The four figures below are the headlines. The complete set, with captions, is in [FIGURES.md](docs/FIGURES.md).

### 1. Reactive navigation vs door width

25 seeded worlds, one run of 20 wall-clock minutes each (about 11 simulated minutes). A follow-the-gap navigator with layered recovery behaviours (v2.9.1) explores on its own, and the analysis counts every door it approaches and crosses. It crossed 12 of the 14 approached doors between 0.80 and 0.95 m wide, 10 of 17 between 0.66 and 0.80 m, and 2 of 7 below 0.66 m, the nominal minimum (0.42 m body plus 0.12 m inflation on each side). Only 38 of the 100 doors were approached at all, so the counts are small and the curve shows a trend, not a sharp threshold. In the same worlds the v1 baseline, the same follow-the-gap core without the recovery layers, reached a terminal deadlock in all 25 runs (median onset 24 s), and v2.9.1 in none.

![Door traversal success vs width](runs/success_vs_width_batch1.png)

### 2. The cost of walking people

The same 25 worlds under three conditions: the three people standing still, 1-3 of them walking routes inside their rooms, and the same with one person per world walking through a door between rooms. All 75 runs completed with no terminal deadlock, and the median exploration speed did not change (13.8, 13.8 and 13.9 m/min). The cross-room walker doubled the robot's encounters at the contested door (robot and walker both within 1.5 m of it, 26 to 54) and the robot kept using the contested doors (17 crossings in both conditions).

The walkers follow their routes without yielding, and they do walk into the robot. The robot is about 0.4 m wide and each person's collision cylinder has a 0.25 m radius, so their shapes meet at roughly 0.40-0.45 m centre to centre, and most walking runs had a closest pass below 0.45 m. A hard contact shows up in the log as the robot moving faster than 1 m/s (its top speed is 0.4 m/s) with a person within 0.5 m. Across all walking batches, the robot moved away from the person in 39 of the 41 such events, so it was being pushed. Hard pushes happened in 7 of the 25 same-room runs and 12 of the 25 cross-room runs, mostly from the side or from behind, and never with the people standing still. These counts are a lower bound, since lighter contacts do not move the robot that fast. The closest pass, 0.27 m centre to centre, was one of them: the cross-room walker came up behind the robot at a 0.67 m door and pushed it 3 m. The navigator's person braking only looks at the middle third of the forward camera image, so this walker never registered.

![Static vs dynamic, 25 paired worlds](runs/phase2_final_dynamic_vs_static.png)

### 3. Mapping while people walk

The robot maps each of the 25 worlds with slam_toolbox while odometry noise (3% linear, 5% angular, measured drift up to 1.57 m) and walking people are active. 25 of 25 maps were saved autonomously in one unattended session. Time spent within 1.2 m of a person is barely correlated with map coverage (r = -0.13), and in a single-world test the walking people left almost no trace in the map, while the same people frozen in place were mapped as obstacles. Coverage, not accuracy, appears to limit the maps: 20 wall-clock minutes of reactive exploration (about 10 simulated minutes) covers a median 63% of wall surface (range 47-92%).

![Twenty-five maps from one unattended session](runs/slam_batch_montage.png)

### 4. Goal-directed missions on self-built maps

Nav2 (AMCL + NavFn + DWB) drives four-goal tours in every world, using only the map from experiment 3, with baseline AMCL settings (v8, `ros/nav2_params_v8.yaml`). An offline reachability check at the planner's radius says 85 of 100 goals are feasible on those maps. The robot truly reaches 11: 7 of the 60 cross-room goals the check calls feasible, plus 4 returns home, three of them in worlds it never left. Ground truth shows what went wrong: in 4 worlds Nav2 reported 6 arrivals that never happened, most likely because AMCL lost track, and in 4 the robot stayed wedged while its goals timed out. In 4 more worlds the map seals the spawn room at the planner's radius (in world 001 the true geometry does too), and the planner refused every cross-room goal. In one world Nav2 reported the first two goals reached after 0.1 s each, while the robot was still at the spawn point, 7.6 and 6.8 m away. The ground-truth check is what catches this class of silent failure.

### 5. The paired AMCL experiment

The same 25 tours again with AMCL tuned for sparse maps (v9, `ros/nav2_params.yaml`: 500-4000 particles instead of 300-1500, a wider motion model, more tolerance for beams the map does not explain). In the tours that could be checked, false arrivals go from 6 to 0. The one tour that could not be checked (world 007) reported all four goals reached within 4.3 s, which cannot be real, so the tuning did not remove false arrivals entirely. Real arrivals barely move, from 11 to 12. The failures change form: instead of reporting arrivals that did not happen, the robot now stays wedged until the goal times out (wedged worlds 4 to 8). Tuning removed the false arrivals in the checked tours but did not raise the number of real ones. My hypothesis is that the maps set this ceiling: 20 minutes of wandering left parts of each world unmapped, although map coverage alone correlates only weakly with real arrivals (r = 0.26 and 0.36). The 4 sealed worlds refused every cross-room goal in both configurations, as the offline check predicted.

![Paired mission results, baseline vs tuned AMCL](runs/phase3_localization_tax.png)

## Main conclusion

The project has two main results. The first is the navigator. The same follow-the-gap core, with recovery layers added one observed failure at a time, went from a terminal deadlock in all 25 worlds (v1, median onset 24 s) to none (v2.9.1). In the same worlds it travelled 3.7 km instead of 245 m and crossed doors 36 times instead of 3, once through a 0.57 m door below its nominal minimum width. It kept that behaviour among walking people (75 runs without a terminal deadlock, median speed unchanged at about 13.8 m/min) and explored all 25 worlds of the mapping batch unattended. Its two limits are measured as well: its person braking only looks forward, so walkers that do not yield can push it, and as an explorer it covers a median 63% of the walls in 20 minutes.

The second result is the way all of this was measured. Checking every result against the simulator's ground truth changed the conclusion at each stage. It caught 6 arrivals that the mission stack reported but never made, 16 healthy runs killed by the first deadlock watchdog, a mapping run that silently used zero odometry noise, and two flaws in my own scenario: walkers that push the robot, and a near-contact threshold set below the distance at which the two bodies touch. None of these appeared in the stack's self-reported results or in the batch runner's verdicts.

## What is in the repository

- `isaac/`: the simulator side, run inside Isaac Sim.
  - `launch_isaac.bat` and `bootstrap.py` start Isaac, open `robot.usda`, start the sensor stack and press play.
  - `robot_model/`: the robot's geometry, materials and physics layers, referenced by `robot.usda`. The walkers are NVIDIA's stock character assets, loaded from the Isaac Sim content server.
  - `generate_world.py`: seeded world generator. Writes a manifest CSV per world (walls, doors with widths, furniture, patrol routes) that doubles as ground truth for every later measurement.
  - `scan_raycast_publisher2.py`, `camera_compressed_publisher.py`, `odom_publisher.py`, `metrics_logger.py`, `person_mover.py`: PhysX-raycast lidar to `/scan`, JPEG camera, odometry with parametric noise to `/odom` + TF, 10 Hz ground-truth logging, scripted walker motion (the people do not react to the robot). All of it re-binds its stage handles on every play, which lets the batch machinery regenerate worlds under a running session.
  - `batch_runner.py`: runs N seeded worlds unattended, with a spread-based deadlock watchdog and file-handshake signals to the WSL side.
- `ros/`: the ROS 2 side, run in WSL.
  - `yolo_navigator_v2.py`: the reactive navigator (follow-the-gap with layered recovery, YOLO person braking). `yolo_navigator.py` is the frozen v1 baseline. `camera_relay.py` turns the JPEG stream from Isaac back into images for the navigator.
  - `slam_batch.sh`, `nav_batch.sh`, `mission_runner.py`, `nav2_params.yaml` (+ `nav2_params_v8.yaml`, `slam_params.yaml`): per-world slam_toolbox or Nav2 bring-up, readiness gates, four-goal tours, DDS cleanup between worlds.
- `analysis/`: evaluation against manifests, offline reachability prediction, mission verification against ground truth and every figure. `run_all.sh` runs them all and keeps their printed output in `analysis/output/`. The notebook `analysis.ipynb` recomputes the experiment 1 and 2 numbers independently.
- `runs/`: manifests, per-run metrics CSVs, odometry logs, the 25 maps (PGM + YAML), mission logs, result JSONs, figures. [`runs/README.md`](runs/README.md) describes the file formats and which files belong to which experiment. The navigator logs were translated to English together with the code that wrote them. Only message text changed, never a number.
- `docs/`: the technical report, the figure gallery and the demo video.

## Reproducing

Requirements: Isaac Sim 6.0.x on Windows, ROS 2 Jazzy inside WSL2, Python 3.10+, an NVIDIA GPU (a 2080 Ti is enough, slowly). Nav2 and slam_toolbox from apt, ultralytics YOLOv8n in a venv for the reactive navigator only. The scripts expect Isaac Sim at `C:\isaacsim` (set in `launch_isaac.bat`) and the repository at `C:\isaac_project` (`/mnt/c/isaac_project` from WSL).

1. One-time setup. Copy `isaac/fastdds.example.xml` to `C:\isaac_project\fastdds.xml` (the UDP-only DDS profile the launcher points at). In WSL, run `mkdir -p ~/robot_project/maps && cp /mnt/c/isaac_project/ros/slam_params.yaml ~/robot_project/`, since the batch scripts read the SLAM parameters and keep maps there. To rerun the missions on the published maps instead of new ones, also copy `runs/world_202607230*.pgm` and `runs/world_202607230*.yaml` into `~/robot_project/maps/`.
2. Start Isaac with `isaac/launch_isaac.bat`. `bootstrap.py` opens the stage, starts the sensor stack, and presses play. The odometry noise is read from `C:\isaac_project\odom_noise.txt` (three numbers: linear sigma, angular sigma, yaw-rate bias), with 0.03 0.05 0 if the file is missing.
3. For a navigation batch (experiments 1 and 2) or a mapping batch (experiment 3), start the navigator in WSL: `ros/camera_relay.py` with the system Python and `ros/yolo_navigator_v2.py` (or `yolo_navigator.py` for v1) inside the ultralytics venv, both with ROS 2 sourced. For mapping, also start `ros/slam_batch.sh`. Then run `isaac/batch_runner.py` from the Script Editor with `NAV_MISSIONS = False` and `MAPPING_SAVE = True` for mapping (`False` otherwise).
4. For a mission batch (experiments 4 and 5): start `ros/nav_batch.sh` in WSL, then run `isaac/batch_runner.py` with `NAV_MISSIONS = True` and `MAPPING_SAVE = False`. Maps for the same seeds must be in `~/robot_project/maps`, either from step 3 or the published ones from step 1.
5. `bash analysis/run_all.sh` regenerates every result file and figure from `runs/` in a minute or two. It needs only Python with numpy, scipy and matplotlib.

A 25-world batch takes 5 to 10 hours at these real-time factors. The seeds in `isaac/batch_runner.py` reproduce the exact worlds used here. The published manifests come from generator v6, which adds the cross-room walker. Experiments 1 and 2 ran the same seeds, so the same geometry, with earlier settings of the generator: no walking routes in the static condition (`PEOPLE_PATHS = False` gives the same), and same-room routes only in the first walking condition, which v6 has no switch for. The people's positions in those runs are in the run CSVs.

## Limitations

- 25 worlds per condition. Enough to see the patterns above, not enough for fine-grained statistics.
- The offline feasibility check is a BFS approximation of NavFn on an inflated grid, not NavFn itself.
- Mission arrivals are verified against ground truth with a 1.5 m threshold, which absorbs the timing tolerance of the log join.
- One tour of 50 (world 007, tuned run) reported all four goals reached within 4.3 s, and no ground-truth log could be matched to it, so it is excluded as unverifiable. The exclusion is recorded in the result JSONs.
- The walkers follow scripted routes and do not react to the robot, so some contacts in experiment 2 are walkers walking into a robot whose person braking only looks forward. Hard contacts are detected from the logged robot motion (speed above 1 m/s with a person within 0.5 m, see `analysis/contacts.py`), since the simulator's collision events were not logged, so lighter contacts are not counted.
- Robot to person distances are measured centre to centre. The "near-contact" threshold of 0.35 m used in the figures is below the distance at which the robot and the 0.25 m-radius person cylinder touch (roughly 0.40-0.45 m), which is why contacts are counted separately.
- Wall-clock timestamps throughout (no `/clock`). This was a deliberate design choice and it is why the stack survives real-time factor swings, but it means message timing does not emulate a real robot's clock discipline.

## Future work

The mapping experiment points at coverage, not filtering, as the bottleneck. Frontier-based exploration in place of reactive wander would test whether better maps raise the ceiling that experiment 5 could not move with tuning. On the social side, walkers that react to the robot and a navigator that watches its rear with the 360-degree lidar would remove the contacts that experiment 2 found. Odometry from wheel encoders instead of a noise model, and a backtrack recovery layer, are smaller follow-ups.

## License

MIT.
