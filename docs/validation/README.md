# Shape validation baseline

The initial independent-copy matrix was run on ROS 2 Jazzy on 2026-09-06.
The exact observed outcomes are in [replicas_baseline.json](replicas_baseline.json).
This is a performance baseline, not a declaration that every scenario passes.

| Shape | Open | Static obstacle on centreline | Corridor | Moving crossing obstacle |
| --- | --- | --- | --- | --- |
| Square | reached | timed out | reached | reached |
| Scout rectangle | reached | timed out | reached | reached |
| Upstream trapezoid | reached | timed out | collision | reached |

Eight of twelve cases reached their goals. The centreline obstacle exposes a
symmetric local-planning deadlock; all three stopped with positive clearance.
The trapezoid corridor case collided and remains a failing navigation case.
The runner correctly exits nonzero for these outcomes. No expected-result
exceptions were introduced to turn these failures green.

Reproduce with `ros2 run neupan_sim neupan_validate --output NEW_DIRECTORY`.
The output contains the exact resolved planner/simulator parameters and all
process logs. Tests validate the orchestration and geometry separately from
these scenario outcomes. Parallel wall-clock ROS scheduling can produce small
trajectory differences across machines/runs.

## Shared-world baseline

On branch `feat/shared-world-multi-robot`, ROS 2 Jazzy, 2026-09-06:

| Scenario | Members | Outcome | Minimum clearance |
| --- | --- | --- | --- |
| [Crossing](shared_crossing.json) | Square + Scout | Both reached | 0.230 m |
| [Head-on](shared_head_on.json) | Square + Scout | Both timed out | 0.300 m |
| [Mixed three](shared_mixed_three.json) | Trapezoid + Scout + square | All reached | 0.920 m |

All robots reported the expected peer count (one or two), and thousands of laser
hits on other robot bodies. Each planner receives those surfaces and velocities
through its own XYZIV point cloud. The head-on case demonstrates the remaining
symmetric local-planning deadlock; it exits nonzero and remains a failing case.
No priority arbitration or central coordination policy is added.

Reproduce each with:

```bash
ros2 run neupan_sim neupan_validate --shared --scenario crossing --output NEW_DIRECTORY
```

Replace `crossing` with `head_on` or `mixed_three`. The independent replica suite
remains the default without `--shared`. Add `--rviz` to display either mode.
A report passes only when every robot reaches its goal and reports the expected
number of peers. Frame/world configuration errors and child exits fail the run.

## Complete interactive launch

`ros2 launch neupan_sim validate_multi_robot.launch.py` directly starts one fleet
simulator, three planner nodes, the readiness barrier and RViz. The default
`multi_robot.yaml` adds two static circles, one segment and one moving circle to
the three-shape world. No report output argument is required.

[The launch integration run](multi_robot_launch.json) reached all three goals:
trapezoid minimum clearance 0.276 m, Scout 0.271 m, square 0.500 m. Every robot
reported two peers and nonzero peer sensor hits. A separate default-launch smoke
check verified RViz with OpenGL 4.6 and clean shutdown of all processes on Ctrl-C.
These are observed outcomes, not a guarantee for every scheduling or scene.
