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
