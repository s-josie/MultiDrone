Motion-planning benchmark workspaces
======================================

All environments use bounds x,y,z = [0,50] and contain five drone start states
and five corresponding goal states.

Difficulty is designed qualitatively from free-space coverage and connectivity:
- easy: high free-space visibility/coverage and many collision-free routes
- medium: reduced visibility with clustered obstacles and several alternative routes
- hard: lower visibility/connectivity with maze-like slabs, vertical barriers, and
  constrained passages

The exact epsilon, alpha, and beta values depend on the visibility/lookout
definitions and robot model used by the evaluator, so they are intentionally not
hard-coded here. These files are intended as geometric benchmark instances that
should be scored by the user's metric implementation.

Drone i is paired with goal i.
