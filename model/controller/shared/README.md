# model/controller/shared/

Shared code that both `motion_control_law_eq6.m` and `motion_control_law_eq17.m` can use.

## Current state

This directory is reserved for future shared kernel extraction. As of the new-main skeleton commit, both controllers retain their own internal EKF + Q/R + F_e + control-law code (the maximum-decoupling design per `architecture: 議題 1 (ii)`).

## Planned content (TODO)

- `ekf_update_7state.m` — pure 7-state EKF math kernel (`predict` + `update`). To be extracted once both controllers are re-verified to produce identical EKF behaviour at the kernel layer.

## Why placeholder, not extracted yet

The kernel extraction is deferred because the two source controllers (sigma `_7state.m` and eq17 `_eq17_7state.m`) have subtly different internal orchestration (sequential vs joint update, slot gating, persistent state layout). Extracting before the controllers are running side-by-side in MATLAB risks introducing silent bugs that span both controllers at once.
