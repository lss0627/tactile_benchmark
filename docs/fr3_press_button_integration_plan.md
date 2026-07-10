# FR3 To PressButton Integration Plan

This document plans how real FR3 articulation will eventually replace the
current PressButton `ee_placeholder` path. It is not runtime control.

## Current Pipeline

PressButton currently has a pusher runtime path, an EE placeholder path,
runtime-smoke HDF5 datasets, replay/offline evaluation, and no-fake-force
validation. Success is still displacement-based, and force/wrench tactile
signals remain unavailable unless a later force backend reports otherwise.

## Migration Path

1. Keep `pusher` as the regression baseline.
2. Keep `ee_placeholder` as the kinematic transition path.
3. Load real FR3 USD visually.
4. Introspect articulation, joint, link, and frame candidates.
5. Bind a controller contract to verified frames.
6. Port the scripted PressButton policy to the controller.
7. Only then collect runtime-smoke FR3 datasets.

## Required Frames And Links

- Base frame: configured as `fr3_link0`.
- EE frame: candidate from config/introspection, currently `fr3_hand_tcp`.
- Gripper frame: candidate from config/introspection, currently `fr3_gripper`.
- Finger links: must be discovered or confirmed before gripper contact logic.
- Tactile frames: planned, not connected.

## Controller Requirements

- Keep the 7D action schema unchanged.
- Enforce xyz/rotation/gripper safety limits.
- Verify the EE transform and reachable workspace.
- Do not send joint commands until the controller gate is explicitly opened.

## PressButton Reachability

The next planning step should compare the button pose against the FR3 base,
arm reach, and EE frame. A scripted policy should fail loudly if the target is
unreachable rather than marking success through a proxy.

## Stop Conditions

- FR3 USD path missing.
- Articulation root not identifiable and no safe fallback is documented.
- EE or gripper frame unknown.
- Controller tries to send joint commands during planning.
- Any output tries to claim benchmark or paper results.

## Failure Modes

- Asset path points to a schema/thumbnail USD rather than the robot USD.
- Frame names differ from the planned config.
- Finger joints are embedded under unexpected prim names.
- Screenshot or WebRTC works while articulation APIs still fail.
- Button displacement success remains geometric and not tactile.

## Using Gate Outputs

The load-only status confirms the USD can be placed in the stage. The
introspection report supplies candidate joints and frames. The control contract
report confirms that no controller has been connected yet. The readiness report
combines all of these into a go/no-go summary for future FR3 controller smoke.
