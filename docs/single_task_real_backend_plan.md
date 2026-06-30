# Single Task Real Backend Integration Plan

This document plans the first real-backend smoke path only. It does not connect
Isaac Sim or Lightwheel in the current repository state.

## First Task Choice

`PressButton` is the default first task for a real backend smoke.

`PressButton` is a better first smoke than `PegInsert` because it can be
visualized with a simple table, one robot, one button object, a fixed camera,
and a binary success condition. It has contact, but the scene can be debugged
without tight insertion tolerances, precise hole geometry, or multi-stage
assembly failure modes. This makes it suitable for validating launch,
rendering, camera placement, robot visibility, object placement, and a minimal
control loop.

`PegInsert` remains important, but it belongs in the later tactile-assembly
phase after visual streaming, robot articulation, contact reporting, and
tactile read paths have been proven on a simpler single-contact task.

## Minimal Goal

The next runtime stage should let a developer open the Isaac Sim WebRTC
Streaming Client and see a minimal `PressButton` scene with:

- one FR3-style robot placeholder or configured robot asset;
- one button object on a stable support surface;
- a camera view that shows the robot end effector and button;
- basic lighting;
- physics enabled only enough to make the button scene coherent;
- a planned reset position for robot and button;
- a planned step path that accepts the existing 7D action schema;
- a planned success metric based on button displacement or activation state.

This visual smoke is not a benchmark result and must not be reported as a paper
metric.

## Scene Components

- Scene: Isaac Sim native stage, table/ground plane, lighting, camera, and
  optional simple primitive button.
- Robot: planned `fr3_tactile` robot path. If a repository-local USD is not
  available, the runtime stage should use a configured local Isaac Sim/Isaac Lab
  robot asset path rather than vendoring a large asset.
- Button asset: start with Isaac Sim primitive geometry where possible. If a
  Lightwheel button asset is used, it must pass `assets/asset_manifest.csv` and
  provenance-gate checks before runtime use.
- Camera: a fixed third-person visual-smoke camera is enough for the first
  stage. Dataset camera schema remains unchanged.
- Lighting: Isaac Sim native dome or area light.
- Physics: Isaac Sim native physics only in the runtime stage; current code does
  not create a `SimulationApp`.
- Reset: planned robot home pose, button initial pose, and deterministic seed.
- Step: future adapter accepts 7D action and maps it to the robot control API.
- Success metric: planned binary button activation, logged separately from mock
  metric code until validated.

## Asset Sources

Isaac Sim primitive objects can be recorded as `isaacsim_builtin` provenance.
Lightwheel / LW-BenchHub assets are optional compatibility targets and must keep
their upstream license, URL, attribution, modification status, and
redistribution status. This project is not a Lightwheel fork.

## Non-Goals

- no 30-task expansion;
- no real Lightwheel runtime connection in this phase;
- no dataset collection from Isaac Sim;
- no training;
- no claim of real benchmark performance;
- no replacement of the stable action, observation, tactile, or dataset schema.
