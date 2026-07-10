# FR3 Articulation Introspection

This gate inspects the loaded FR3 USD/stage to identify articulation, joint,
link, and frame candidates. It is diagnostic only.

## Scope

- Dry-run emits the expected report schema from
  `configs/robots/fr3_real_articulation.yaml`.
- Runtime mode loads the FR3 USD and traverses stage prims.
- The report records articulation root, joint names, link names, EE frame
  candidates, gripper frame candidates, finger link candidates, visual prims,
  and collision prims when available.

## Non-Scope

- No controller is attached.
- No joint command is sent.
- No PressButton action is executed.
- No benchmark result is produced.

The output is a bridge between load-only visual smoke and later controller
planning. If the articulation root cannot be identified, the report should say
so explicitly with a warning rather than inventing a controller-ready state.
