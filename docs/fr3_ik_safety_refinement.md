# FR3 IK Safety Refinement

This note records the safety refinement outcome that led to the local
differential IK diagnostic.

The previous seeded Lula target IK path could solve tiny target poses, but it
returned nonlocal joint targets for 1-5 mm Cartesian translations. The largest
arm joint deltas were about 0.758-0.774 rad, far above the intended local
motion envelope. No joint commands were sent during that diagnosis, and the
result blocked task-control progression.

The follow-up diagnostic is documented in
`docs/fr3_differential_ik_diagnostic.md`. It replaces global target IK for tiny
runtime motions with an FK finite-difference translation Jacobian and damped
least-squares local solve. The local path produced bounded tiny deltas, passed
FK validation, and executed one tiny free-space motion without NaN or safety
abort.

Current boundary:

- Lula global target IK remains diagnostic-only for this stage.
- Local differential IK is the recommended tiny EE motion method.
- No PressButton control, dataset collection, tactile force sensing, or
benchmark result is included in this refinement.
