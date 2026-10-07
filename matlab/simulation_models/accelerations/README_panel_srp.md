# Prepared panel SRP

`ComputePanelSrpResponse` evaluates body force divided by pressure, optional
direction partials, and optional torque divided by pressure about the body
origin. Batch columns are independent. Areas and centres are SI (`m^2`, `m`);
responses are `m^2`, direction partials are per supplied direction component,
and torque responses are `m^3`. Pressure, mass, attitude and external eclipse
remain outside this spacecraft geometry/optics response.

Prepare shadow samples with `BuildPanelSrpShadowData`; add triangle data from
`BuildTriangleRayData` under `strShadowData.strRayData`. Keep all shadow geometry
and its positive ray offset in one length unit. The optimized evaluator uses
conservative emitter-bundle rejection and exact intersection. Legacy sampled
payloads remain supported; no preparation is repeated for the new payload.
The local LUT builder's `strTriangleEdges` layout is also supported. Panel
visibility uses projected flat bundle traversal for either layout; the generic
`TraceTriangleRay` API retains its explicit flat/BVH selector.

`BuildSrpResponseLut` retains geometry/visibility reuse, optional torque, disk
caching, and complete-grid MEX construction. `ui32BatchCount` defaults to 64.
An explicit `fcnPanelResponse` uses the three-input complete-response ABI and
owns shadowing; it cannot be combined with `fcnVisibility` or disk caching.
Provider overrides bypass response reuse, since a callback may carry mutable
state. Both paths omit duplicate seam/pole evaluations and pad the final batch.

`CodegenPanelSrpResponse` strips host metadata, validates geometry and optics,
and freezes capacities. Mesh values and the self-shadow selection remain
runtime data. Choose a leading output count: one force, two force/Jacobian,
three force/Jacobian/torque. A fixed batch capacity avoids repeated MEX transport
for LUT construction. Generated numerical evaluation disables dynamic allocation
and variable sizing. MEX runtime checks are optional; default false assumes
validated preparation. MATLAB gateway allocation remains separate.

The Jacobian freezes sampled visibility and the illuminated face set. Include
normalization of the supplied direction. No derivative of ray-hit switches or
shadow-sample transitions is claimed. Torque keeps each geometric face centre
under partial visibility; resolving the illuminated centre of pressure remains
a separate physical-model refinement.

Max-fidelity dynamics uses this response with its existing SI/LU conversion,
frame rotation, inverse-square pressure and centre-of-mass torque correction.
Force-only RHS calls omit torque and orbital diagnostic output. Rich descriptor
metadata remains in host provenance, outside propagation/codegen payloads.
When a truth LUT is supplied, its force and optional torque remain authoritative.
Both direct and LUT Jacobians retain the optional attitude-position chain rule.

The pre-merge 64-sample complete MEX force and force/Jacobian measurements reduce
438.87/447.43 us to 192.24/194.81 us on varied spacecraft directions. Force parity
is within 7.77e-16 m^2 against the previous panel law. A 16-sample table fails the
approved accuracy gate (0.974% global vector RMS, 3.83% P99 versus 64 samples),
so retain 64 samples. Against a 1,024-sample diagnostic reference, 64-sample
per-query relative RMS/P99 are 0.576%/2.20%; 256-sample values are 0.233%/0.941%.
These sampled comparisons do not establish a continuous-sphere bound or calibrated
physical optics. Preserve reference quadrature and identities with results.

## RCS-1 comparison

The 8 October comparison uses RCS-1 commit
`9b6a2d47c572396982b5fa3249972c34040ea852`, with self-shadowing explicitly enabled.
For the same 68-face spacecraft, 64 samples per face and 64 Sun directions,
visibility and force match exactly. Median complete MATLAB evaluation is
103.322 ms in RCS-1 and 53.852 ms here, a 47.88% reduction over three warm repeats.
RCS-1 rebuilds its Sun-plane tree within a direction evaluation. This implementation
reuses prepared triangle edges and rejects whole emitter bundles before tracing.
It also provides batched force, frozen-visibility Jacobians and optional torque
without storing every face response. These timings compare MATLAB evaluation;
the earlier MEX figures above are a separate measurement.
These timings predate harmonization with the local vectorized visibility and
cached-grid construction paths; they are not new performance measurements of
the combined implementation.
