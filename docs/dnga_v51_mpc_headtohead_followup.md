# DNGA V5.1-dev follow-up: same-event native-MPC A/B and offset sensitivity

**Status: offline-only. No actuation validated. No public-road engaged OP test.**

## Additional observations from completed November 5 V5.1 replay

The full 39-segment V5.1 replay had 46,799/46,799 successful Lane and E2E frames, zero solver failures, and 18,535 high-confidence lane frames at camera setting +4cm / path 0cm **assumed** (not recovered from log config).

Stratifying by a **planned-curvature proxy** for straight versus curved (not independently verified roadway classification):

| Subset | Frames | Median E2E minus lane path | Fraction right | Median E2E 50ms-ish point-0 change | Median lane point-0 change |
|---|---:|---:|---:|---:|---:|
| High-confidence all | 18,535 | +10.42 cm | 76.1% | 5.27 cm | 0.65 cm |
| Straighter proxy, abs lane MPC curvature at 0 and 5 below 0.001/m | 10,021 | +9.59 cm | 77.3% | 5.25 cm | 0.61 cm |
| Curved proxy, complement | 8,514 | +11.49 cm | 74.7% | 5.30 cm | 0.70 cm |

The E2E first-point displacement is a **direct translation** of raw modelV2 first-point Y; the camera and driving offsets are constant across that prior replay, so the high 50ms-ish variability originates in the **model outputs**, not in an oscillating offset setting. The model output is still camera-relative; this observation cannot prove real-world steering oscillation.

Filtered consecutive frames were 25–120ms apart and in the same segment. This is a descriptive snapshot analysis, not a causal determination of why the model predicts that path.

## Reproducible head-to-head production replay tool

Tool: [tools/dnga/compare_v50_v51_mpc_offroad.py](../tools/dnga/compare_v50_v51_mpc_offroad.py).

It loads the **original V5.0 LateralPlanner** from the pinned local commit
\`ce841812a5b17a6bdcac9b4e9deb5235f06978b9\`, alongside the currently checked out V5.1-dev class. Uses native Acados MPC. Four independent planners per recorded model event:

- V5.0 lane
- V5.0 E2E (old aliasing behavior)
- V5.1 lane
- V5.1 independent E2E

All four consume the **same** recorded modelV2/carState/controlsState event and **the same explicit simulated configuration**. No prior-cycle logged V5.0 lateralPlan is used as the comparator.

The separate **sweep** runs V5.1 lane and E2E at camera/path configurations:
\`(camera, path-5cm), (camera,path), (camera,path+5cm), (0cm,path), (6cm,path)\`. Path and camera offsets are in the legacy UI's signed-centimeter convention.

Run only while parked, disconnected from the car, and offroad. The tool refuses to run if IsOnroad is true or IsOffroad is false. It neither imports CarController nor transmits CAN.

### Installation on OnePlus 3T

Keep the checked-out V5.1-dev code. An older Git on the phone may not create \`origin/V5.1-dev\`; retrieve the tool directly from \`FETCH_HEAD\` to avoid changing branches:

\`\`\`bash
cd /data/openpilot
git fetch origin V5.1-dev
git diff --quiet HEAD FETCH_HEAD -- selfdrive/controls/lib/lateral_planner.py selfdrive/controls/lib/dnga_path_utils.py || { echo "Planner differs; stop"; exit 1; }
git show FETCH_HEAD:tools/dnga/compare_v50_v51_mpc_offroad.py > /data/compare_v50_v51_mpc_offroad.py
python3 -m py_compile /data/compare_v50_v51_mpc_offroad.py
\`\`\`

### Baseline smoke (200 model frames; 4 planners)

\`\`\`bash
cd /data/openpilot
PYTHONPATH=/data/openpilot python3 -u /data/compare_v50_v51_mpc_offroad.py \
 --archive /data/realdata1.zip --cases baseline --camera-cm 4 --path-cm 0 \
 --max-model-frames 200 --out /data/v51_same_frame_smoke
cat /data/v51_same_frame_smoke/summary_baseline.txt
\`\`\`

For full 39-segment work remove \`--max-model-frames\`, set a NEW output directory, and keep \`--stride 1\` for stateful baseline correctness. Run separate \`--cases sweep --stride 10\` for exploratory sensitivity; the downsampling alters MPC warm starts and should not be called a full production timing replay.

The \`matched_baseline.csv\` and \`matched_sweep.csv\` files are long-form, one row per version/mode/offset per model event. Join on \`route, segment, logMonoTime\` for exact-input comparisons. Report solver_status, finite, measured deltas in \`candidate_y0\`, \`mpc_target_y0\`, \`curvature0\`, \`curvature5\` and high-confidence subsets.

**Gates:** zero exceptions/invalid solver rows, correct sign of offset-to-path shifts, and no unjustified near-field discontinuities. These checks do *not* establish correct actual vehicle lane position, steering smoothness, or safe longitudinal braking. The November 5 braking deficit and revving are still unresolved; leave actuation unchanged.
