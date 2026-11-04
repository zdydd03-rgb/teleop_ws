> 2026-11-04迁入teleop_ws：本文及sources_manifest.json保留原参考项目的来源/哈希说明，不用于运行时加载旧目录。当前解析器在pico_o7/pico_hand_skeleton_viewer.py，模型/vendor/config均位于本包，入口见README.md。

# Vendored source and model notes

The terminal cannot clone GitHub or install external wheels in this sandbox. The connected web reader could retrieve raw text. This directory contains a reconstructed, source-only offline subset of BotRunner64/somehand, identifying itself as version 0.3.0, plus the manufacturer O7 v3 right URDF. It is not a full Git checkout or an installed CLI distribution.

The exact delivered source bytes, source URLs, and SHA256 hashes are recorded in `sources_manifest.json`. Git commit IDs were unavailable. The snapshot and included sources make this delivery reproducible without remote branch resolution.

Local changes to somehand:

- Created minimal `domain`, `application`, `infrastructure`, and `runtime` package initializers. These export only the offline classes and avoid importing omitted camera, network, controller, or bi-hand modules.
- Added `self.last_optimize_result = result` in `infrastructure/vector_solver.py`. Upstream silently returns an optimizer candidate even if optimization fails; this instrumentation permits honest success/failure accounting. No objective, solver settings, or derivative logic was changed. The pristine retrieved file is retained in `vendor/somehand/upstream_vector_solver.py.txt` for comparison.
- `run_offline.py` imports the original `application.engine.RetargetingEngine` directly and passes `domain.models.HandFrame`. This preserves the single-hand API implementation without exposing a misleading full `somehand.api` or CLI.

The original O7 base/common/right YAMLs are preserved for inspection, but cannot run because their external MJCF asset is missing. `configs/o7_right_vectors.yaml` is a local derived configuration, not the upstream preset. It uses only manufacturer URDF body origins: ten direction constraints and the upstream point-derived thumb frame. It omits all unverified tip sites and fingertip distance constraints.

`model_tools.py` is local code. It converts manufacturer origin/axis/inertia/mimic data to MuJoCo and intersects driving-joint bounds with passive mimic limits. No STL geometry, collision model, actuator device backend, or hypothetical fingertip offsets are introduced. The display shows joint origins and existing inertial centers. This is a kinematic validation model, not a physics or collision validation.

`vendor/pico_hand_skeleton_viewer.py` and its original test are byte copies from Stage 1.5. The position parser, ZIP iterator, and geometric angle function are reused unchanged. Topology uncertainty wording in that preserved historical file is superseded by the user's explicit finger-index confirmation for this task. The new player follows its Matplotlib widget and validity-hiding approach.

Upstream licenses remain in `vendor/somehand/LICENSE` and `vendor/linkerhand-urdf/LICENSE` (Apache 2.0). No original material or `/home/ai/teleop_ws` file was modified.
