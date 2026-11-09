# DROID 入出力契約（openpi commit c23745b）

openpi の `pi0_droid_jointpos_polaris` / `pi05_droid_jointpos_polaris` と、Isaac Lab Arena の
`Pi0DroidAdapter`（`isaaclab_arena_openpi/policy/droid_adapter.py`）を読んで確定させた内容。

## 観測（bridge → openpi）

| キー | 型・shape | 内容 |
| --- | --- | --- |
| `observation/exterior_image_1_left` | uint8 (224, 224, 3) RGB | 外部カメラ。`openpi_client.image_tools.resize_with_pad` で 224×224 |
| `observation/wrist_image_left` | uint8 (224, 224, 3) RGB | 手首カメラ。同上 |
| `observation/joint_position` | float32 (7,) | Franka 関節角 [rad]、Franka 標準のゼロ点・符号 |
| `observation/gripper_position` | float32 (1,) | 0 = 開、1 = 閉 |
| `prompt` | str | 言語指示 |

Unity 側の画像は 320×180（16:9、DROID の ZED と同じ縦横比）で取り、bridge で 224×224 にパディング付き縮小する。

## 行動（openpi → bridge）

- `actions`: float32 (H, 8)。`[0:7]` = **絶対**関節位置 [rad]、`[7]` = グリッパー目標 0〜1。
  根拠: polaris config の `action_space=DroidActionSpace.JOINT_POSITION`。DROID RLDS ローダーは
  `action_dict/joint_position`（絶対値）をそのまま行動にしている。
- 実行するのは先頭 H 個だけ（open-loop horizon）: pi0 = 10、pi0.5 = 15。Arena と同じく単純な chunk replay。
- グリッパーは bridge で 0.5 を閾値に 2 値化（`gripper.binarize`）。DROID の実データでもほぼ 2 値。

## 時間

- 15 Hz（DROID の記録周期）。1 step = 1/15 s の sim 時間。

## ロボットの初期姿勢

`[0, -π/5, 0, -4π/5, 0, 3π/5, 0]`（DROID の reset 姿勢）。グリッパーは開。

## 未確認（実機データとの照合が必要）

- Robotiq の取り付け角度（現在は menagerie の Franka hand と同じ attachment フレーム）
- 外部カメラ・手首カメラの位置と画角（`bridge/configs/default.yaml` の `scene.cameras` で変更可能）
