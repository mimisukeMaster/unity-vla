# Unity ↔ bridge プロトコル（version 1）

TCP（既定 127.0.0.1:5555）、同期の要求→応答。実装: `bridge/unity_vla/protocol.py`、`unity/.../Scripts/Protocol.cs`。

```
uint32 LE  frame_len      以降のバイト数
uint32 LE  header_len
bytes      header（UTF-8 JSON）
bytes      blob 領域（header.blobs に name/offset/length/dtype/shape）
```

画像は生 RGB24、行 0 = 画像の上端。応答は必ず `ok: true|false`（false のとき `error`）。

| cmd | 要求 | 応答 |
| --- | --- | --- |
| `hello` | `protocol` | `protocol`, `unity_version`, `joint_names`, `joint_lower/upper`, `phys_sign`, `cameras` |
| `reset` | `seed`, `scene`, `control_dt`, `physics_dt`, `home_qpos?` | — |
| `step` | `action`（8 値: 関節位置 rad ×7 + グリッパー 0〜1） | `sim_time` |
| `observe` | — | `joint_position`(7), `gripper_position`, `success`, `sim_time`, `task`, blobs `exterior`, `wrist` |
| `set_joints` | `qpos`(7), `gripper` | —（瞬間移動。FK 確認用） |
| `link_poses` | — | `links`: 各リンクの `pos`/`rot`（ロボット基準、Unity 座標） |
| `close` | — | — |

`reset` はロボット・物体・カメラを作り直し、腕を初期姿勢に固定したまま 0.5 s 物理を回して物体を落ち着かせる。
`scene` の書式は `bridge/configs/default.yaml` の `task.scene` を参照。
