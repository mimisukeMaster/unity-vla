# unity-vla

Unity 6.3 LTS 上の Franka Panda + Robotiq 2F-85 を、openpi の **pi0（DROID・関節位置版）** で閉ループ制御するシミュレーション。

```
[Unity: SimServer] <-- TCP :5555 lockstep --> [bridge (Python)] <-- WebSocket --> [openpi server (NVIDIA GPU)]
 物理・描画・成功判定                          エピソード進行・記録・評価               推論のみ（Modal / Docker）
```

- Unity は bridge が `step` を送ったときだけ物理を進める（推論が遅くても観測と行動がずれない）
- ロボットと机上シーンは実行時に `Resources/UnityVLA/robot_manifest.json` から組み立てる（URDF-Importer 不要）
- 観測・行動は openpi の DROID 形式そのまま（[docs/droid_contract.md](docs/droid_contract.md)）

## 構成

| パス | 中身 |
| --- | --- |
| `unity/` | Unity プロジェクト（`Assets/UnityVLA` 以下が本体） |
| `bridge/` | Python（uv）: Unity クライアント、policy、runner、評価 |
| `server/` | openpi サーバー: Modal（`modal_app.py`）と Docker（`serve_local.sh`） |
| `tools/` | ロボットアセット変換（mujoco_menagerie → Unity）、FK 参照値 |
| `docs/` | プロトコル、DROID 契約、ロボットモデル、マイルストーン確認手順 |

## セットアップ（Windows + WSL2）

PC の Windows 側には Unity 以外何も入れない。Python は WSL2 の中で `uv` の仮想環境だけを使う。

1. **Unity**: Unity Hub で **Unity 6.3 LTS（6000.3.x）** をインストール →「Add project from disk」で `unity/` を開く。
   初回は `Packages/` と `ProjectSettings/` が自動生成される。
2. メニュー **UnityVLA → Create Tabletop Scene** → Play。机・ロボット・物体が出れば OK。
3. **WSL2 から Windows の localhost に届くように**、Windows 側の `%UserProfile%\.wslconfig` に以下を書いて `wsl --shutdown`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
4. **bridge（WSL2 の中）**:
   ```bash
   curl -LsSf https://astral.sh/uv/install.sh | sh      # uv を WSL のユーザー領域に
   cd unity-vla/bridge && uv sync                         # bridge/.venv に隔離
   uv run unity-vla ping                                  # Unity と接続確認
   ```

## マイルストーンの確認

| | 内容 | コマンド（`bridge/` で実行） |
| --- | --- | --- |
| M1 | 順運動学・追従 | Unity Test Runner（EditMode / PlayMode）、`uv run unity-vla check-fk` |
| M2 | ロックステップ・決定性 | `uv run unity-vla run -c configs/dummy_sine.yaml` → `uv run unity-vla compare runs/.../ep000*/episode.npz runs/.../ep001*/episode.npz` |
| M0 | 推論サーバー単体 | `uv run unity-vla policy-check -c configs/pi0.yaml` |
| M3 | pi0 閉ループ | `uv run unity-vla run -c configs/pi0.yaml` |
| M4 | 成功率評価 | `uv run unity-vla eval -c configs/eval_pi0.yaml` |

詳しくは [docs/milestones.md](docs/milestones.md)。結果は `runs/<日時>_<tag>/` に動画（外部 + 手首カメラ）、`episode.npz`、`result.json`、`summary.json` として出る。

## 推論サーバー

pi0 の推論には **8 GB 超の NVIDIA GPU** が必要。手元に無い場合は Modal（無料クレジットあり）を使う:

```bash
uv tool install modal && modal setup                      # WSL2 内、ユーザー領域
modal run  server/modal_app.py::download --variant pi0    # チェックポイントを Volume へ（CPU）
modal serve server/modal_app.py                           # serve_pi0 / serve_pi05 の URL が出る
```

`bridge/configs/pi0.yaml` の `host` を `wss://<表示されたURL のホスト>`、`port: null` にする。

## ロボットアセットの再生成

```bash
git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie ../mujoco_menagerie
cd tools && uv sync && uv run python build_robot_assets.py --menagerie ../../mujoco_menagerie
```

## 出典・ライセンス

- ロボットモデル: [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)（franka_emika_panda: Apache-2.0、robotiq_2f85: BSD-2-Clause）
- policy: [openpi](https://github.com/Physical-Intelligence/openpi)（Apache-2.0）、commit `c23745b`（Isaac Lab Arena と同じ）
