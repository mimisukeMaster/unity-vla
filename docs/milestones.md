# マイルストーンの確認手順

コマンドはすべて WSL2 の `bridge/` で実行。Unity は Tabletop シーンを Play しておく。

## M1 ロボット（GPU 不要）

1. Unity: **Window → General → Test Runner**
   - EditMode: `Kinematics_MatchReference`（manifest から組んだ階層が MuJoCo の FK と 0.1 mm 以内で一致）ほか
   - PlayMode: `Arm_TracksTargets_Within5mm`、`Arm_Teleport_MatchesReference`、`Gripper_OpensAndCloses`、`Arm_Deterministic`
2. bridge から実機相当の確認: `uv run unity-vla check-fk` → `PASS` なら 50 姿勢すべてで誤差 5 mm 以内。
3. 目視: Play 中に `UnityVLA` オブジェクトの **Manual Control** をオンにしてスライダーで関節を動かす。

## M2 通信・決定性（GPU 不要）

```bash
uv run unity-vla run -c configs/dummy_hold.yaml     # 腕が垂れたりずれたりしないこと
uv run unity-vla run -c configs/dummy_sine.yaml     # 同じ seed で 3 エピソード
uv run unity-vla compare runs/<dir>/ep000_seed0/episode.npz runs/<dir>/ep001_seed0/episode.npz   # PASS
```

## M0 推論サーバー（NVIDIA GPU）

```bash
modal serve server/modal_app.py                       # 別ターミナル
uv run unity-vla policy-check -c configs/pi0.yaml     # shape (10, 8) 以上なら PASS、遅延も表示
```

## M3 pi0 の閉ループ

```bash
uv run unity-vla run -c configs/pi0.yaml --tag pi0
uv run unity-vla run -c configs/pi05.yaml --tag pi05  # 比較
```

`runs/<dir>/ep000_seed0/episode.mp4` で、腕が対象物へ向かうことを確認する。

## M4 評価

```bash
uv run unity-vla eval -c configs/eval_pi0.yaml     # 20 seed × 3 指示 = 60 エピソード
uv run unity-vla eval -c configs/eval_pi05.yaml
```

`summary.json` に全体・指示ごとの成功率、平均ステップ数、エラー数が出る。
成功率を上げる打ち手（順に試す）: カメラ位置・画角 → 照明・材質 → 机と背景を DROID に寄せる → 色のランダム化。
