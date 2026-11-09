# ロボットモデル

## 生成

`tools/build_robot_assets.py` が mujoco_menagerie の `franka_emika_panda/panda_nohand.xml` と
`robotiq_2f85/2f85.xml` を読み、Unity 座標に変換したメッシュ（`*.bytes`）と `robot_manifest.json` を書く。
同時に、MuJoCo の順運動学を Unity の規約で再現できるかを自己検証し、参照値
`fk_reference.json`（50 姿勢 × 全リンク）を出す。

## 座標変換

- MuJoCo/ROS: 右手系 Z-up。Unity: 左手系 Y-up、ロボット前方 = +Z。
- `v_unity = C v_ros`、`C = [[0,-1,0],[0,0,1],[1,0,0]]`（det = -1）。回転は `C R Cᵀ`。
- 回転関節 a 周りの +θ（右手系）は Unity で `Quaternion.AngleAxis(-θ, C a)`。manifest の `sign: -1`。
- ArticulationBody の reduced coordinate の向きは仮定しない。起動時に各関節を 0.25 rad 動かして
  実測し（`RobotFactory.CalibrateSigns`、`physSign`）、`p = q × sign × physSign` で変換する。
- メッシュの面の向きも Unity の Cube と比較して自動で合わせる（`MeshLibrary`）。

## グリッパー（2F-85）

実機は閉ループのリンク機構で、ArticulationBody（木構造）では表せない。

- **見た目**: driver / coupler / spring_link / follower を、開度 g の表（21 点）から運動学的に動かす。
  平行把持の近似（spring と driver が同じ角度、follower は逆回転でパッドを平行に保つ）。
- **物理**: 各指のパッドを「横（開閉方向）+ 縦（指の方向）」の 2 段 prismatic 関節で駆動し、
  実機パッドの軌跡（ストローク約 87 mm、閉じると約 28 mm 上がる）をそのまま辿る。
  把持力の上限は `RobotSettings.gripForce`（既定 40 N）。
- 観測の `gripper_position` はパッドの実位置から逆算するので、物を掴むと 1 未満で止まる（実機と同じ）。

## 物理設定

- 腕は重力オフ + 高剛性 PD（Franka のトルク上限 87/12 N·m）。DROID の実機コントローラ相当の追従を優先。
- 物理刻み 0.002 s、1 制御ステップ = 33 サブステップ。
- ロボット同士の衝突は無効、ロボット–物体・机–物体は有効。
