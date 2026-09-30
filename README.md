# アームの設置位置姿勢とハンドとマーカーの関係の同時推定

## jupyter labの設定

`*.ipynb`はリポジトリに保存しない。JupyterのGUIや
```
jupytext --to wolfram hand-eye-calib.ipynb
```
を使用して`*.wolfram`を生成し、それをリポジトリに保存する。
`wolfram`から`ipynb`の生成は
```
jupytext --to notebook hand-eye-calib.wolfram
```
でできるが、[`jupytext.toml`](./jupytext.toml)が書いてあるのでJupyter labで
自動的に生成されるはず

## 手順(ros2, wolframScript使用)
v4l2カメラとしてRealSense D435iを使用しているためレンズのディストーションは
ファームウェアで補正済としている。自分でディストーションのキャリブレーションを行っている
場合は後述。


1. PiPERアーム先端のフランジにAprilTag(tag36h11)を適当に固定する。
2. [`apriltag_bridge`](https://github.com/TSUSAKA-ucl/apriltag_bridge)を
   使ってv4l2カメラ(webCam)の画像を取得してapril tagのスクリーン上での位置を
   出力する。カメラの解像度が1280x720の場合の例
   ```
    ros2 run apriltag_bridge apriltag_bridge_node --ros-args -p camera_index:=4 | \
	ffplay -f rawvideo -pixel_format bgr24 -video_size 1280x720 -framerate 30 -
	```
3. [`agx_arm_ctrl`](https://github.com/agilexrobotics/agx_arm_ros)と
   [`agx_arm_simple_move`]の`waypoint_runner.py`を使って適当な位置に
   アームを動かし、その間、`ros2 bag record`でデータ取得
   ```
   PYTHONPATH=$VIRTUAL_ENV/lib/python3.10/site-packages:$PYTHONPATH ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py can_port:=can_0 arm_type:=piper follow:=true control:=false
   ```
   `piper-sdk`Pythonパッケージのインストール方法次第で`PYTHONPATH`の環境変数設定が必要になることがある
4. 取得したbagファイルから`/feedback/joint_states`, `/feedback/arm_status`,
   `/apriltag_detections`をcsvに抽出  
   ```
   ros2 run bag2csv bag2csv /feedback/joint_states rosbag2_2026_09_16-15_56_51/ > rosbag2_2026_09_16-15_56_51-joint_states.csv
   ros2 run bag2csv bag2csv /feedback/arm_status rosbag2_2026_09_16-15_56_51/ > rosbag2_2026_09_16-15_56_51-arm_status.csv
   ros2 run bag2csv bag2csv /apriltag_detections rosbag2_2026_09_16-15_56_51/ > rosbag2_2026_09_16-15_56_51-apriltag_detections.csv
   ```
4. [`read-tag-robot-data.wolfram`](./read-tag-robot-data.wolfram)ノートブックで
   取得データ(CSV)を読み取り所定の部分を取り出す。`uvDataSetected`と`poseDataSelected`
5. [`camera-pose-quat3.wolfram`](./camera-pose-quat3.wolfram)ノートブックで
   最もフィットするパラメーターを求める  
   以上でカメラの設置位置と焦点距離の同時キャリブレーション完了
6. ffmpegとMediaMTXで同v4l2カメラ画像を配信する
   [こちら参照](https://github.com/TSUSAKA-ucl/mediamtx-example)
7. Webブラウザー上で、カメラ画像とPiPERのモデルを重ねる  
   MediaMTXと同じホストで、
   `https://github.com/TSUSAKA-ucl/ik-cd-worker-minimum-test`の`piper-webrtc`ブランチ
   をcloneしてきて、`pnpm install && npx copy-assets && pnpm dev`。
   サーバー認証用の鍵がMediaMTXのものと一致していること

### 自分でディストーションのキャリブレーションを行っている場合

ディストーション補正の式をそのまま`camera-pose-quat3.wolfram`の
`homo`関数に(定数で)仕掛けるのが簡単。あるいは画像全体を
undistort(rectify)した中でのapril tagの位置を出す。ただし現在
`apriltag_bridge`はimage pipeline対応ではなくv4l2を直接OpenCVで読み出
しているため`apriltag_bridge`が`CameraInfo`をsubscribeしてundistortす
るようにコード変更する必要がある
