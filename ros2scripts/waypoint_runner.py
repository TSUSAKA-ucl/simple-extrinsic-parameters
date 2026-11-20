#!/usr/bin/env python3

# waypointの入力ファイルをyamlとjsonの両方に対応
# 各waypointのオブジェクトにpositionとorientationキーがある場合、手先の位置姿勢とみなしてgeometry_msgs/PoseStampedをpublishする
# nameとpositionキーがある場合はジョイント値とみなしてsensor_msgs/JointStateをpublishする
# どちらもない場合はエラーとしてログ出力しスキップする
# JointStateはvelocity=[], effort=[]でpublishし、入力ファイルにはnameとpositionの配列があればよい

import sys
import yaml
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray


class WaypointRunner(Node):
    TARGET_DIAG_NAME = 'arm'
    MOTION_STATUS_KEY = 'motion_status'

    def __init__(self, yaml_path):
        super().__init__('waypoint_runner')

        # 1. ウェイポイントの読み込み
        self.waypoints = self.load_waypoints(yaml_path)
        self.current_idx = 0

        # 2. 通信の設定
        self.pub_pose = self.create_publisher(PoseStamped, 'control/move_p', 10)
        self.pub_joint = self.create_publisher(JointState, 'control/move_j', 10)
        self.sub_status = self.create_subscription(
            DiagnosticArray,
            'feedback/arm_status',
            self.status_cb,
            10,
        )

        # 3. ステート管理 ("IDLE", "WAIT_START", "WAIT_DONE", "DWELL", "FINISHED")
        self.state = 'IDLE'
        self.dwell_timer = None
        self.dwell_time_sec = 2.0  # 停止時間（秒）

        self.get_logger().info(f'合計 {len(self.waypoints)} 個のウェイポイントをロードしました')

        # 少し待ってから最初の目標位置を送信（初回トピック接続の確実化のため）
        self.init_timer = self.create_timer(1.0, self.start_first_waypoint)

    def load_waypoints(self, path):
        """YAML/JSONファイルからウェイポイントのリストを読み込む"""
        try:
            with open(path, 'r') as f:
                if path.endswith('.json'):
                    data = json.load(f)
                    if isinstance(data, dict):
                        data = data.get('waypoints', [])
                else:
                    loaded = yaml.safe_load(f) or {}
                    data = loaded.get('waypoints', []) if isinstance(loaded, dict) else loaded
                return data
        except Exception as e:
            self.get_logger().error(f'ウェイポイントファイルの読み込みエラー: {e}')
            sys.exit(1)

    def start_first_waypoint(self):
        """最初のウェイポイントを送信してタイマー解除"""
        self.init_timer.cancel()
        self.send_next_waypoint()

    def send_next_waypoint(self):
        """次のウェイポイントをPublishしてステート更新"""
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('すべてのウェイポイントの移動が完了しました。ノードを終了します。')
            self.state = 'FINISHED'
            return

        wp = self.waypoints[self.current_idx]
        if isinstance(wp, dict) and 'orientation' in wp and 'position' in wp:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            msg.pose.position.x = float(wp['position']['x'])
            msg.pose.position.y = float(wp['position']['y'])
            msg.pose.position.z = float(wp['position']['z'])
            msg.pose.orientation.x = float(wp['orientation']['x'])
            msg.pose.orientation.y = float(wp['orientation']['y'])
            msg.pose.orientation.z = float(wp['orientation']['z'])
            msg.pose.orientation.w = float(wp['orientation']['w'])

            self.get_logger().info(f'[{self.current_idx + 1}/{len(self.waypoints)}] 手先位置姿勢を送信中...')
            self.pub_pose.publish(msg)
        elif isinstance(wp, dict) and 'name' in wp and 'position' in wp:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(wp['name'])
            msg.position = [float(p) for p in wp['position']]
            msg.velocity = []
            msg.effort = []

            self.get_logger().info(f'[{self.current_idx + 1}/{len(self.waypoints)}] ジョイント値を送信中...')
            self.pub_joint.publish(msg)
        else:
            self.get_logger().error(
                f'ウェイポイント {self.current_idx + 1} は無効です。'
                'positionとorientation、またはnameとpositionの両方が必要です。'
            )
            self.current_idx += 1
            self.send_next_waypoint()
            return

        # ステートを「移動開始待ち」へ変更
        self.state = 'WAIT_START'

    def _extract_motion_status(self, msg: DiagnosticArray) -> int | None:
        """DiagnosticArrayから values['motion_status'] の数値（文字列化済み）をintで取得する"""
        for diag in msg.status:
            if diag.name != self.TARGET_DIAG_NAME:
                continue
            for kv in diag.values:
                if kv.key == self.MOTION_STATUS_KEY:
                    try:
                        return int(kv.value.strip())
                    except (ValueError, AttributeError):
                        self.get_logger().warning(
                            f'{self.MOTION_STATUS_KEY} の数値変換に失敗しました: "{kv.value}"'
                        )
                        return None
        return None

    def status_cb(self, msg: DiagnosticArray):
        """ステータストピック（高速周期）の受信コールバック"""
        status = self._extract_motion_status(msg)
        if status is None:
            return

        # 1. 移動開始を待っている状態
        if self.state == 'WAIT_START':
            if status == 1:
                self.get_logger().info('アームが移動を開始しました')
                self.state = 'WAIT_DONE'

        # 2. 移動完了を待っている状態
        elif self.state == 'WAIT_DONE':
            if status == 0:
                self.get_logger().info(f'目標位置に到着。{self.dwell_time_sec} 秒間停止します...')
                self.state = 'DWELL'
                self.dwell_timer = self.create_timer(self.dwell_time_sec, self.on_dwell_complete)

    def on_dwell_complete(self):
        """停止時間が経過した際のタイマーコールバック"""
        if self.dwell_timer is not None:
            self.dwell_timer.cancel()
            self.dwell_timer = None
        self.current_idx += 1
        self.send_next_waypoint()


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 waypoint_runner.py <path_to_waypoints.yaml|json>')
        sys.exit(1)

    waypoint_path = sys.argv[1]

    rclpy.init()
    node = WaypointRunner(waypoint_path)

    try:
        while rclpy.ok() and node.state != 'FINISHED':
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
