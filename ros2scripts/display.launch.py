import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 外部パッケージ（agx_arm_description）のシェアパスを取得
    # ※ source install/setup.bash されていれば、このLaunchがどこにあっても取得可能
    pkg_path = get_package_share_path('agx_arm_description')

    # 2. XacroまたはURDFのパスを設定 (お使いのファイル名に合わせて変更してください)
    # 例: urdf/agx_arm.urdf.xacro もしくは urdf/agx_arm.urdf
    # agx_arm_urdf/piper/urdf/piper_description.urdf
    default_model_path = os.path.join(pkg_path,
                                      'agx_arm_urdf',
                                      'piper',
                                      'urdf',
                                      'piper_description.urdf')
    
    # 3. RVizの設定ファイルのパス (もしパッケージ内にあれば)
    default_rviz_config_path = os.path.join(pkg_path,
                                            'rviz',
                                            'display.rviz')

    # Launch引数の定義
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 4. xacroコマンドを実行してrobot_description（XML文字列）を生成
    # xacroを使う場合は、Command(['xacro ', LaunchConfiguration('model')]) にします
    # ※ URDF（素のXML）の場合は、Command(['cat ', LaunchConfiguration('model')]) にします
    robot_description_content = Command(['cat ', LaunchConfiguration('model')])

    # 5. robot_state_publisher ノードの設定
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    # 6. RViz2 ノードの設定
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # joint_state_publisher はここには含めず、上記2つのノードだけで起動
    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        rviz_node
    ])
