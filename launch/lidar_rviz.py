import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    # 获取当前工作目录
    current_dir = os.getcwd()

    # 启动rviz2并加载配置文件
    rviz2_node = ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(current_dir, 'rviz.rviz')],
        output='screen'
    )

    ld.add_action(rviz2_node)

    return ld
