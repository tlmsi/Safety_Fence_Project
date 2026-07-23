from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch the Stage 1 sorting-cell Gazebo world."""

    gazebo_package = get_package_share_directory("sorting_cell_gazebo")
    ros_gz_sim_package = get_package_share_directory("ros_gz_sim")

    world_file = f"{gazebo_package}/worlds/stage1_world.sdf"
    gazebo_launch_file = f"{ros_gz_sim_package}/launch/gz_sim.launch.py"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "gz_args": f"-r {world_file}",
            "on_exit_shutdown": "True",
        }.items(),
    )

    return LaunchDescription([
        gazebo,
    ])
