# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("david_urdf3", package_name="srdf2").to_moveit_configs()
#     return generate_demo_launch(moveit_config)


from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("david_urdf3", package_name="srdf2")
        .robot_description(file_path="config/david_urdf3.urdf.xacro")
        .robot_description_semantic(file_path="config/david_urdf3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")           # ✅ load kinematics
        .trajectory_execution(file_path="config/moveit_controllers.yaml")           # ✅ load controllers
        .planning_pipelines(pipelines=["ompl"])                                     # optional but good
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config)
