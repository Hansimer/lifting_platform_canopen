# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import OpaqueFunction
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare


# def launch_setup(context, *args, **kwargs):

#     prefix = LaunchConfiguration("prefix")
#     can_interface_name = LaunchConfiguration("can_interface_name")

#     # bus configuration
#     bus_config_package = LaunchConfiguration("bus_config_package")
#     bus_config_directory = LaunchConfiguration("bus_config_directory")
#     bus_config_file = LaunchConfiguration("bus_config_file")
#     bus_config = PathJoinSubstitution(
#         [FindPackageShare(bus_config_package), bus_config_directory, bus_config_file]
#     )

#     # master configuration
#     master_config_package = LaunchConfiguration("master_config_package")
#     master_config_directory = LaunchConfiguration("master_config_directory")
#     master_config_file = LaunchConfiguration("master_config_file")
#     master_config = PathJoinSubstitution(
#         [FindPackageShare(master_config_package), master_config_directory, master_config_file]
#     )

#     # ros2 control configuration
#     ros2_control_config_package = LaunchConfiguration("ros2_control_config_package")
#     ros2_control_config_directory = LaunchConfiguration("ros2_control_config_directory")
#     ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
#     ros2_control_config = PathJoinSubstitution(
#         [
#             FindPackageShare(ros2_control_config_package),
#             ros2_control_config_directory,
#             ros2_control_config_file,
#         ]
#     )

#     # robot description
#     description_package = LaunchConfiguration("description_package")
#     description_file = LaunchConfiguration("description_file")
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [FindPackageShare(description_package), "urdf", description_file]
#             ),
#             " ",
#             "bus_config_path:=",
#             bus_config,
#             " ",
#             "master_dcf:=",
#             master_config,
#             " ",
#             "prefix:=",
#             prefix,
#             " ",
#             "can_interface_name:=",
#             can_interface_name,
#             " ",
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     # control node
#     control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_description, ros2_control_config],
#         output="screen",
#     )

#     # robot state publisher
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description],
#     )

#     # controller spawners
#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#         output="screen",
#     )

#     lifting_platform_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["lifting_platform_controller", "--controller-manager", "/controller_manager"],
#         output="screen",
#     )

#     nodes_to_start = [
#         control_node,
#         robot_state_publisher_node,
#         joint_state_broadcaster_spawner,
#         lifting_platform_controller_spawner,
#     ]

#     return nodes_to_start


# def generate_launch_description():

#     declared_arguments = []
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "prefix", description="Prefix.", default_value=""
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "can_interface_name",
#             default_value="can0",
#             description="Interface name for can",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "description_package",
#             description="Package where urdf file is stored.",
#             default_value="lifting_platform_canopen",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "description_file",
#             description="Name of the urdf file.",
#             default_value="lifting_platform.urdf.xacro",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "ros2_control_config_package",
#             default_value="lifting_platform_canopen",
#             description="Path to ros2_control configuration.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "ros2_control_config_directory",
#             default_value="config",
#             description="Path to ros2_control configuration.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "ros2_control_config_file",
#             default_value="ros2_controllers.yaml",
#             description="Path to ros2_control configuration.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "bus_config_package",
#             default_value="lifting_platform_canopen",
#             description="Path to bus configuration.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "bus_config_directory",
#             default_value="config/lifting_platform",
#             description="Path to bus configuration.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "bus_config_file",
#             default_value="bus.yml",
#             description="Path to bus configuration.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "master_config_package",
#             default_value="lifting_platform_canopen",
#             description="Path to master configuration file (*.dcf)",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "master_config_directory",
#             default_value="config",
#             description="Path to master configuration file (*.dcf)",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "master_config_file",
#             default_value="master.dcf",
#             description="Path to master configuration file (*.dcf)",
#         )
#     )

#     return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    can_interface_name = LaunchConfiguration("can_interface_name")

    # bus configuration
    bus_config_package = LaunchConfiguration("bus_config_package")
    bus_config_directory = LaunchConfiguration("bus_config_directory")
    bus_config_file = LaunchConfiguration("bus_config_file")
    bus_config = PathJoinSubstitution(
        [FindPackageShare(bus_config_package), bus_config_directory, bus_config_file]
    )

    # master configuration
    master_config_package = LaunchConfiguration("master_config_package")
    master_config_directory = LaunchConfiguration("master_config_directory")
    master_config_file = LaunchConfiguration("master_config_file")
    master_config = PathJoinSubstitution(
        [FindPackageShare(master_config_package), master_config_directory, master_config_file]
    )

    # ros2 control configuration
    ros2_control_config_package = LaunchConfiguration("ros2_control_config_package")
    ros2_control_config_directory = LaunchConfiguration("ros2_control_config_directory")
    ros2_control_config_file = LaunchConfiguration("ros2_control_config_file")
    ros2_control_config = PathJoinSubstitution(
        [
            FindPackageShare(ros2_control_config_package),
            ros2_control_config_directory,
            ros2_control_config_file,
        ]
    )

    # robot description
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "bus_config:=",
            bus_config,
            " ",
            "master_config:=",
            master_config,
            " ",
            "can_interface_name:=",
            can_interface_name,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # control node
    # 实时性优化：整个进程限定在隔离核 {2,5} 上运行。
    #   - RT 控制环线程（JTC + 硬件读写）由 controller_manager 参数
    #     thread_priority/cpu_affinity 单独绑定核 5（SCHED_FIFO 85）
    #   - lely CANopen master（SYNC 生成/PDO 交换）及进程内其他线程落在核 2，
    #     不与 RT 控制环竞争
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="lifting_platform",
        parameters=[robot_description, ros2_control_config],
        output="screen",
        prefix=["taskset -c 2,5 "],
    )

    # robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="lifting_platform",
        output="both",
        parameters=[robot_description],
    )

    # controller spawners
    # NOTE: use RELATIVE controller_manager name so that it is expanded with the
    # node namespace (/lifting_platform/controller_manager) and does NOT collide
    # with another ROS2 system's root-level /controller_manager on the same network.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="lifting_platform",
        arguments=["lifting_joint_state_broadcaster", "--controller-manager", "controller_manager"],
        output="screen",
    )

    cia402_device_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="lifting_platform",
        arguments=["cia402_device_1_controller", "--controller-manager", "controller_manager"],
    )

    # 中间层 —— Joint Trajectory Controller (JTC)
    # 接收 MoveIt 轨迹（/lifting_platform/lifting_platform_controller/follow_joint_trajectory），
    # 在 RT 控制环内按时间戳插值目标位置并写入 position 命令接口。
    # 默认以 --inactive 加载（不占用 updown/position 接口，forward 控制器处于激活态）。
    # 需要 MoveIt 轨迹执行时，先停 forward 再激活 JTC：
    #   ros2 run controller_manager deactivate_controller \
    #     lifting_forward_position_controller --controller-manager /lifting_platform/controller_manager
    #   ros2 run controller_manager activate_controller \
    #     lifting_platform_controller --controller-manager /lifting_platform/controller_manager
    lifting_platform_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="lifting_platform",
        arguments=[
            "lifting_platform_controller",
            "--controller-manager", "controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    # forward 控制器（设计保留，恢复 spawner，默认激活用于手动位置控制）
    # 注意：forward 与 JTC 共用 updown/position 命令接口，不能同时激活。
    # 通过话题发布目标位置：
    #   ros2 topic pub /lifting_forward_position_controller/commands \
    #     std_msgs/msg/Float64MultiArray "{data: [0.3]}"
    lifting_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="lifting_platform",
        arguments=[
            "lifting_forward_position_controller",
            "--controller-manager", "controller_manager",
        ],
        output="screen",
    )

    # CSP init node - delayed start to ensure controller services are ready
    # This node will:
    # 1. Wait for /joint_states to confirm motor communication
    # 2. Call init to initialize CANopen bus
    # 3. Switch to CSP mode
    csp_init_node = TimerAction(
        period=5.0,  # Wait 5 seconds for controller_manager and services to be ready
        actions=[
            Node(
                package="lifting_platform_canopen",
                executable="csp_init_node",
                name="csp_init_node",
                output="screen",
                parameters=[{
                    "controller_name": "cia402_device_1_controller",
                    "max_retries": 3,
                }],
                namespace="lifting_platform",
            ),
        ],
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        cia402_device_controller_spawner,
        lifting_platform_controller_spawner,       # JTC，--inactive 加载
        lifting_forward_position_controller_spawner,  # forward，默认激活
        csp_init_node,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "name", description="robot name", default_value="lifting_platform"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("prefix", description="Prefix.", default_value="")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="can0",
            description="Interface name for can",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Package where urdf file is stored.",
            default_value="lifting_platform_canopen",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            description="Name of the urdf file.",
            default_value="lifting_platform.urdf.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_package",
            default_value="lifting_platform_canopen",
            description="Path to ros2_control configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_directory",
            default_value="config",
            description="Path to ros2_control configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_control_config_file",
            default_value="ros2_controllers.yaml",
            description="Path to ros2_control configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_package",
            default_value="lifting_platform_canopen",
            description="Path to bus configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_directory",
            default_value="config/lifting_platform",
            description="Path to bus configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bus_config_file",
            default_value="bus.yml",
            description="Path to bus configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_package",
            default_value="lifting_platform_canopen",
            description="Path to master configuration file (*.dcf)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_directory",
            default_value="config/lifting_platform",
            description="Path to master configuration file (*.dcf)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "master_config_file",
            default_value="master.dcf",
            description="Path to master configuration file (*.dcf)",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])