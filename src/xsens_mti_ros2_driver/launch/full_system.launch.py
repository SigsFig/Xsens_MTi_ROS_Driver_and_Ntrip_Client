import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/xsens',
        description='Serial port for the XSens device (default: /dev/xsens via udev rule)'
    )
    ntrip_host_arg = DeclareLaunchArgument(
        'ntrip_host', default_value='ntrip.earthscope.org',
        description='NTRIP caster hostname'
    )
    ntrip_mountpoint_arg = DeclareLaunchArgument(
        'ntrip_mountpoint', default_value='CLAR_RTCM3P3',
        description='NTRIP mountpoint'
    )
    ntrip_username_arg = DeclareLaunchArgument(
        'ntrip_username', default_value='confident_kowalevski',
        description='NTRIP username'
    )
    ntrip_password_arg = DeclareLaunchArgument(
        'ntrip_password', default_value='0bt0VSR2vxHrZn2g',
        description='NTRIP password'
    )
    heading_offset_arg = DeclareLaunchArgument(
        'heading_offset_deg', default_value='30.0',
        description='Compass heading offset in degrees to zero North'
    )

    # --------------------------------------------------------------------------
    # XSens driver node
    # Baudrate is passed as an integer — the driver's get_parameter("baudrate")
    # expects int and will silently ignore a string value.
    # --------------------------------------------------------------------------
    xsens_params = Path(
        get_package_share_directory('xsens_mti_ros2_driver'), 'param', 'xsens_mti_node.yaml'
    )
    xsens_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[
            xsens_params,
            {
                'scan_for_devices': False,
                'port': LaunchConfiguration('port'),
                'baudrate': 921600,       # must be int — driver uses get_parameter<int>
                'enable_deviceConfig': False,
            }
        ],
    )

    # --------------------------------------------------------------------------
    # NTRIP client node
    # send_default_gga=False means the node waits for real NMEA from /nmea,
    # which the XSens driver publishes (pub_nmea: true in xsens_mti_node.yaml).
    # --------------------------------------------------------------------------
    ntrip_node = Node(
        package='ntrip',
        executable='ntrip',
        name='ntrip_client',
        output='screen',
        parameters=[{
            'host':               LaunchConfiguration('ntrip_host'),
            'port':               2101,   # int — NTRIP TCP port
            'mountpoint':         LaunchConfiguration('ntrip_mountpoint'),
            'username':           LaunchConfiguration('ntrip_username'),
            'password':           LaunchConfiguration('ntrip_password'),
            'send_default_gga':   False,  # use real NMEA from XSens
            'debug':              False,
        }],
    )

    # --------------------------------------------------------------------------
    # Subscriber / path planner
    # Path is resolved from HOME so it works on any machine (laptop or Jetson).
    # --------------------------------------------------------------------------
    subscriber_script = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'src', 'xsens_subscriber.py'
    )
    subscriber_process = ExecuteProcess(
        cmd=[
            'python3',
            subscriber_script,
            '--ros-args',
            '-p', ['heading_offset_deg:=', LaunchConfiguration('heading_offset_deg')],
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Arguments
        port_arg,
        ntrip_host_arg,
        ntrip_mountpoint_arg,
        ntrip_username_arg,
        ntrip_password_arg,
        heading_offset_arg,

        # Nodes
        xsens_node,
        ntrip_node,
        subscriber_process,
    ])
