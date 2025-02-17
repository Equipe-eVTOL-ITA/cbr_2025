from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Declare the nodes
    camera_publisher_node = Node(
        package='camera_publisher',
        executable='webcam',
        name='camera_publisher'
    )

    gesture_classifier_node = Node(
        package='gesture_classifier',
        executable='gesture_classifier',
        name='gesture_classifier',
        parameters=[{'num_hands': 2}]  # Set num_hands parameter
    )

    frtl_2024_fase4_node = Node(
        package='frtl_2024_fase4',
        executable='fase4',
        name='frtl_2024_fase4'
    )

    # TimerAction to delay the launch of frtl_2024_fase4 by 15 seconds
    delayed_fase4_node = TimerAction(
        period=15.0,  # 15 seconds delay
        actions=[frtl_2024_fase4_node]
    )

    # Create a LaunchDescription and add the nodes to it
    return LaunchDescription([
        camera_publisher_node,
        gesture_classifier_node,
        delayed_fase4_node
    ])
