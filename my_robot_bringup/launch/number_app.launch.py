from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("new_number", "my_number")
    
    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_pub",
        name="number_publisher_launch",
        remappings=[
            remap_number_topic
        ],
        parameters=[
            {"number_to_publish": 4},
            {"publish_freq": 5.0}
        ]
    )

    number_counter_node = Node(
        package="my_py_pkg",
        executable="number_counter",
        name="number_counter_launch",
        remappings=[
            remap_number_topic,
            ("cnt_number", "my_number_cnt")
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    return ld
