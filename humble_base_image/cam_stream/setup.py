from setuptools import setup

package_name = 'cam_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumit',
    maintainer_email='paul.sumit.cste@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cam_publisher = cam_stream.cam_publisher:main",
            "cam_subscriber = cam_stream.cam_subscriber:main",
            "binary_file_publisher = cam_stream.binary_file_publisher:main",
            "binary_file_subscriber = cam_stream.binary_file_subscriber:main",
            "pub_binary_file_with_timestamp = cam_stream.pub_binary_file_with_timestamp:main",
            "sub_binary_file_with_timestamp = cam_stream.sub_binary_file_with_timestamp:main",
            "binary_publisher_relay_subscriber = cam_stream.binary_publisher_relay_subscriber:main",
            "cam_subscriber_relay_publisher = cam_stream.cam_subscriber_relay_publisher:main",
            "cam_publisher_relay_subscriber_sterio = cam_stream.cam_publisher_relay_subscriber_sterio:main",
            "imu_subscriber = cam_stream.imu_subscriber:main",
            "intermidiate_processing_time_collector_pub = cam_stream.intermidiate_processing_time_collector_pub:main",
            "intermidiate_processing_time_collector_binary = cam_stream.intermidiate_processing_time_collector_binary:main",
            "binary_subscriber_relay_publisher = cam_stream.binary_subscriber_relay_publisher:main",
            "string_publisher_relay_subscriber = cam_stream.string_publisher_relay_subscriber:main",
            "string_subscriber_relay_publisher = cam_stream.string_subscriber_relay_publisher:main",
            "string_transfer_rate_calculator = cam_stream.string_transfer_rate_calculator:main",
            "binary_image_transfer_rate_calculator = cam_stream.binary_image_transfer_rate_calculator:main",
            "intermidiate_processing_time_collector_imu = cam_stream.intermidiate_processing_time_collector_imu:main",
            "intermidiate_processing_time_collector_string = cam_stream.intermidiate_processing_time_collector_string:main",
            "imu_transfer_rate_calculator = cam_stream.imu_transfer_rate_calculator:main",
            "imu_publisher_relay_subscriber = cam_stream.imu_publisher_relay_subscriber:main",
            "imu_subscriber_relay_publisher = cam_stream.imu_subscriber_relay_publisher:main",
            "binary_transfer_rate_publisher = cam_stream.binary_transfer_rate_publisher:main",
            "string_transfer_rate_publisher = cam_stream.string_transfer_rate_publisher:main",
            "imu_transfer_rate_publisher = cam_stream.imu_transfer_rate_publisher:main",
            "cam_feed_publisher_relay_subscriber = cam_stream.cam_feed_publisher_relay_subscriber:main",
            "cam_feed_subscriber_relay_publisher = cam_stream.cam_feed_subscriber_relay_publisher:main"

        ],
    },
)
