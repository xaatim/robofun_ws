from setuptools import find_packages, setup

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='xayari229@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "camera_pub = robot_vision.camera_pub: main",
          "camera_sub = robot_vision.camera_sub: main",
          "face_rec = robot_vision.face_rec: main",
          "alert_sub = robot_vision.alert_sub: main",
          "web_teleop_node = robot_vision.web_teleop_node: main",
          "latency_monitor = robot_vision.latency_monitor: main",
          
        ],
    },
)
