from setuptools import setup

package_name = 'mybot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='我的第一个 ROS2 Python 包',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'circle_mover = mybot.circle_mover:main',
        ],
    },
)
