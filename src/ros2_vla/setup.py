from setuptools import setup

package_name = 'ros2_vla'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='The ros2_vla package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_capture_node = ros2_vla.audio_capture_node:main',
            'whisper_node = ros2_vla.whisper_node:main',
            'llm_node = ros2_vla.llm_node:main',
            'task_dispatcher_node = ros2_vla.task_dispatcher_node:main',
        ],
    },
)
