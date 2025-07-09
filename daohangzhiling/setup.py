from setuptools import setup

package_name = 'nav_goal_sender'

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
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='Send goal to Nav2 remotely from VM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_nav_goal = nav_goal_sender.send_nav_goal:main',
            'listen_voice_cmd = nav_goal_sender.distinguish_listener:main'
        ],
    },
)

