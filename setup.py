from setuptools import find_packages, setup

package_name = 'ros_gpt_assistant_nav2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'openai==1.28.1',
    ],
    zip_safe=True,
    maintainer='Elettra Robotics Lab',
    maintainer_email='info@elettraroboticslab.it',
    description='A ROS2 node that uses OpenAI GPT Assistant to generate navigation instructions for nav2.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openai_assistant_parser = ros_gpt_assistant_nav2.openai_assistant_parser:main',
            'ros_gpt_assistant_nav2 = ros_gpt_assistant_nav2.ros_gpt_assistant_nav2:main',
            'teleop_navigator = ros_gpt_assistant_nav2.teleop_navigator:main',
        ],
    },
)
