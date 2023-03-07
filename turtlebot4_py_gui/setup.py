from setuptools import setup

package_name = 'turtlebot4_py_gui'

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
    maintainer='chris',
    maintainer_email='chris@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = turtlebot4_py_gui.turtlebot4_py_gui_node:main',
            'service = turtlebot4_py_gui.srv_test:main',
            'demo = turtlebot4_py_gui.demo_node:main'
        ],
    },
)
