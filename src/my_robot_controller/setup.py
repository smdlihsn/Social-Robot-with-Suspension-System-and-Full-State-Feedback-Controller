from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_controller'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append((os.path.join('share', package_name, path), [os.path.join(path, filename)]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),] + package_files('models/'),  # This automatically grabs EVERY subfolder in models


    # data_files=[
    #     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
        
    #     # Copy the launch files
    #     (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    #     # Copy the world files
    #     (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        
    #     #Robot A: Original
    #     (os.path.join('share', package_name, 'models', 'grey_robot'), glob('models/grey_robot/*.sdf') + glob('models/grey_robot/*.config')),
    #     (os.path.join('share', package_name, 'models', 'grey_robot', 'meshes'), glob('models/grey_robot/meshes/*')),
        
    #     # Robot B: Rear Drive
    #     (os.path.join('share', package_name, 'models', 'grey_robot_rear_drive'), 
    #      glob('models/grey_robot_rear_drive/*.sdf') + glob('models/grey_robot_rear_drive/*.config')),

    #     # Human Model
    #     (os.path.join('share', package_name, 'models/human_model'), glob('models/human_model/model*')),
    #     (os.path.join('share', package_name, 'models/human_model/meshes'), glob('models/human_model/meshes/*')),

    #     # This line tells ROS to copy everything in 'models' to the install folder
    #     # (os.path.join('share', package_name, 'models'), glob('models/**/*'), glob('models/**/**/*')),
        
    #     # This line tells ROS to copy everything in 'config' to the install folder
    #     # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hasanu',
    maintainer_email='hasanu@todo.todo',
    description='Active Suspension',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'active_suspension = my_robot_controller.fsf_control:main',
            'lets_dance = my_robot_controller.dance_robot:main',
            'human_tracker = my_robot_controller.human_tracker:main',
            'square_loop = my_robot_controller.square_loop:main',
            'dummy_driver = my_robot_controller.dummy_control:main',
            'old_fsf = my_robot_controller.fsf_control_old:main',
            'imufix_fsf = my_robot_controller.imufix_fsf:main',
        ],
    },
)
