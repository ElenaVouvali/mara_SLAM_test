from setuptools import setup

package_name = 'slam_evaluation'

setup(
    name=package_name,
    version='0.7.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ELena',
    author_email='evouvali@iknowhow.gr',
    maintainer='Elena Vouvali',
    maintainer_email='evouvali@iknowhow.gr',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A ROS2 Python package that compares ground truth and lidar estimated pose and computes the position and orientation error',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'ground_truth_node = '
        	'slam_evaluation.ground_truth_node:main',
        	'pose_subscriber_node = '
        	'slam_enaluation.pose_subscriber_node:main',
        	'imu_republisher_node = '
        	'slam_evaluation.imu_correct_node:main',        	
        	'ground_truth = slam_evaluation.ground_truth_publisher:main',
        	'error_estimation = slam_evaluation.pose_subscriber:main',
        	'imu_republisher = slam_evaluation.imu_correct:main',
        ],
    },
)
