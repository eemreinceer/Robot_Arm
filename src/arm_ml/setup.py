from setuptools import find_packages, setup

package_name = 'arm_ml'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emre Inceer',
    maintainer_email='inceer22@gmail.com',
    description='Deep Learning-based Inverse Kinematics for 6DOF Robotic Arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_ik_dataset = arm_ml.generate_dataset:main',
            'train_ik = arm_ml.train:main',
            'dl_ik_node = arm_ml.dl_ik_node:main',
            'benchmark_ik_latency = arm_ml.benchmark_latency:main',
            'evaluate_ik = arm_ml.evaluate_model:main',
        ],
    },
)
