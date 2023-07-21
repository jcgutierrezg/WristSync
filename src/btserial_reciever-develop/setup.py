from setuptools import setup

package_name = 'btserial_reciever'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Camilo Gutierrez',
    maintainer_email='jc.gutierrezg@uniandes.edu.co',
    description='This package contains a bluetooth serial reciever used to recieve messages from the RPGlove and send them to ROS2 to control Robocols arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reciever = btserial_reciever.reciever:main',
            'translate_quaternion = btserial_reciever.translate_quaternion:main'
        ],
    },
)
