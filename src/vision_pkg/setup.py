from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

    # ⭐ 这一段就是 launch 的关键
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='letheon',
    maintainer_email='letheon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            
        'image_repub_node = vision_pkg.image_repub_node:main',

        'd435_node = vision_pkg.d435:main',

        'ShapeColorDetect_Node = vision_pkg.ShapeColorDetect:main',

        'Positionpub_Node = vision_pkg.position_pub:main',

        'ColorDetect_Node = vision_pkg.ColorDetect:main',

	    'vision_bridge = vision_pkg.vision_bridge_node:main',

	    'down_servo_bridge_node=vision_pkg.down_servo_bridge_node:main',

        'DownCamera_Node=vision_pkg.DownCamera:main',

        'qrcode_node=vision_pkg.QRcode:main',
        ],
    },
)
