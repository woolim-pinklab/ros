from setuptools import find_packages, setup
import os
import glob

package_name = 'soccer_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join("launch", "*.launch.*"))),
        ('share/' + package_name + '/viewer', glob.glob(os.path.join("viewer", "*.html"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pinklab',
    maintainer_email='woolim@pinklab.art',
    description='초간단 turtle soccer — 헤드리스 심판 + three.js 뷰어',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'soccer_referee = soccer_game.soccer_referee:main',
            'example_bot = soccer_game.example_bot:main',
        ],
    },
)
