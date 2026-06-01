from setuptools import setup

package_name = 'voice_alert'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jungju',
    maintainer_email='jungju3756@gmail.com',
    description='ToF 장애물 감지 음성 알림 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'voice_alert_node = voice_alert.voice_alert_node:main',
        ],
    },
)
