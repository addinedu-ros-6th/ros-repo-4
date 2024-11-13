from setuptools import find_packages, setup

package_name = 'aruco_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'aruco_follower.aruco_follower'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치 경로 설정
        ('share/' + package_name + '/launch', ['launch/aruco_follower_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sehyung',
    maintainer_email='dkapqk8282@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_follower = aruco_follower.aruco_follower:main'
        ],
    },
)
