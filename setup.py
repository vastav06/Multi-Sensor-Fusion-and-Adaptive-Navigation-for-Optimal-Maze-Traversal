from setuptools import setup

package_name = 'carllab6'

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
    maintainer='donatello',
    maintainer_email='donatello@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'class = carllab6.classifierNode:main',
            'range = carllab6.getObjectRange:main',  
            'go = carllab6.goToWall:main',                            
        ],
    },
)
