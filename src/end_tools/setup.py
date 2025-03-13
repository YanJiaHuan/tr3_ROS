from setuptools import setup, find_packages
import glob
import os

package_name = 'end_tools'

# Automatically collect scripts from all submodules' scripts folders
script_files = glob.glob(os.path.join(package_name, '**', '*.py'), recursive=True)

# Create console_scripts entries automatically
entry_points = {
    'console_scripts': [
        f"{os.path.splitext(os.path.basename(script))[0]} = "
        f"{script.replace('/', '.').replace('.py', '')}:main"
        for script in script_files if script.endswith('_node.py')
    ]
}

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),


    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],

    install_requires=['setuptools', 'pybluez'],
    zip_safe=True,
    maintainer='jiahuan',
    maintainer_email='1337842463@qq.com',
    description='ROS2 package wrapping ESP32 end tools (Bluetooth, magnet, LED, etc.)',
    license='MIT',
    tests_require=['pytest'],
    entry_points=entry_points,
)
