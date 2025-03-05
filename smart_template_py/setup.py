import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'smart_template_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),  
        ('share/' + package_name, ['plugin.xml']),      
        ('share/' + package_name + '/files', glob('files/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mariana Bernardes (BWH), Pedro Moreira (BWH)',
    maintainer_email='mcostabernardesmatias@bwh.harvard.edu, plopesdafrotamoreira@bwh.harvard.edu',
    description='SmartTemplate python package with GUI plugin',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'template = smart_template_py.template:main',
            'virtual_template = smart_template_py.virtual_template:main',
            'world_pose_listener = smart_template_py.world_pose_listener:main',
            'keypress = smart_template_py.keypress:main',
        ],
        'rqt_gui_py.plugins': [
            'smart_template_gui = smart_template_py.smart_template_gui:SmartTemplateGUIPlugin',
        ],
    },
)
