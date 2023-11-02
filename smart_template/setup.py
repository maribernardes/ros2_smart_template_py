from setuptools import setup

package_name = 'smart_template'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Moreira (BWH)',
    maintainer_email='plopesdafrotamoreira@bwh.harvard.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'template = smart_template.template:main',
            'keypress = smart_template.keypress:main',
            'initialization = smart_template.initialization:main'
            'smart_needle_interface = smart_template.smart_needle_interface:main'
            'mri_tracking_interface = smart_template.mri_tracking_interface:main'
        ],
    },
)
