from setuptools import setup

package_name = 'kuka_r900_gui_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyqt5', 'tf-transformations'],
    zip_safe=True,
    maintainer='hk',
    maintainer_email='hk@example.com',
    description='Simple GUI for KUKA IK target pose',
    entry_points={
        'console_scripts': [
            'ik_gui = kuka_r900_gui_interface.gui_node:main',
        ],
    },
)
