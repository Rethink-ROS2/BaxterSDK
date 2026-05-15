from setuptools import find_packages, setup

package_name = 'baxter_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andnet DeBoer',
    maintainer_email='deboerandnet@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'analog_io_rampup = baxter_examples.analog_io_rampup:main',
            'xdisplay_image = baxter_examples.xdisplay_image:main',
            'head_wobbler = baxter_examples.head_wobbler:main',
            'navigator_io = baxter_examples.navigator_io:main',
        ],
    },
)
