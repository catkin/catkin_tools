import os
import sys

from setuptools import setup
from setuptools import find_packages


install_requires = [
    'catkin-pkg',
    'setuptools',
    'PyYAML',
]

if sys.version_info[0] == 2 and sys.version_info[1] <= 6:
    install_requires.append('argparse')

this_dir = os.path.abspath(os.path.dirname(__file__))
osx_resources_path = os.path.join(
    this_dir,
    'catkin_tools',
    'notifications',
    'resources',
    'osx',
    'catkin build.app')
osx_notification_resources = [os.path.join(dp, f)
                              for dp, dn, fn in os.walk(osx_resources_path)
                              for f in fn]
src_path = os.path.join(this_dir, 'catkin_tools')
osx_notification_resources = [os.path.relpath(x, src_path)
                              for x in osx_notification_resources]

setup(
    name='catkin_tools',
    version='0.1.0',
    packages=find_packages(exclude=['tests', 'docs']),
    package_data={
        'catkin_tools': [
            'notifications/resources/linux/catkin_icon.png',
        ] + osx_notification_resources
    },
    install_requires=install_requires,
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='William Woodall',
    maintainer_email='william@osrfoundation.org',
    url='http://catkin-tools.readthedocs.org/',
    keywords=['catkin'],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description="Command line tools for working with catkin.",
    long_description="Provides command line tools for working with catkin.",
    license='Apache 2.0',
    test_suite='tests',
    entry_points={
        'console_scripts': [
            'catkin = catkin_tools.commands.catkin:main',
        ],
        'catkin_tools.commands.catkin.verbs': [
            'build = catkin_tools.verbs.catkin_build:description',
            'list = catkin_tools.verbs.catkin_list:description',
        ],
    },
)
