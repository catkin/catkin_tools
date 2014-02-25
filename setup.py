from setuptools import setup
from setuptools import find_packages


install_requires = [
    'catkin-pkg',
    'distribute',
]

setup(
    name='catkin_tools',
    version='0.0.0',
    packages=find_packages(exclude=['tests', 'docs']),
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
        'License :: OSI Approved :: Apache 2.0',
        'Programming Language :: Python',
    ],
    description="Command line tools for working with catkin.",
    long_description="""\
This python package provides command line tools for working with catkin, \
catkin packages, and catkin workspaces. The provided tools ease development \
tasks like building one to many catkin packages at the same time, preparing \
catkin packages for release, and other things.""",
    license='Apache 2.0',
    test_suite='tests',
    entry_points={
        'console_scripts': [
            'catkin = catkin_tools.commands.catkin:main',
        ],
        'catkin_tools.commands.catkin.verbs': [
            'build = catkin_tools.verbs.catkin_build:description',
        ],
    },
)
