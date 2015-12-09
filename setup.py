from distutils import log
import os
import platform
from setuptools import find_packages
from setuptools import setup
from setuptools.command.install import install
from stat import ST_MODE
import sys


# Setup installation dependencies
install_requires = [
    'catkin-pkg > 0.2.9',
    'setuptools',
    'PyYAML',
]
if sys.version_info[0] == 2 and sys.version_info[1] <= 6:
    install_requires.append('argparse')

# Figure out the resources that need to be installed
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

# Figure out where to install the data_files
data_files = []
import argparse
parser = argparse.ArgumentParser(description="shouldn't see this")
parser.add_argument('--prefix')
opts, _ = parser.parse_known_args(list(sys.argv))
target_prefix = sys.prefix
if opts.prefix:
    target_prefix = opts.prefix
# If the target is the root system target, use /etc
if target_prefix == '/usr':
    target_prefix = '/'
if (
    platform.platform().lower().startswith('darwin') and
    target_prefix.startswith('/System/Library/Frameworks/Python.framework')
):
    # This is the system install of Python on OS X, install to `/etc`.
    target_prefix = '/'
completion_dest = os.path.join(target_prefix, 'etc/bash_completion.d')
data_files.append((
    completion_dest,
    ['completion/catkin_tools-completion.bash']))


class PermissiveInstall(install):
    def run(self):
        install.run(self)
        if os.name == 'posix':
            for file in self.get_outputs():
                # all installed files should be readable for anybody
                mode = ((os.stat(file)[ST_MODE]) | 0o444) & 0o7777
                log.info("changing permissions of %s to %o" % (file, mode))
                os.chmod(file, mode)


setup(
    name='catkin_tools',
    version='0.3.0',
    packages=find_packages(exclude=['tests', 'docs']),
    package_data={
        'catkin_tools': [
            'notifications/resources/linux/catkin_icon.png',
        ] + osx_notification_resources
    },
    data_files=data_files,
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
            'clean = catkin_tools.verbs.catkin_clean:description',
            'config = catkin_tools.verbs.catkin_config:description',
            'create = catkin_tools.verbs.catkin_create:description',
            'init = catkin_tools.verbs.catkin_init:description',
            'list = catkin_tools.verbs.catkin_list:description',
            'locate = catkin_tools.verbs.catkin_locate:description',
            'profile = catkin_tools.verbs.catkin_profile:description',
        ],
    },
    cmdclass={'install': PermissiveInstall},
)
