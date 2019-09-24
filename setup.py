import argparse
from distutils import log
import os
import site
from stat import ST_MODE
import sys

from setuptools import find_packages
from setuptools import setup
from setuptools.command.install import install

# Setup installation dependencies
install_requires = [
    'catkin-pkg > 0.2.9',
    'setuptools',
    'PyYAML',
    'osrf-pycommon > 0.1.1',
    'trollius'
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


def _resolve_prefix(prefix, type):
    osx_system_prefix = '/System/Library/Frameworks/Python.framework/Versions'
    if type == 'man':
        if prefix == '/usr':
            return '/usr/share'
        if sys.prefix.startswith(osx_system_prefix):
            return '/usr/share'
    elif type == 'bash_comp':
        if prefix == '/usr':
            return '/'
        if sys.prefix.startswith(osx_system_prefix):
            return '/'
    elif type == 'zsh_comp':
        if sys.prefix.startswith(osx_system_prefix):
            return '/usr/local'
    else:
        raise ValueError('not supported type')
    return prefix


def get_data_files(prefix):
    data_files = []

    # Bash completion
    bash_comp_dest = os.path.join(_resolve_prefix(prefix, 'bash_comp'),
                                  'etc/bash_completion.d')
    data_files.append((bash_comp_dest,
                       ['completion/catkin_tools-completion.bash']))

    # Zsh completion
    zsh_comp_dest = os.path.join(_resolve_prefix(prefix, 'zsh_comp'),
                                 'share/zsh/site-functions')
    data_files.append((zsh_comp_dest, ['completion/_catkin']))
    return data_files


class PermissiveInstall(install):

    def run(self):
        install.run(self)
        if os.name == 'posix':
            for file in self.get_outputs():
                # all installed files should be readable for anybody
                mode = ((os.stat(file)[ST_MODE]) | 0o444) & 0o7777
                log.info("changing permissions of %s to %o" % (file, mode))
                os.chmod(file, mode)

        # Provide information about bash completion after default install.
        if (sys.platform.startswith("linux") and
                self.install_data == "/usr/local"):
            log.info("""
----------------------------------------------------------------
To enable tab completion, add the following to your '~/.bashrc':

  source {0}

----------------------------------------------------------------
""".format(os.path.join(self.install_data,
                        'etc/bash_completion.d',
                        'catkin_tools-completion.bash')))

from distutils.core import setup
from distutils.dist import Distribution
dist = Distribution()
dist.parse_config_files()
dist.parse_command_line()
prefix = dist.get_option_dict('install').get('prefix',("default", sys.prefix))[1]

setup(
    name='catkin_tools',
    version='0.4.5',
    packages=find_packages(exclude=['tests*', 'docs']),
    package_data={
        'catkin_tools': [
            'notifications/resources/linux/catkin_icon.png',
            'notifications/resources/linux/catkin_icon_red.png',
            'verbs/catkin_shell_verbs.bash',
            'docs/examples',
        ] + osx_notification_resources
    },
    data_files=get_data_files(prefix),
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
            'env = catkin_tools.verbs.catkin_env:description',
            'init = catkin_tools.verbs.catkin_init:description',
            'list = catkin_tools.verbs.catkin_list:description',
            'locate = catkin_tools.verbs.catkin_locate:description',
            'profile = catkin_tools.verbs.catkin_profile:description',
        ],
        'catkin_tools.jobs': [
            'catkin = catkin_tools.jobs.catkin:description',
            'cmake = catkin_tools.jobs.cmake:description',
        ],
        'catkin_tools.spaces': [
            'build = catkin_tools.spaces.build:description',
            'devel = catkin_tools.spaces.devel:description',
            'install = catkin_tools.spaces.install:description',
            'log = catkin_tools.spaces.log:description',
            'source = catkin_tools.spaces.source:description',
        ],
    },
    cmdclass={'install': PermissiveInstall},
)
