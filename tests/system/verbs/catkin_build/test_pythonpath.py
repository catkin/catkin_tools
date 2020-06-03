import os
import re
import shutil
import subprocess

from tests.system.workspace_factory import workspace_factory
from tests.utils import redirected_stdio, catkin_success

RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

"""
These tests check if the PYTHONPATH environment variable is set correctly when building a
plain cmake package. The tests are run for python version 2 and 3 and for install and
devel workspaces. The workspaces are set to merged because for linked workspaces, 
the catkin that is injected by catkin_tools_prebuild generates the setup files.
"""


def test_python2_devel():
    with workspace_factory() as wf:
        os.mkdir(os.path.join(wf.workspace, 'src'))
        shutil.copytree(
            os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'cmake_pkg'),
            os.path.join('src', 'cmake_pkg'))
        assert catkin_success(['config', '--merge-devel'])
        assert catkin_success(['build', '-DPYTHON_VERSION=2'])
        pythonpaths = subprocess.check_output(
            ['bash', '-c', 'source ' + wf.workspace + '/devel/setup.sh && echo $PYTHONPATH']).decode().split(':')
        ws_pythonpath = [p for p in pythonpaths if p.startswith(wf.workspace)][0]
        assert ws_pythonpath
        # it might be dist-packages (debian) or site-packages
        assert re.match('^' + wf.workspace + '/devel/lib/python2\.\d/(site|dist)-packages$', ws_pythonpath)


def test_python3_devel():
    with workspace_factory() as wf:
        os.mkdir(os.path.join(wf.workspace, 'src'))
        shutil.copytree(
            os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'cmake_pkg'),
            os.path.join('src', 'cmake_pkg'))
        assert catkin_success(['config', '--merge-devel'])
        assert catkin_success(['build', '-DPYTHON_VERSION=3'])
        pythonpaths = subprocess.check_output(
            ['bash', '-c', 'source ' + wf.workspace + '/devel/setup.sh && echo $PYTHONPATH']).decode().split(':')
        ws_pythonpath = [p for p in pythonpaths if p.startswith(wf.workspace)][0]
        assert ws_pythonpath
        # it might be python3/dist-packages (debian) or python3.x/site-packages
        assert (ws_pythonpath == wf.workspace + '/devel/lib/python3/dist-packages' or
                re.match('^' + wf.workspace + '/devel/lib/python3\.\d/site-packages$', ws_pythonpath))


def test_python2_install():
    with workspace_factory() as wf:
        os.mkdir(os.path.join(wf.workspace, 'src'))
        shutil.copytree(
            os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'cmake_pkg'),
            os.path.join('src', 'cmake_pkg'))
        assert catkin_success(['config', '--merge-devel', '--merge-install', '--install'])
        assert catkin_success(['build', '-DPYTHON_VERSION=2'])
        pythonpaths = subprocess.check_output(
            ['bash', '-c', 'source ' + wf.workspace + '/install/setup.sh && echo $PYTHONPATH']).decode().split(':')
        ws_pythonpath = [p for p in pythonpaths if p.startswith(wf.workspace)][0]
        assert ws_pythonpath
        # it might be dist-packages (debian) or site-packages
        assert re.match('^' + wf.workspace + '/install/lib/python2\.\d/(site|dist)-packages$', ws_pythonpath)


def test_python3_install():
    with workspace_factory() as wf:
        os.mkdir(os.path.join(wf.workspace, 'src'))
        shutil.copytree(
            os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'cmake_pkg'),
            os.path.join('src', 'cmake_pkg'))
        assert catkin_success(['config', '--merge-devel', '--merge-install', '--install'])
        assert catkin_success(['build', '-DPYTHON_VERSION=3'])
        pythonpaths = subprocess.check_output(
            ['bash', '-c', 'source ' + wf.workspace + '/install/setup.sh && echo $PYTHONPATH']).decode().split(':')
        ws_pythonpath = [p for p in pythonpaths if p.startswith(wf.workspace)][0]
        assert ws_pythonpath
        # it might be python3/dist-packages (debian) or python3.x/site-packages
        assert (ws_pythonpath == wf.workspace + '/install/lib/python3/dist-packages' or
                re.match('^' + wf.workspace + '/install/lib/python3\.\d/site-packages$', ws_pythonpath))
