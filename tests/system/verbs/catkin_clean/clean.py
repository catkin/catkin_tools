# For reasons unknown to me, this file cannot be named test_clean.py because the CI will fail.
import os
import shutil

from ...workspace_factory import workspace_factory
from ....utils import catkin_success
from ....utils import redirected_stdio

RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']


def test_clean_devel():
    """Test if files in the devel space get cleaned correctly"""
    with redirected_stdio():
        with workspace_factory() as wf:
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'products_0'),
                os.path.join('src/products_0'))
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_pkg'),
                os.path.join('src/python_pkg'))

            assert catkin_success(['config', '--link-devel'])
            assert catkin_success(BUILD)
            assert os.path.exists(os.path.join(wf.workspace, 'devel', 'lib', 'products_0'))
            assert os.path.exists(os.path.join(wf.workspace, 'devel', 'lib', 'libproducts_0_lib.so'))
            assert os.path.exists(os.path.join(wf.workspace, 'devel', 'lib', 'python3',
                                               'dist-packages', 'python_pkg'))

            assert catkin_success(['clean', 'products_0'])
            assert not os.path.exists(os.path.join(wf.workspace, 'devel', 'lib', 'products_0'))
            assert not os.path.exists(os.path.join(wf.workspace, 'devel', 'lib', 'libproducts_0_lib.so'))
            assert os.path.exists(os.path.join(wf.workspace, 'devel', 'lib', 'python3',
                                               'dist-packages', 'python_pkg'))

            assert catkin_success(['clean', 'python_pkg'])
            assert not os.path.exists(os.path.join(wf.workspace, 'devel', 'lib', 'python3',
                                                   'dist-packages', 'python_pkg'))
            assert os.path.exists(os.path.join(wf.workspace, 'devel', '_setup_util.py'))


def test_clean_install():
    """Test if installed files get cleaned correctly"""
    with redirected_stdio():
        with workspace_factory() as wf:
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'products_0'),
                os.path.join('src/products_0'))
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_pkg'),
                os.path.join('src/python_pkg'))

            assert catkin_success(['config', '--install'])
            assert catkin_success(BUILD)
            assert os.path.exists(os.path.join(wf.workspace, 'install', 'lib', 'products_0'))
            assert os.path.exists(os.path.join(wf.workspace, 'install', 'lib', 'libproducts_0_lib.so'))
            assert os.path.exists(os.path.join(wf.workspace, 'install', 'lib', 'python3', 'dist-packages',
                                               'python_pkg', 'lib.py'))

            assert catkin_success(['clean', 'products_0'])
            assert not os.path.exists(os.path.join(wf.workspace, 'install', 'lib', 'products_0'))
            assert not os.path.exists(os.path.join(wf.workspace, 'install', 'lib', 'libproducts_0_lib.so'))
            assert os.path.exists(os.path.join(wf.workspace, 'install', 'lib', 'python3', 'dist-packages',
                                               'python_pkg', 'lib.py'))

            assert catkin_success(['clean', 'python_pkg'])
            assert not os.path.exists(os.path.join(wf.workspace, 'install', 'lib', 'python3', 'dist-packages',
                                                   'python_pkg', 'lib.py'))
            assert os.path.exists(os.path.join(wf.workspace, 'install', '_setup_util.py'))
