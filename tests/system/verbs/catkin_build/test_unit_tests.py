from __future__ import print_function

import os
import shutil

from ....utils import in_temporary_directory
from ....utils import redirected_stdio

from catkin_tools.commands.catkin import main as catkin_main

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

def catkin_success(args, env={}):
    orig_environ = dict(os.environ)
    try:
        os.environ.update(env)
        catkin_main(args)
    except SystemExit as exc:
        ret = exc.code
        if ret != 0:
            import traceback
            traceback.print_exc()
    finally:
        os.environ = orig_environ
    return ret == 0

@in_temporary_directory
def test_ctest_merged():
    """Test ctest-based tests with a merged develspace"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    assert os.path.exists(os.path.join(RESOURCES_DIR, 'cmake_pkgs'))
    shutil.copytree(os.path.join(RESOURCES_DIR, 'cmake_pkgs'), source_space)
    print(cwd)
    assert os.path.exists(os.path.join(source_space, 'test_pkg'))

    with redirected_stdio() as (out, err):
        assert catkin_success(
            ['build', '--no-notify', '--no-status', '--verbose', 'test_pkg'])
        assert catkin_success(
            ['build', '--no-notify', '--no-status', '--verbose', '--no-deps',
             'test_pkg', '--make-args', 'test', 'ARGS="-V"'])
