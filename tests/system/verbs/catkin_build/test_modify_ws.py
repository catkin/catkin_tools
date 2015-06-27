
from __future__ import print_function

import os
import shutil

from math import floor

from ...workspace_factory import workspace_factory

from ....utils import in_temporary_directory
from ....utils import assert_cmd_success
from ....utils import assert_cmd_failure
from ....utils import assert_files_exist
from ....utils import catkin_success
from ....utils import catkin_failure
from ....utils import redirected_stdio


from ....workspace_assertions import assert_workspace_initialized
from ....workspace_assertions import assert_no_warnings

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--force']  # , '--no-notify', '--no-color', '--no-status']


def test_add_package():
    """Test build behavior when adding packages to the workspace"""
    pass  # TODO: Implement this for various dependency relationships


def test_remove_package():
    """Test build behavior when removing packages from the workspace"""
    pass  # TODO: Implement this for various dependency relationships


def test_rename_package():
    """Test build behavior when renaming a package in the workspace"""
    pass  # TODO: Implement this for various dependency relationships


def test_ignore_package():
    """Test build behavior when adding a CATKIN_IGNORE file to a package in the workspace"""
    pass  # TODO: Implement this for various dependency relationships


def test_deblacklist():
    """Test build behavior when removing a package from the blacklist that has yet to be built"""
    pass  # TODO: Implement this for various dependency relationships
