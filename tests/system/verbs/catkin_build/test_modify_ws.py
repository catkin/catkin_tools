import os

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--yes']  # , '--no-notify', '--no-color', '--no-status']


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
