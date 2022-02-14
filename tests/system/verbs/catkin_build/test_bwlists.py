import os

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--yes']  # , '--no-notify', '--no-color', '--no-status']


def test_buildlist():
    """Test building buildlisted packages only"""
    pass  # TODO: Write test


def test_skiplist():
    """Test building all packages except skiplisted packages"""
    pass  # TODO: Write test


def test_skiplist_buildlist():
    """Test building with non-empty skiplist and buildlist"""
    pass  # TODO: Write test
