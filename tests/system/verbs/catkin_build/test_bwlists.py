import os

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--yes']  # , '--no-notify', '--no-color', '--no-status']


def test_whitelist():
    """Test building whitelisted packages only"""
    pass  # TODO: Write test


def test_blacklist():
    """Test building all packages except blacklisted packages"""
    pass  # TODO: Write test


def test_blacklist_whitelist():
    """Test building with non-empty blacklist and whitelist"""
    pass  # TODO: Write test
