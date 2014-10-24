import os
import shutil
import mock

from catkin_tools import config

from nose.tools import assert_raises_regexp
from ..utils import in_temporary_directory
from ..utils import redirected_stdio


@mock.patch('catkin_tools.config.initialize_verb_aliases')
@in_temporary_directory
def test_config_initialization(patched_func):
    cwd = os.getcwd()
    test_folder = os.path.join(cwd, 'test')
    # Test normal case
    config.initialize_config(test_folder)
    assert os.path.isdir(test_folder)
    assert not os.path.exists(os.path.join(test_folder, 'verb_aliases'))
    # Assert a second invocation is fine
    config.initialize_config(test_folder)
    shutil.rmtree(test_folder)
    # Test failure with file for target config path
    with open(test_folder, 'w') as f:
        f.write('this will cause a RuntimeError')
    with assert_raises_regexp(RuntimeError, "The catkin config directory"):
        config.initialize_config(test_folder)


@in_temporary_directory
def test_verb_alias_config_initialization():
    cwd = os.getcwd()
    test_folder = os.path.join(cwd, 'test')
    # Test target directory does not exist failure
    with assert_raises_regexp(RuntimeError, "Cannot initialize verb aliases because catkin configuration path"):
        config.initialize_verb_aliases(test_folder)
    # Test normal case
    os.makedirs(test_folder)
    config.initialize_verb_aliases(test_folder)
    assert os.path.isdir(test_folder)
    assert os.path.isdir(os.path.join(test_folder, 'verb_aliases'))
    defaults_path = os.path.join(test_folder, 'verb_aliases', '00-default-aliases.yaml')
    assert os.path.isfile(defaults_path)
    # Assert a second invocation is fine
    config.initialize_verb_aliases(test_folder)
    # Check that replacement of defaults works
    with open(defaults_path, 'w') as f:
        f.write("This should be overwritten (simulation of update needed)")
    with redirected_stdio() as (out, err):
        config.initialize_verb_aliases(test_folder)
    assert "Warning, builtin verb aliases at" in out.getvalue(), out.getvalue()
    shutil.rmtree(test_folder)
    # Check failure from verb aliases folder existing as a file
    os.makedirs(test_folder)
    with open(os.path.join(test_folder, 'verb_aliases'), 'w') as f:
        f.write("this will cause a RuntimeError")
    with assert_raises_regexp(RuntimeError, "The catkin verb aliases config directory"):
        config.initialize_verb_aliases(test_folder)
    shutil.rmtree(test_folder)


@in_temporary_directory
def test_get_verb_aliases():
    cwd = os.getcwd()
    test_folder = os.path.join(cwd, 'test')
    # Test failure case where config folder does not exist
    with assert_raises_regexp(RuntimeError, "Cannot get verb aliases because the catkin config path"):
        config.get_verb_aliases(test_folder)
    # Test failure case where aliases folder does not exist
    with mock.patch('catkin_tools.config.initialize_verb_aliases'):
        config.initialize_config(test_folder)
    with assert_raises_regexp(RuntimeError, "Cannot get verb aliases because the verb aliases config path"):
        config.get_verb_aliases(test_folder)
    shutil.rmtree(test_folder)
    # Test the normal case
    config.initialize_config(test_folder)
    aliases = config.get_verb_aliases(test_folder)
    assert 'b' in aliases
    assert aliases['b'] == 'build'
    # Test a custom file
    base_path = os.path.join(test_folder, 'verb_aliases')
    with open(os.path.join(base_path, '01-my-custom-aliases.yaml'), 'w') as f:
        f.write("""\
b: build --isolate-devel
ls: null
""")
    aliases = config.get_verb_aliases(test_folder)
    assert 'b' in aliases
    assert aliases['b'] == 'build --isolate-devel', aliases['b']
    assert 'ls' not in aliases
    # Test a bad alias files
    bad_path = os.path.join(base_path, '02-bad.yaml')
    with open(bad_path, 'w') as f:
        f.write("""\
- foo
- bar
""")
    with assert_raises_regexp(RuntimeError, "Invalid alias file"):
        config.get_verb_aliases(test_folder)
    os.remove(bad_path)
    with open(bad_path, 'w') as f:
        f.write("""\
null: foo
""")
    with assert_raises_regexp(RuntimeError, "Invalid alias in file"):
        config.get_verb_aliases(test_folder)
    os.remove(bad_path)
    with open(bad_path, 'w') as f:
        f.write("""\
foo: 13.4
""")
    with assert_raises_regexp(RuntimeError, "Invalid alias expansion in file"):
        config.get_verb_aliases(test_folder)
    os.remove(bad_path)
    # Test with an empty custom file
    empty_path = os.path.join(base_path, '02-my-empty.yaml')
    with open(empty_path, 'a') as f:
        os.utime(empty_path, None)
    aliases = config.get_verb_aliases(test_folder)
