from catkin_tools.jobs.utils import merge_envs


def test_merge_envs_basic():
    job_env = { 'PATH': '/usr/local/bin:/usr/bin', 'FOO': 'foo' }

    merge_envs(job_env, [
        { 'PATH': '/usr/local/bin:/bar/baz/bin' },
        { 'BAR': 'bar' } ])

    # Validate that the known path was not moved from the existing order, and the unfamiliar
    # path was correctly prepended.
    assert job_env['PATH'] == '/bar/baz/bin:/usr/local/bin:/usr/bin'

    # Confirm that a key only in the original env persists.
    assert job_env['FOO'] == 'foo'

    # Confirm that a new key is added.
    assert job_env['BAR'] == 'bar'


def test_merge_envs_complex():
    ''' Confirm that merged paths are deduplicated and that order is maintained. '''
    job_env = { 'PATH': 'C:B:A' }
    merge_envs(job_env, [{ 'PATH': 'D:C' }, { 'PATH': 'E:A:C' }])
    assert job_env['PATH'] == 'E:D:C:B:A', job_env['PATH']


def test_merge_envs_nonpaths():
    ''' Confirm that non-path vars are simply overwritten on a last-wins policy. '''
    job_env = { 'FOO': 'foo:bar' }
    merge_envs(job_env, [{ 'FOO': 'bar:baz' }, { 'FOO': 'baz:bar' }])
    assert job_env['FOO'] == 'baz:bar'
