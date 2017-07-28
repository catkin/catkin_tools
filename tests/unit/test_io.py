try:
    # Python3
    from queue import Queue
except ImportError:
    # Python2
    from Queue import Queue

from catkin_tools.execution import io as io


def test_invalid_utf8_characters():
    io_container = io.IOBufferContainer('test', 'test_id', 'test', Queue(), '/tmp')
    bad_byte_array = (128).to_bytes(1, byteorder='big')
    try:
        io_container._decode(bad_byte_array)
    except UnicodeDecodeError:
        assert False, "Bad decoding"
