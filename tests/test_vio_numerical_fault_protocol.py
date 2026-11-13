import json
import socket
import struct
from egohand.vio.live_subprocess import report_error


def receive_error(error):
    a,b=socket.socketpair()
    try:
        report_error(a,error)
        n=struct.unpack('<I',b.recv(4))[0]
        return json.loads(b.recv(n))
    finally:
        a.close();b.close()


def test_numerical_fault_is_explicitly_recoverable_not_a_valid_pose():
    result=receive_error(RuntimeError('VIO_NUMERICAL_FAULT: pose rejected'))
    assert result['type']=='error'
    assert result['code']=='VIO_NUMERICAL_FAULT' and result['recoverable'] is True
    assert 'matrix' not in result and 'pose_valid' not in result


def test_unrelated_errors_are_not_retried_as_numerical_faults():
    result=receive_error(RuntimeError('settings unreadable'))
    assert result=={'type':'error','message':'settings unreadable'}
