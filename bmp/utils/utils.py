import sys, os
from contextlib import contextmanager

@contextmanager
def no_output():
    sys.stdout.flush()
    _origstdout = sys.stdout
    _oldstdout_fno = os.dup(sys.stdout.fileno())
    _devnull = os.open(os.devnull, os.O_WRONLY)
    _newstdout = os.dup(1)
    os.dup2(_devnull, 1)
    os.close(_devnull)
    sys.stdout = os.fdopen(_newstdout, 'w')
    yield
    sys.stdout.close()
    sys.stdout = _origstdout
    sys.stdout.flush()
    os.dup2(_oldstdout_fno, 1)
    os.close(_oldstdout_fno)  # Added
