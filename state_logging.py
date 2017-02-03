import json


_logfile = None
_log_objs = {}
_log_path = "~/{}"


def init_log(id):
    global _logfile
    _logfile = open(_log_path.format(id), "w", 0)


def close_log():
    _logfile.close()


def log(name, callback):
    if not callable(callback):
        raise ValueError("callback must be callable")
    _log_objs[name] = callback


def do_log():
    if _logfile is None or _logfile.closed:
        raise IOError("logfile is not ready for writing")

    state = {}
    for name, callback in _log_objs.items():
        state[name] = callback()

    json.dump(state, _logfile)
    _logfile.flush()
