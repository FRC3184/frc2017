import calendar

import time
from wpilib.driverstation import DriverStation as ds

_ds_instance = ds.getInstance()
_t0 = calendar.timegm(time.gmtime())


def get_match_time():
    if ds.isFMSAttached(_ds_instance):
        return ds.getMatchTime(_ds_instance)
    else:
        return calendar.timegm(time.gmtime()) - _t0
