import calendar

import time
from wpilib.driverstation import DriverStation 

#_ds_instance = ds.getInstance()
_t0 = calendar.timegm(time.gmtime())


def get_match_time():
    ds = DriverStation.getInstance()
    if ds.isFMSAttached():
        return ds.getMatchTime()
    else:
        return calendar.timegm(time.gmtime()) - _t0
