from wpilib.driverstation import DriverStation as ds

_ds_instance = ds.getInstance()


def get_match_time():
    if ds.isFMSAttached(_ds_instance):
        return ds.getMatchTime(_ds_instance)
    else:
        return 0  # todo
