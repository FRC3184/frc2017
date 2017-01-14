from ctypes import *

cdll.LoadLibrary("libpathfinder_arm.so")  # TODO Make sure neon_pathfinder is in PATH and library file is copied

neon_pathfinder = CDLL("libpathfinder_arm.so")
print(neon_pathfinder.__dict__)
# This seems to work, but need a RIO to test
