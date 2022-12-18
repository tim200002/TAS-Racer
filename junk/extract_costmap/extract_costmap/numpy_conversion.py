from array import array as Array

from nav_msgs.msg import OccupancyGrid, MapMetaData

import numpy as np
from numpy.lib.stride_tricks import as_strided
from nav_msgs.msg import OccupancyGrid


def occupancygrid_to_numpy(msg: OccupancyGrid):
    data = \
      np.asarray(msg.data,
                 dtype=np.int8).reshape(msg.info.height, msg.info.width)

    return np.ma.array(data, mask=data==-1, fill_value=-1)


def numpy_to_occupancy_grid(arr, info=None):
    if not len(arr.shape) == 2:
        raise TypeError('Array must be 2D')
    if not arr.dtype == np.int8:
        raise TypeError('Array must be of int8s')

    grid = OccupancyGrid()
    if isinstance(arr, np.ma.MaskedArray):
        # We assume that the masked value are already -1, for speed
        arr = arr.data

    grid.data = Array('b', arr.ravel().astype(np.int8))
    grid.info = info or MapMetaData()
    grid.info.height = arr.shape[0]
    grid.info.width = arr.shape[1]

    return grid