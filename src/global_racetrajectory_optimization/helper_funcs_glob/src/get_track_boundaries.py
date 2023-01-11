
import numpy as np
import helper_funcs_glob

def get_track_boundaries(reftrack: np.ndarray,
               reftrack_normvec_normalized: np.ndarray) -> tuple:
    # calculate boundaries and interpolate them to small stepsizes (currently linear interpolation)
    bound_r = reftrack[:, :2] + reftrack_normvec_normalized * np.expand_dims(reftrack[:, 2], 1)
    bound_l = reftrack[:, :2] - reftrack_normvec_normalized * np.expand_dims(reftrack[:, 3], 1)

    return bound_r, bound_l