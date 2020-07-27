import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


# Standard deviation of pH changes. (process variation)
# I think this is per unit used for dt. We use seconds.
PH_STD = 0.1


def create(dt: float) -> KalmanFilter:
    """Initialize the kalman filter. dt is the time between measurements, in seconds."""
    filter_ = KalmanFilter(dim_x=2, dim_z=1)

    # The initialization value doesn't matter much, since the filter
    # will correct quickly. Initialize to 7 pH, and no change in pH over time.
    filter_.x = np.array([[7.0], [0.0]])  # initial state (pH, and dpH_dt)

    filter_.F = np.array([[1.0, 1.0], [0.0, 1.0]])  # state transition matrix

    filter_.H = np.array([[1.0, 0.0]])  # Measurement function
    # Initialization of the covariance matrix is based on how close our
    # initialization is. Since we always initialize to 0.7, let's assume
    # most pH measurements will be within a few pH units of it.
    filter_.P *= 3.0  # covariance matrix

    # High values of state uncertainty mean more smoothing. (Similar to low noise
    # variance)
    filter_.R = 0.01  # state uncertainty, ie measurement noise

    # Process uncertainly variance appears to have a large effect on both
    # the response time, and stability of the result. Smaller values means slower
    # response, more overshooting, and more smoothing. Dt is related to this.
    filter_.Q = Q_discrete_white_noise(2, dt=dt, var=PH_STD ** 2)  # process uncertainty

    return filter_
