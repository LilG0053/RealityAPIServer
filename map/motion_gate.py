"""Odometry motion needed before running another scan match."""

import math
import numpy as np

TRANSLATION_THRESHOLD_M = 0.02
YAW_THRESHOLD_RAD = math.radians(5.0)


def below_motion_threshold(current_xy, current_yaw, previous_xy, previous_yaw):
    if previous_xy is None:
        return False
    yaw_delta = math.atan2(math.sin(current_yaw - previous_yaw),
                           math.cos(current_yaw - previous_yaw))
    return (np.linalg.norm(current_xy - previous_xy) <= TRANSLATION_THRESHOLD_M
            and abs(yaw_delta) <= YAW_THRESHOLD_RAD)
