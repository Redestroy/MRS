import math
import numpy as np

def HjuTransform(step_angle, step_distance, max_distance, max_angle, pixel_array):
    HjuArr = np.zeros([max_distance/step_distance+1,max_angle/step_distance+1])
    for d in range(0, max_distance, step_distance):
        for a in range(0, max_angle, step_angle):
            for p in pixel_array:
                if IsInLine(a, d, p.x, p.y):
                    HjuArr[d, a] += p.value
    return HjuArr


def IsInLine(angle, distance, x, y, discretion_coef = 1):
    return (x * math.cos(angle) + y * math.sin(angle) >= distance - discretion_coef) and (x * math.cos(angle) + y * math.sin(angle) <= distance - discretion_coef)
