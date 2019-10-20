import math

import numpy as np

# This is a hacky motion planning utility.
# Given a starting position and velocity and an end position, it generates a motion composed of an acceleration, a constant velocity and a breaking part so that the derivation is continuous and velocity and acceleration are below given parameter values.
# See the plot module for how it looks like.

# "Sign function" that maps 0 to 1.
def sign(x):
    s = np.sign(x)
    if s == 0: s = 1
    return s

def calc_v_peak(v_start, a, s_delta):
    if sign(a) == sign(v_start):
        return sign(a) * math.sqrt(((v_start * v_start) / 2) + abs(a) * abs(s_delta))
    else:
        #print("Overshoot!")
        if v_start >= 0:
            return sign(a) * math.sqrt(-abs(a) * (s_delta - ((v_start * v_start) / (2 * abs(a)))))
        else:
            return math.sqrt(abs(a) * (s_delta + ((v_start * v_start) / (2 * abs(a)))))

# Returns (motion function, motion duration).
def plan(s_start, s_end, v_start, v_max, a):
    s_delta = s_end - s_start
    #print("s_delta = {}".format(s_delta))
    # Check which sign we need.
    if v_start / a > s_delta:
        #print("Negative acc / v_highest")
        a_acc = -a
        v_peak = calc_v_peak(v_start, a_acc, s_delta)
        v_highest = max(-v_max, v_peak)
    else:
        a_acc = a
        v_peak = calc_v_peak(v_start, a_acc, s_delta)
        v_highest = min(v_max, v_peak)
    a_break = -a_acc
    #print("v_highest = {}".format(v_highest))
    duration_acc = (v_highest - v_start) / a_acc
    duration_break = abs(v_highest / a)
    s_acc = v_start * duration_acc + ((duration_acc * duration_acc * a_acc) / 2)
    s_break = v_highest * duration_break + ((duration_break * duration_break * a_break) / 2)
    #print("s_acc = {}".format(s_acc))
    #print("s_break = {}".format(s_break))
    duration_const_non_peak = (s_delta - s_acc - s_break) / v_highest
    duration_const = max(0, duration_const_non_peak)
    duration_total = duration_acc + duration_const + duration_break
    print("duration_acc = {}".format(duration_acc))
    print("duration_const = {}".format(duration_const))
    print("duration_break = {}".format(duration_break))
    print("duration_total = {}".format(duration_total))
    t_const = duration_acc
    t_break = t_const + duration_const
    def s(t):
        def s_acc(t):
            return s_start + v_start * t + (math.pow(t, 2) * a_acc) / 2
        def s_const(t):
            return s_acc(t_const) + (t - t_const) * v_highest
        def s_break(t):
            #print("s_const(t_break) = {}".format(s_const(t_break)))
            return s_const(t_break) + v_highest * (t - t_break) + ((math.pow(t - t_break, 2) * a_break) / 2)

        if t <= t_const:
            return s_acc(t)
        elif t < t_break:
            return s_const(t)
        elif t <= duration_total:
            return s_break(t)
        elif t > duration_total:
            return s_end

    #print("v at the end of acc: {}".format((s(t_const) - s(t_const - 0.001)) / 0.001))
    # Sanity checks
    assert abs(s(duration_total) - s_end) < 0.0001
    assert abs(s(0) - s_start) < 0.0001
    return s, duration_total
