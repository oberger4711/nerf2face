PLOT = False

import nose
if PLOT:
    import matplotlib
    import matplotlib.pyplot as plt
import numpy as np

from motion_plan import plan

RESOLUTION = 0.001
EPSILON = 0.005

if PLOT: _, ax = plt.subplots()

def setup():
    pass

def teardown_plot():
    if PLOT:
        ax.set(xlabel="t (s)", ylabel="s", title="Planned Motion")
        ax.grid()
        plt.show()

@nose.with_setup(setup, teardown_plot)
def test_plan():
    parameter_sets_to_test = [
            #s_start, s_end, v_start, v_max, a_max
            [0.0,     0.0,   0.0,     1,     1],
            [0.0,     0.0,   1.0,     1,     1],
            [0.0,     2.0,   0.0,     1,     1],
            [0.0,     2.0,   0.0,     2,     4],
            [1.0,     2.0,   0.1,     2,     4],
            [1.0,    -2.0,  1.0,     2,     4],
            [1.0,    -2.0,  -1.0,     2,     4],
            [-2.0,    2.0,   0.0,     1,     1],
            [-2.0,   -0.5,   0.0,     1,     1],
            [0.0,     1.0,   0.5,     1,     1],
            [0.0,     1.0,   0.0,     1,     2],
            [0.0,     0.5,   0.0,     1,     2],
            [0.0,     0.5,   1.0,     1,     1], # Break only.
            [0.0,    -0.1,  -1,       1,     1],
            [0.0,     0.1,   1,       1,     1],
            [0.0,     0.2,   1,       1,     1], # Overshoots.
            [0.0,    -1.0,   0,       1,     1], # Falling motion.
            [1.0,     0.2,   1,       1,     1],
        ]

    for parameter_set in parameter_sets_to_test:
        print()
        print("s_start = {}, s_end = {}, v_start = {}, v_max = {}, a_max = {}".format(*parameter_set))
        s, duration = plan(*parameter_set)
        print("duration = {}".format(duration))
        ts = np.arange(0, duration + RESOLUTION, RESOLUTION)
        s_ts = np.array([s(t) for t in ts])

        s_start, s_end, v_start, v_max, a_max = parameter_set
        # Check planned motion.
        assert abs(s(0) - s_start) < EPSILON
        assert abs(s(duration) - s_end) < EPSILON
        # Check that velocity is continuous (no jumps).
        v_before = v_start
        for i in range(int((duration + RESOLUTION) // RESOLUTION)):
            if i == 0: continue
            # Check derivations using difference quotient approximation.
            v_current = (s_ts[i] - s_ts[i - 1]) / RESOLUTION
            assert abs(v_current) <= v_max + EPSILON
            if i == 1: assert abs(v_current - v_start) < EPSILON
            a_current = (v_current - v_before) / RESOLUTION
            assert abs(a_current) <= a_max + EPSILON
            v_before = v_current

        if PLOT: ax.plot(ts, s_ts)

nose.main()
