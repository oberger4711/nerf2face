import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from motion_plan import plan

PLOT_RESOLUTION = 0.001

fig, ax = plt.subplots()

# Working:
parameter_sets_to_plot = [
        #s_start, s_end, v_start, v_max, a
        #[0.0,     2.0,   0.0,     1,     1],
        #[0.0,     2.0,   0.0,     2,     4],
        #[1.0,     -2.0,   -1.0,     2,     4],
        #[0.0,     1.0,   0.5,     1,     1],
        #[0.0,     1.0,   0.0,     1,     2],
        #[0.0,     0.5,   0.0,     1,     2],
        #[0.0,     0.5,  1.0,     1,     1], # Break only.
        #[0.0,     -0.1,  -1,      1,     1],
        #[0.0,     0.1,  1,       1,     1],
        #[0.0,      0.2,   1,       1,     1], # Overshoots.
        #[0.0,      -1.0,   0,       1,     1], # Falling motion.
        [1.0,      0.2,   1,       1,     1],
    ]


for parameter_set in parameter_sets_to_plot:
    print()
    print("s_start = {}, s_end = {}, v_start = {}, v_max = {}, a = {}".format(*parameter_set))
    s, duration = plan(*parameter_set)
    ts = np.arange(0, duration + PLOT_RESOLUTION, PLOT_RESOLUTION)
    s_ts = np.array([s(t) for t in ts])
    ax.plot(ts, s_ts)

ax.set(xlabel="t (s)", ylabel="s", title="Planned Motion")
ax.grid()

plt.show()
