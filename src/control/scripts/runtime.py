import time
# ROS
import rospy

class RuntimeMeasure(object):
    def __enter__(self):
        self.start_time = time.time()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        end_time = time.time()
        runtime = end_time - self.start_time
        self.statistics.add_measurement(runtime)

class RuntimeStatistics(object):
    def __init__(self):
        self.measurements = []

    def measure(self):
        measure = RuntimeMeasure()
        measure.statistics = self
        return measure

    def add_measurement(self, measurement):
        self.measurements.append(measurement)

    def calculate_stats(self):
        avg_runtime = sum(self.measurements) / len(self.measurements)
        max_runtime = max(self.measurements)
        min_runtime = min(self.measurements)

        rospy.loginfo("Average runtime: {:.2f} seconds".format(avg_runtime))
        rospy.loginfo("Maximum runtime: {:.2f} seconds".format(max_runtime))
        rospy.loginfo("Minimum runtime: {:.2f} seconds".format(min_runtime))