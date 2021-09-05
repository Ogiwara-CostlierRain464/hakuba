#! /usr/bin/env python
# 1. enter to venv shell.  `source venv/bin/activate`
# 2. run node.  `rosrun hakuba test.py`
# see: https://qiita.com/tnjz3/items/4d64fc2d36b75e604ab1

import rospy
import sys
import numpy as np
import matrixprofile as mp
from matplotlib import pyplot as plt
from std_msgs.msg import String


def ECG():
    ecg = mp.datasets.load('ecg-heartbeat-av')
    ts = ecg['data']
    window_size = 150
    profile = mp.compute(ts, windows=window_size)
    profile = mp.discover.motifs(profile, k=1)
    threshold = 1200
    av = \
        np.append(np.zeros(threshold),
                  np.ones(len(profile['mp']) -
                          threshold))

    profile = mp.transform.apply_av(profile, "custom", av)
    profile = mp.discover.motifs(profile, k=3, use_cmp=True, exclusion_zone=100)
    figures = mp.visualize(profile)
    figures[3].show()


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == "__main__":
    print(sys.version)
    ECG()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


# You have to impl Martix Profile in C++
# Only utilize CPU power, not GPU!
# At first, you have to check
# - Matrix XIV (Matrix Profile with GPU) https://dl.acm.org/doi/pdf/10.1145/3357223.3362721
# - Check if this paper exists. https://mediatum.ub.tum.de/doc/1471292/1471292.pdf

