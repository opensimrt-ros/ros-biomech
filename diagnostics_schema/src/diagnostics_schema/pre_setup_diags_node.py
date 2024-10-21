#!/usr/bin/env python3

import rospy
import pre_setup_diags
from pre_setup_diags import CheckOwnHost, PingHost, Sound, Video


rospy.init_node("pre_setup_tester")

tests = []

tests.append(CheckOwnHost())
tests.append(PingHost("router","192.168.1.1"))
tests.append(PingHost("myself","192.168.1.100", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(PingHost("tablet","192.168.1.101"))
tests.append(PingHost("vicon pc dongle","192.168.1.102", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(PingHost("vicon pc","192.168.1.103", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Sound("/srv/data/calib.wav"))
tests.append(Video("/dev/video0", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video1", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video2", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video3", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video4", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video5", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video6", pre_setup_diags.OPTIONAL_REQUIREMENT))

pre_setup_diags.do(tests)

