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
## I am maybe being a bit pedantic here, but these extra devices are not useless specially since it appears they provide better timestamping information, which we may want
## source https://unix.stackexchange.com/questions/512759/multiple-dev-video-for-one-physical-device
## more info https://linuxtv.org/downloads/v4l-dvb-apis/userspace-api/v4l/dev-meta.html and here: https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/commit/?id=088ead25524583e2200aa99111bea2f66a86545a
tests.append(Video("/dev/video1", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video2", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video3", pre_setup_diags.OPTIONAL_REQUIREMENT))

tests.append(Video("/dev/video4", pre_setup_diags.OPTIONAL_REQUIREMENT)) ## this is the main device
tests.append(Video("/dev/video5", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video6", pre_setup_diags.OPTIONAL_REQUIREMENT))
tests.append(Video("/dev/video7", pre_setup_diags.OPTIONAL_REQUIREMENT))

pre_setup_diags.do(tests)

