cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()
rosbuild_add_rostest(test/test-add-two-ints.launch)
rosbuild_add_rostest(test/test-peer-subscribe-notify.launch)
rosbuild_add_rostest(test/test-add-two-ints-with-roscpp-server.launch)
rosbuild_add_rostest(test/test-talker-listener.launch)
rosbuild_add_rostest(test/test-talker-listener-with-roscpp.launch)
rosbuild_add_rostest(test/test-on-shutdown.launch)
rosbuild_add_rostest(test/test-connection-header.launch)

