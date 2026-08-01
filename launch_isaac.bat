@echo off
cd /d C:\isaacsim
set ROS_DOMAIN_ID=0
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=C:\isaac_project\fastdds.xml
call isaac-sim.bat --exec "C:\isaac_project\bootstrap.py"
