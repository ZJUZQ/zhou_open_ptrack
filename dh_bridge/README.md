A ROS node for dh cameras. 

You can use the parameters.yaml file inside the config folder, to change the paramters to your needs.

Before catkin_make, remember to copy the following to CMakeLists.txt:

########################################################################

link_directories(/home/test/3rdparty/lib
         /home/test/3rdparty/bin/hc_lib
         /home/test/3rdparty/bin/dh_lib
         /home/test/3rdparty/bin/hc_bin/HCNetSDKCom  )
include_directories(/home/test/3rdparty/include
          /home/test/3rdparty/include/dh_include
          /home/test/3rdparty/include/hc_include )

########################################################################