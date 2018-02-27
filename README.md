# Baxter---Garment-Manipulation

**installing Garment-Manipulation**

$ cd ~/catkin_ws/src

$ git clone https://github.com/vigneshrajaponnambalam/Baxter---Garment-Manipulation

$ catkin_make

**running the various scenarios**

various scenarios tested (with different position of cloth like folded, bended etc..) 

Get the information about cloth detected from Baxter camera and processing done

change the processing behavior by modifying the cloth position manually.

**provided nodes**

image_listener: a node which detects the grasping points of cloths by baxter arm

            subscribed topics: camera rgb image ("/camera/rgb/image_raw") 
                               camera depth image ("/camera/depth/image_raw")
                               camera info topic ("/camera/rgb/camera_info")
                               control state ("/state_change")
                               
            published topics: publisher pose - display the global pose for corners detected
                                (/leftgoal)
                                (/rightgoal)
                              publisher axis - display the corner points in x and y axis locally
                                (/leftaxis)
                                (/rightaxis)


