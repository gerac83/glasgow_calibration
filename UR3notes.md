# Changes for UR3

## File: calibration_glasgow/include/parameters.h

* Line 31: Robot specific comment. 

24 static const char CAM_SUB[] = "/Image_for_Calibration";
25 static const char CAMERA_INFO[] = "/Cam_info_for_Calibration";
26 static const char TRANFORM_SUB[] = "/transform_GrippertoBase";

## File: calibration_glasgow/launch/frame_definition.launch

* Line 4: The `camera_rgb_optical_frame` should be the same since we are using the Zed camera, however `torso` will be different; change this to `base_link` or the frame that is at `[0 0 0]`
* **NOTE:** This launch file is used after we have done the calibration

## twodto3d/src/transform_pub.cpp

* Line 56 and 86: The frame defined for the `/cuff_status` msg is irrelevant and can have any value, so no need to change. For future reference, this node can be removed and have a service advertised in `twodto3d.cpp` if using for UR3 (not for Baxter), and calling the service from the terminal.

## twodto3d/src/twodto3d.cpp

* Lines 70 and 71: Change to the frame that is at `[0 0 0]` (i.e. `base_link`) and the gripper frame (the frame you use to move the robot using MoveIt!)
* Lines 167 and 168: Image and camera info topics. Since we are still using the ZED camera, no need to change but leaving note for future reference