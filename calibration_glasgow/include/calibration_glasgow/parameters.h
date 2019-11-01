
#ifndef PARAMETERS_H_
#define PARAMETERS_H_

static const char WINDOW_LEFT[] = "RGB camera image";

// Parameter server
static const char CALIB_TARGET[] = "/glasgow_calibration/target";
static const char WIDTH_MARKER[] = "/glasgow_calibration/marker_width";
static const char MARKER_SIZE_X[] = "/glasgow_calibration/marker_size_x";
static const char MARKER_SIZE_Y[] = "/glasgow_calibration/marker_size_y";
static const char MAX_ERROR_TH[] = "/glasgow_calibration/max_error";

static const char OUTPUT_IMAGE_DIR[] = "/glasgow_calibration/outputImageDir";
static const char OUTPUT_CALIB_DIR[] = "/glasgow_calibration/outputCalibDir";

static const char SAVE_MODE[] = "/glasgow_calibration/save_mode";
static const char INPUT_SCALE[] = "/glasgow_calibration/resize_imgs_factor";

static const char HE_CALIB_FILE_URL[] = "/glasgow_calibration/gHc_calibration_file";
static const char DEBUGQ[] = "/glasgow_calibration/debug";

// Messages
static const char CAM_SUB[] = "/Image_for_Calibration";
static const char CAMERA_INFO[] = "/Cam_info_for_Calibration";
static const char TRANFORM_SUB[] = "/transform_GrippertoBase";

//Services
static const char HE_CALIB[] = "/glasgow_calibration/HandEyeCalibration";

//Robot specific
static const char BASE_LINK[] = "/base_link";
static const char CAMERA_FRAME[] = "/zed_left_camera_optical_frame";

#endif
