//
//  calibration_services.cpp
//  Hand-eye calibration for Baxter without knowledge of the target position
//  Based on the CloPeMa robot head calibration routines
//
//  Created by Gerardo Aragon on 09/2016.
//  Copyright (c) 2016 Gerardo Aragon. All rights reserved.

#include <calibration_glasgow/calibration_services.h>
#include <calibration_glasgow/parameters.h>
#include <calibration_glasgow/camera_calibration.h>
#include <calibration_glasgow/handeye_calibration.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

enum SaveOp {nothing, images, all};

class CmainCalibration
{
public:

    Calibration c_;
    Chandeye hE_robot; // To find the transformation between calibration target and gripper
    Chandeye hE_camera; // To find the transformation between camera and robot
    tf::TransformListener tf_;

    vector<vector<Point2f> > imagePoints;
    vector<vector<Point2f> > points_handEye;
    vector<Mat> rvecs_cam, tvecs_cam;
    vector<Mat> rvecs_rb2gripper, tvecs_rb2gripper;

    //string ROBOT_GRIPPER;
    string GRIPPER_LINK;
    string ROBOT_BASE;

    Mat cameraMatrix, distCoeffs;
    Mat imLeft, cornersImgLeft;
    bool found_chees;

    cv_bridge::CvImagePtr cv_ptr;
    // pcl::PointCloud<pcl::PointXYZ> cloud_pcl;

    tf::Quaternion transform_rotation;
    tf::Vector3 transform_translation;

    ros::ServiceServer process_target_srv_;
    ros::ServiceServer he_calib_srv_;

    bool debugQ;
    int noImgPtu;
    vector<Mat> bigMatQ, bigMatE;

    CmainCalibration() : it_(nh_),
        im_sub_(it_, CAM_SUB, 5),
        cam_info_sub_(nh_, CAMERA_INFO, 5),
        trans_sub_(nh_, TRANFORM_SUB, 5),
        sync(syncPolicy(5), im_sub_, cam_info_sub_, trans_sub_)
    {
        c_.init();

        double param;

        ros::Duration(10.0).sleep();
        ROS_INFO("Initialising calibration node!!");

        if(nh_.hasParam(CALIB_TARGET))
        {
            nh_.getParam(CALIB_TARGET, c_.calibTarget);
        }else{
            ROS_ERROR("\"%s\" parameter is not set in the server", CALIB_TARGET);
        }

        if(nh_.hasParam(OUTPUT_IMAGE_DIR) && nh_.hasParam(OUTPUT_CALIB_DIR) && nh_.hasParam(MAX_ERROR_TH))
        {
            nh_.getParam(OUTPUT_IMAGE_DIR, c_.imageOutputDir);
            nh_.getParam(OUTPUT_CALIB_DIR, c_.calibOutputDir);
            double temp;
            nh_.getParam(MAX_ERROR_TH, temp);
            c_.maxError = (float)temp;

        }else{
            ROS_ERROR("\"%s\", \"%s\" and/or \"%s\" parameters are not set in the server", OUTPUT_IMAGE_DIR, OUTPUT_CALIB_DIR, MAX_ERROR_TH);
            nh_.shutdown();
            return;
        }

        if(nh_.hasParam(WIDTH_MARKER) && nh_.hasParam(MARKER_SIZE_X) && nh_.hasParam(MARKER_SIZE_Y) && nh_.hasParam(CALIB_TARGET)
                && nh_.hasParam(SAVE_MODE) && nh_.hasParam(INPUT_SCALE))
        {
            nh_.getParam(WIDTH_MARKER,param);
            c_.squareSize = (float)param;
            nh_.getParam(MARKER_SIZE_X,param);
            c_.boardSize.width = (int)param;
            nh_.getParam(MARKER_SIZE_Y,param);
            c_.boardSize.height = (int)param;
            nh_.getParam(CALIB_TARGET, c_.calibTarget);
            nh_.getParam(SAVE_MODE,param);
            c_.save_mode = (int)param;
            nh_.getParam(INPUT_SCALE,param);
            c_.scaleInput = (float)param;
            nh_.getParam(HE_CALIB_FILE_URL, c_.he_calib_url);
        }else{
            ROS_ERROR("Check that \"%s\", \"%s\", \"%s\", \"%s\", \"%s\", \"%s\" and/or \"%s\" parameters are set in the server", WIDTH_MARKER, MARKER_SIZE_X, MARKER_SIZE_Y, CALIB_TARGET, SAVE_MODE, INPUT_SCALE, HE_CALIB_FILE_URL);
        }

        c_.nrFrames = 1;
        
        string cmd = "exec rm -r " + c_.calibOutputDir + "*.xml";
        system(cmd.c_str());
        cmd = "exec rm -r " + c_.imageOutputDir + "*.tif";
        system(cmd.c_str());
        cmd = "exec rm -r " + c_.calibOutputDir + "*.py";
        system(cmd.c_str());
        cmd = "exec mkdir -p " + c_.imageOutputDir;
        system(cmd.c_str());
        // ***********************

        rvecs_rb2gripper.resize(0);
        tvecs_rb2gripper.resize(0);

        imagePoints.resize(0);

        bigMatQ.resize(0);
        bigMatE.resize(0);

        cv::namedWindow(WINDOW_LEFT, CV_WINDOW_NORMAL);
        cv::resizeWindow(WINDOW_LEFT, 640, 480);
        cvMoveWindow(WINDOW_LEFT, 10, 10);

        // Service calls
        he_calib_srv_ = nh_.advertiseService(HE_CALIB, &CmainCalibration::HandEyeCalibrationSrv, this);

        //capture_images();
        cv::startWindowThread();

        ROS_INFO("Node initialised...");

        sync.registerCallback(boost::bind(&CmainCalibration::mainRoutine, this, _1, _2, _3));
    }

    ~CmainCalibration()
    {
        ROS_INFO("Bye!");
    }

    // Callback that stores images in memory
    void mainRoutine(const sensor_msgs::ImageConstPtr& imIn, const sensor_msgs::CameraInfoConstPtr& cam_info, const geometry_msgs::TransformStampedConstPtr& msg_trans)
    {
        ROS_INFO("Reading messages!");
        // Get images
        try
        {
            cv_ptr = cv_bridge::toCvCopy(imIn, enc::BGR8);
            
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imIn->encoding.c_str());
            return;
        }

        // pcl::fromROSMsg(*pcIn, cloud_pcl);

        imLeft = cv_ptr->image;
        cameraMatrix = getCameraInfo(cam_info);
        distCoeffs = cv::Mat(5,1,CV_64FC1, cv::Scalar::all(0));

        ROBOT_BASE = msg_trans->header.frame_id;
        GRIPPER_LINK = msg_trans->child_frame_id;
        tf::quaternionMsgToTF(msg_trans->transform.rotation, transform_rotation);
        tf::vector3MsgToTF(msg_trans->transform.translation, transform_translation);

        // Display images
        display_image(imLeft);

        processTarget();
        
        return;

    }

    bool HandEyeCalibrationSrv(calibration_glasgow::HandEyeCalibration::Request& req, calibration_glasgow::HandEyeCalibration::Response& rsp)
    {
        int totalFrames = 8;
        if(req.doIt == true)
        {
            // At least totalFrames images have to be captured
            if( (int)imagePoints.size() > totalFrames-1)
            {

                ROS_INFO_STREAM("Calibrating... Total number of good images: " << (int)imagePoints.size());

                if(c_.calibTarget == c_.opencvStr)
                {
                    runBA(imagePoints, cameraMatrix, rvecs_cam, tvecs_cam);
                }

                ROS_INFO("Saving camera calibration parameters using SetCameraInfo service");

                c_.saveRobotPoses("robot_poses.py", bigMatQ);

                // *****************
                ROS_INFO("Calibration from calibration target to robot gripper (left camera)");
                // false so the transformation is from camera to calibration target
                if(!(hE_robot.loadParameters(rvecs_rb2gripper, tvecs_rb2gripper, rvecs_cam, tvecs_cam, false)))
                    ROS_FATAL("Wrong size in rotation and translation vectors for hand eye calibration");

                // Perform calibration!
                hE_robot.calibrate();
                Mat robotMat = hE_robot.gHc.clone();

                Mat rb2cam = hE_robot.bHg[1] * robotMat * hE_robot.tHc[1].inv();
                // rb2cam = rb2cam.inv();

                ROS_INFO("Robot base to camera (left and right):");
                printMatrix(rb2cam); // optical frame

                // *******************
                tf::TransformListener listener;
                tf::StampedTransform transform, transform2;
                    try{
                      
                      listener.waitForTransform("/base_link", "/zed_left_camera_optical_frame", ros::Time(0), ros::Duration(3.0));
                      listener.lookupTransform("/base_link", "/zed_left_camera_optical_frame", ros::Time(0), transform);
                      //ROS_INFO("Successful.");
                    //  success = true;
                    }
                    catch (tf::TransformException &ex) {
                      ROS_ERROR("%s",ex.what());
                }

                tf::Quaternion q = transform.getRotation();
                tf::Vector3 v = transform.getOrigin();

                ROS_INFO_STREAM(q);
                ROS_INFO_STREAM(v);

                Mat R = getRotation(q.getX(), q.getY(), q.getZ(), q.getW());
                Mat transK = Mat::zeros(3,1, CV_32F);
                transK.at<float>(0) = v.getX();
                transK.at<float>(1) = v.getY();
                transK.at<float>(2) = v.getZ();

                Mat H_rgb2of = Mat::eye(4,4, CV_32F);
                H_rgb2of.at<float>(0,0) = (float)R.at<float>(0,0);
                H_rgb2of.at<float>(0,1) = (float)R.at<float>(0,1);
                H_rgb2of.at<float>(0,2) = (float)R.at<float>(0,2);
                H_rgb2of.at<float>(1,0) = (float)R.at<float>(1,0);
                H_rgb2of.at<float>(1,1) = (float)R.at<float>(1,1);
                H_rgb2of.at<float>(1,2) = (float)R.at<float>(1,2);
                H_rgb2of.at<float>(2,0) = (float)R.at<float>(2,0);
                H_rgb2of.at<float>(2,1) = (float)R.at<float>(2,1);
                H_rgb2of.at<float>(2,2) = (float)R.at<float>(2,2);

                H_rgb2of.at<float>(0,3) = (float)transK.at<float>(0);
                H_rgb2of.at<float>(1,3) = (float)transK.at<float>(1);
                H_rgb2of.at<float>(2,3) = (float)transK.at<float>(2);

                printMatrix(H_rgb2of);


                //   try{
                      
                //       listener.waitForTransform("/camera_link", "/camera_rgb_frame", ros::Time(0), ros::Duration(3.0));
                //       listener.lookupTransform("/camera_link", "/camera_rgb_frame", ros::Time(0), transform2);
                //       //ROS_INFO("Successful.");
                //     //  success = true;
                //     }
                //     catch (tf::TransformException &ex) {
                //       ROS_ERROR("%s",ex.what());
                //   }

                // q = transform2.getRotation();
                // v = transform2.getOrigin();

                // ROS_INFO_STREAM(q);
                // ROS_INFO_STREAM(v);

                // R = getRotation(q.getX(), q.getY(), q.getZ(), q.getW());
                // transK = Mat::zeros(3,1, CV_32F);
                // transK.at<float>(0) = v.getX();
                // transK.at<float>(1) = v.getY();
                // transK.at<float>(2) = v.getZ();

                // Mat H_cl2rgb = Mat::eye(4,4, CV_32F);
                // H_cl2rgb.at<float>(0,0) = (float)R.at<float>(0,0);
                // H_cl2rgb.at<float>(0,1) = (float)R.at<float>(0,1);
                // H_cl2rgb.at<float>(0,2) = (float)R.at<float>(0,2);
                // H_cl2rgb.at<float>(1,0) = (float)R.at<float>(1,0);
                // H_cl2rgb.at<float>(1,1) = (float)R.at<float>(1,1);
                // H_cl2rgb.at<float>(1,2) = (float)R.at<float>(1,2);
                // H_cl2rgb.at<float>(2,0) = (float)R.at<float>(2,0);
                // H_cl2rgb.at<float>(2,1) = (float)R.at<float>(2,1);
                // H_cl2rgb.at<float>(2,2) = (float)R.at<float>(2,2);

                // H_cl2rgb.at<float>(0,3) = (float)transK.at<float>(0);
                // H_cl2rgb.at<float>(1,3) = (float)transK.at<float>(1);
                // H_cl2rgb.at<float>(2,3) = (float)transK.at<float>(2);

                //printMatrix(H_cl2rgb);
      //ros::Duration(1.0).sleep();
   //   succe
                // 1. rgb 2 optical frame
                // 2. camera linl 2 rgb
                // get torso 2 camera link
                // *******************

                //888888888888888888888888888888888888888888888888888888888888
               // Mat rb2cl = rb2cam * H_rgb2of.inv() * H_cl2rgb.inv();
                Mat rb2cl = rb2cam * H_rgb2of.inv();

                ROS_INFO_STREAM("Robot 2 camera link!!!");
                printMatrix(rb2cl);


                Mat rot_rb2cl = Mat::zeros(3,3,CV_32F);
                for(int i = 0; i < rot_rb2cl.rows; i++)
                {
                    for(int j = 0 ; j < rot_rb2cl.cols; j++)
                    {
                        rot_rb2cl.at<float>(i,j) = rb2cl.at<float>(i,j);
                        // cout << rb2cam.at<float>(i,j) << "\t";
                    }
                    // cout<<endl;
                }
                // cout<<endl;

                printMatrix(rot_rb2cl);
                Vec3f rpy = rotationMatrixToEulerAngles(rot_rb2cl);
                ROS_INFO_STREAM(rpy);
                ROS_INFO_STREAM(rb2cl.at<float>(0,3) << " " << rb2cl.at<float>(1,3) << " " << rb2cl.at<float>(2,3) << " " <<rpy.val[2] << " " << rpy.val[1] << " " << rpy.val[0]);

                
                // Save stuff
                if(c_.save_mode >= all)
                {
                    c_.saveTransformations("robotBase_to_gripper_.xml", "base2target", rvecs_rb2gripper, tvecs_rb2gripper);
                    c_.saveTransformations("cam_to_target.xml", "cam2target", rvecs_cam, tvecs_cam);
                }
                c_.saveHandEyeTransform("gripper2target.xml", robotMat, "gripperHtarget");
                saveCalibration(rb2cam.inv(), "camera"); //OK!

                // *****************

                imagePoints.resize(0);
                ROS_INFO("Calibration done!");
                rsp.status_message = "calibrated!";
                rsp.success = true;
            }
            else
            {
                ROS_INFO("At least %i images must be captured", totalFrames);
                rsp.status_message = "calibration failed!!";
                rsp.success = false;
                return false;
            }
        }
        else
        {
            rsp.status_message = "Nothing to do";
            rsp.success = true;
        }

        return true;
    }


private:

    vector<Point2f> pointBuf;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    typedef image_transport::SubscriberFilter ImageSubscriber;


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, geometry_msgs::TransformStamped> syncPolicy;

    ImageSubscriber im_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_;
    message_filters::Subscriber<geometry_msgs::TransformStamped> trans_sub_;

    message_filters::Synchronizer<syncPolicy> sync;

    // Private functions

    // Checks if a matrix is a valid rotation matrix.
    bool isRotationMatrix(Mat &R)
    {
        Mat Rt;
        transpose(R, Rt);
        Mat shouldBeIdentity = Rt * R;
        Mat I = Mat::eye(3,3, shouldBeIdentity.type());
         
        return  norm(I, shouldBeIdentity) < 1e-6;
         
    }
     
    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).
    Vec3f rotationMatrixToEulerAngles(Mat &R)
    {
     
        assert(isRotationMatrix(R));
         
        float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );
     
        bool singular = sy < 1e-6; // If
     
        float x, y, z;
        if (!singular)
        {
            x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
            y = atan2(-R.at<float>(2,0), sy);
            z = atan2(R.at<float>(1,0), R.at<float>(0,0));
        }
        else
        {
            x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
            y = atan2(-R.at<float>(2,0), sy);
            z = 0;
        }
        return Vec3f(x, y, z);
         
         
         
    }

    void processTarget()
    {
        vector<Point2f> pointBufL;
        bool found1 = false;

        Mat inImL;
        // imLeft, cloud_pcl
        imLeft.copyTo(inImL);

        // Find corners
        if(c_.calibTarget == c_.opencvStr)
        {
            ROS_INFO("Finding chessboard corners");
            found_chees = false;
            Mat leftOut = findCorners(inImL);
            found1 = found_chees;
            pointBufL = pointBuf;
            
            display_image(leftOut);

            if(found1){
                imagePoints.push_back(pointBufL);
            }
        }

        if(found1)
        {
            ROS_INFO("Saving images for camera calibration");
            // It saves the images on hardisk for debug purposes and
            // find chessboard corners

            if(c_.save_mode >= images)
                saveImages("_L.tif", imLeft);

            ROS_INFO("Computing camera pose wrt the calibration target...");
            c_.runExtrinsic(cameraMatrix, distCoeffs, pointBufL);
            rvecs_cam.push_back(c_.rvecEx.clone());
            tvecs_cam.push_back(c_.tvecEx.clone());

            // Two transformation required:
            //      1. Robot base to robot gripper
            ROS_INFO_STREAM("Transform from: " << ROBOT_BASE << " to " << GRIPPER_LINK);

            tf::Quaternion q = transform_rotation;
            tf::Vector3 v = transform_translation;

            std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
                      << q.getZ() << ", " << q.getW() << "]" << std::endl;

            Mat rotK = getRotation(q.getX(), q.getY(), q.getZ(), q.getW());

            // Translation
            Mat transK = Mat::zeros(3,1, CV_32F);
            transK.at<float>(0) = v.getX();
            transK.at<float>(1) = v.getY();
            transK.at<float>(2) = v.getZ();

            Mat rTemp;
            Rodrigues(rotK, rTemp);

            printMatrix(rTemp);
            printMatrix(transK);

            rvecs_rb2gripper.push_back(rTemp);
            tvecs_rb2gripper.push_back(transK);
            // savePose(v.getX(), v.getY(), v.getZ(), q.getX(), q.getY(), q.getZ(), q.getW(), 0, 0, 0);

            ROS_INFO_STREAM("Number of images processed: " << c_.nrFrames);
            c_.nrFrames++;

            // Capture next images
        }
        else
        {
            ROS_INFO("Images were not good... capturing new ones!");
        }
    }

    void display_image(cv::Mat inImg)
    {
        Mat dispImgL;
        inImg.copyTo(dispImgL);
        
        line(dispImgL, Point(dispImgL.cols/2.0, 0), Point(dispImgL.cols/2.0, dispImgL.rows), Scalar( 0, 255, 0), 5);
        line(dispImgL, Point(0, dispImgL.rows/2.0), Point(dispImgL.cols, dispImgL.rows/2.0), Scalar( 0, 255, 0), 5);

        cv::imshow(WINDOW_LEFT, dispImgL);
    }

    Mat getCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        Mat K = Mat::zeros(3,3,CV_64F);
        try
        {
            for(int i = 0; i < K.rows; i++)
            {
                for(int j = 0 ; j < K.cols; j++)
                {
                    K.at<double>(i,j) = msg->K[3*i+j];
                }
            }
        }
        catch (...)
        {
            ROS_ERROR("Invalid camera information topic!");
        }

        return K;
    }

    void saveCalibration(cv::Mat cal, string child_frame)
    {
        string calFile;

        calFile = ros::package::getPath("calibration_glasgow") + "/" + child_frame + ".calib";
        ROS_WARN("Calibration saved to %s: ",calFile.c_str());

        std::remove((calFile).c_str());
        std::ofstream file;
        file.open((calFile).c_str(), ios::app);
        file << "2" << endl;

        file << child_frame.c_str() << endl << endl;

        for (int i=0; i<cal.rows; i++)
        {
            for (int j=0; j<cal.cols; j++)
                file << cal.at<float>(i,j) << "  ";

            file << endl;
        }

        file.close();
        //ROS_INFO("Calibration saved to %s: ",calFile.c_str());
    }

    void runBA(vector<vector<Point2f> > imagePoints, Mat&  cameraMatrix_in, vector<Mat>& rvecs, vector<Mat>& tvecs)
    {
        vector<cv::Point3d> points3D;
        vector<vector<Point2d> > pointsImg;
        vector<vector<int > > visibility;
        vector<Mat > cameraMatrix, distCoeffs, R, T;

        vector<Point3f> corners;
        c_.calcBoardCornerPositions(corners);

        ROS_INFO_STREAM("size of image points: " << imagePoints.size());

        int NPOINTS = corners.size(); 	// number of 3d points
        int NCAMS = imagePoints.size(); 	// number of cameras

        ROS_INFO_STREAM("Number of points for BA: " << NPOINTS);

        // fill 3d points
        points3D.resize(NPOINTS);
        for(int i = 0; i < NPOINTS; i++)
        {
            points3D[i].x = (double)corners[i].x;
            points3D[i].y = (double)corners[i].y;
            points3D[i].z = (double)corners[i].z;
        }


        // fill 2d image points
        pointsImg.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)
        {
            pointsImg[i].resize(NPOINTS);
            for(int j = 0; j < NPOINTS; j++)
            {
                pointsImg[i][j].x = (double)imagePoints[i][j].x;
                pointsImg[i][j].y = (double)imagePoints[i][j].y;
            }
        }

        // fill visibility (all points are visible)
        visibility.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)  {
            visibility[i].resize(NPOINTS);
            for(int j=0; j<NPOINTS; j++) visibility[i][j]=1;
        }

        // fill camera intrinsics
        cameraMatrix.resize(NCAMS);
        for(int i=0; i<NCAMS; i++)
            cameraMatrix[i] = cameraMatrix_in.clone();

        // fill distortion (assume no distortion)
        distCoeffs.resize(NCAMS);
        for(int i=0; i<NCAMS; i++) distCoeffs[i] = cv::Mat(5,1,CV_64FC1, cv::Scalar::all(0));

        cvsba::Sba sba;

        cvsba::Sba::Params params;
        params.type = cvsba::Sba::MOTION;
        //params.iterations = 1000;
        //params.minError = 1e-16;
        params.fixedIntrinsics = 5;
        params.fixedDistortion = 5;
        sba.setParams(params);

        sba.run(points3D,  pointsImg,  visibility,  cameraMatrix,  rvecs,  tvecs, distCoeffs);

        ROS_ERROR_STREAM("Initial error=" << sba.getInitialReprjError() << ". Final error=" << sba.getFinalReprjError());


    }

    void printMatrix(Mat M, bool printType = true)
    {
        if(printType)
            ROS_INFO_STREAM("Matrix type:" << M.type());
        // dont print empty matrices
        if (M.empty()){
            ROS_INFO("---");
            return;
        }
        // loop through columns and rows of the matrix
        for(int i=0; i < M.rows; i++){
            for(int j=0; j < M.cols ; j++){
                if(M.type() == 6)
                    cout << M.at<double>(i,j) << "\t";
                else
                    cout << M.at<float>(i,j) << "\t";
            }
            cout<<endl;
        }
        cout<<endl;
    }

    Mat findCorners(const Mat& view)
    {
        Mat view_scaled;
        if(c_.scaleInput > 1.0)
        {
            ROS_INFO_STREAM("Scale factor: " << c_.scaleInput);
            ROS_INFO_STREAM("Size: " << view.cols/c_.scaleInput << " " << view.rows/c_.scaleInput);
            resize(view, view_scaled, Size(view.cols/c_.scaleInput, view.rows/c_.scaleInput),
                   0, 0, cv::INTER_CUBIC);
        }
        else
            view_scaled = view;

        pointBuf.resize(0);
        found_chees = findChessboardCorners( view_scaled, c_.boardSize, pointBuf,
                                             CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if(c_.scaleInput > 1.0)
        {
            for(int i = 0; i < (int)pointBuf.size(); i++)
            {
                pointBuf[i].x *= c_.scaleInput;
                pointBuf[i].y *= c_.scaleInput;
            }
        }

        if (found_chees) // If done with success,
        {
            ROS_INFO("Found corners!");
            // improve the found corners' coordinate accuracy for chessboard
            Mat viewGray;
            cvtColor(view, viewGray, CV_BGR2GRAY);
            cornerSubPix( viewGray, pointBuf, Size(11,11),
                          Size(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                                      30, 0.01));

            // Draw the corners.
            drawChessboardCorners(view, c_.boardSize, Mat(pointBuf), found_chees);
        }
        else
            ROS_ERROR("No corners found...");

        return view;
    }

    void saveImages(string str1, const Mat& imL)
    {
        stringstream ss;
        ss << c_.nrFrames;

        string out_imageL = c_.imageOutputDir + ss.str() + str1;

        ROS_INFO("Saving image to: %s", out_imageL.c_str());

        imwrite(out_imageL, imL);

        ROS_INFO("Image saved!");
    }

    Mat getRotation(double x, double y, double z, double w)
    {
        Mat rotK = Mat::zeros(3,3,CV_32F);

        double sqw = w*w;
        double sqx = x*x;
        double sqy = y*y;
        double sqz = z*z;

        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        double m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
        double m11 = (-sqx + sqy - sqz + sqw)*invs ;
        double m22 = (-sqx - sqy + sqz + sqw)*invs ;

        double tmp1 = x*y;
        double tmp2 = z*w;
        double m10 = 2.0 * (tmp1 + tmp2)*invs ;
        double m01 = 2.0 * (tmp1 - tmp2)*invs ;

        tmp1 = x*z;
        tmp2 = y*w;
        double m20 = 2.0 * (tmp1 - tmp2)*invs ;
        double m02 = 2.0 * (tmp1 + tmp2)*invs ;
        tmp1 = y*z;
        tmp2 = x*w;
        double m21 = 2.0 * (tmp1 + tmp2)*invs ;
        double m12 = 2.0 * (tmp1 - tmp2)*invs ;

        rotK.at<float>(0,0) = (float)m00;
        rotK.at<float>(0,1) = (float)m01;
        rotK.at<float>(0,2) = (float)m02;
        rotK.at<float>(1,0) = (float)m10;
        rotK.at<float>(1,1) = (float)m11;
        rotK.at<float>(1,2) = (float)m12;
        rotK.at<float>(2,0) = (float)m20;
        rotK.at<float>(2,1) = (float)m21;
        rotK.at<float>(2,2) = (float)m22;

        return rotK;
    }

    void savePose(double x, double y, double z, double qx, double qy, double qz, double qw, double yaw, double pitch, double roll)
    {
        Mat m(1, 7, CV_32F);
        Mat mE(1, 7, CV_32F);
        
        // Position
        m.at<float>(0,0) = x;
        m.at<float>(0,1) = y;
        m.at<float>(0,2) = z;
        
        // Orientation (quaternion)
        m.at<float>(0,3) = qx;
        m.at<float>(0,4) = qy;
        m.at<float>(0,5) = qz;
        m.at<float>(0,6) = qw;
        
        bigMatQ.push_back(m);
        
        // Position
        mE.at<float>(0,0) = x;
        mE.at<float>(0,1) = y;
        mE.at<float>(0,2) = z;
        
        // Orientation (quaternion)
        mE.at<float>(0,3) = yaw;
        mE.at<float>(0,4) = pitch;
        mE.at<float>(0,5) = roll;
        
        bigMatE.push_back(mE);
        
        return;

    }

};

/* *************** MAIN PROGRAM *************** */
int main(int argc, char** argv)
{
    ros::init( argc, argv, "rh_calibration_automatic" );
    CmainCalibration cal_;

    while( ros::ok() )
    {
        ros::spin();
    }

    return EXIT_SUCCESS;
}



