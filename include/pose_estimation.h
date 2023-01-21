#include <vector>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "aruco_samples_utility.hpp"
using namespace std;
using namespace cv;

// cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta);
int obtainPoses(vector<cv::Mat> &poses);