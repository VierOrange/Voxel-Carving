#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include "Volume.h"


class VoxelCarving {
public:
    VoxelCarving(const cv::String& c);
    ~VoxelCarving();

    bool initialize();
    void obtainProjections();
    void carve() const;
    bool writeMesh() const;

private:
    cv::String configFile;

    cv::String cameraDataPath;
    cv::String silPath;
    cv::String imagesPath;
    cv::String filenameOut;
    int mc_res;
    int mc_length;

    int markersX;
    int markersY;
    float markerLength; 
	float markerSeparation; 
    int dictionaryId;


    cv::Mat camMatrix;
    cv::Mat distCoeffs;

    std::vector<cv::String> imageNames;
    std::vector<cv::String> silNames;

	std::vector<cv::Mat> projections;

    Volume* pVol;

    bool readParameters();
    static double eval(const Eigen::Vector3d& p, const cv::Mat& undistSilhouette, const cv::Mat& projectionMatrix);
};
