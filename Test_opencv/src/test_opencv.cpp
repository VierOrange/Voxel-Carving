#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    // create Markers
    // cv::Mat markerImage;
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
    // for(int i = 0; i < 8; ++i) {
    //     cv::aruco::drawMarker(dictionary, i, 200, markerImage, 1);
    //     cv::imwrite("marker" + std::to_string(i) + "_7X7.png", markerImage);
    // }

    cv::Mat inputImage = cv::imread("./input2.jpg", cv::IMREAD_COLOR);
    if (inputImage.empty()) {
        std::cout << "empty image" << std::endl;
        return 0;
    }
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
    cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    cv::Mat outputImage1 = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage1, markerCorners, markerIds);
    cv::imwrite("output2_marker.png", outputImage1);
    
    cv::Mat outputImage2 = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage2, rejectedCandidates);
    cv::imwrite("output2_rejected.png", outputImage2);
    // cv::imshow("out", outputImage1);
    // std::cin.get();
    //
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    while (inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }
        cv::imshow("out", imageCopy);
        char key = (char) cv::waitKey(1);
        if (key == 27) {
            break;
        }
    }
    return 0;
}
