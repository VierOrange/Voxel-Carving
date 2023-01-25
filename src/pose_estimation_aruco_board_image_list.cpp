// by Xing Zhou
// args: -w=5 -h=7 -l=100 -s=20 -d=15 -c=config/out_camera_data.yml -v=data/[image_to_be_read]

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include "aruco_samples_utility.hpp"

using namespace std;
using namespace cv;

const char* about = "Pose estimation using a ArUco Planar Grid board";

const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{l        |       | Marker side length (in pixels) }"
        "{s        |       | Separation between two consecutive markers in the grid (in pixels)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{c        |       | Output file with calibrated camera parameters }"
        "{rs       |       | Apply refind strategy }";


int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 7) {
        parser.printMessage();
        return 0;
    }

    int markersX = parser.get<int>("w");
    int markersY = parser.get<int>("h");
    float markerLength = parser.get<float>("l");
    float markerSeparation = parser.get<float>("s");
    bool refindStrategy = parser.has("rs");


    Mat camMatrix, distCoeffs;
    if(parser.has("c")) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(0);
    if (parser.has("d")) {
        int dictionaryId = parser.get<int>("d");
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    }
    else {
        cerr << "Dictionary not specified" << endl;
        return 0;
    }

    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);

    // create board object
    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

    double totalTime = 0;
    int totalIterations = 0;

    vector<String> vImageList;
    glob("data/apple/*.jpg", vImageList, false);

    vector<Mat> vImages;
    size_t nImages = vImageList.size();
    // for (size_t i = 0; i < nImages; ++i) {
    //     cout << vImageList[i] << endl;
    //     vImages.push_back(imread(vImageList[i]));
    // }
    ostringstream oss;
    oss << nImages << "\n";

    for(size_t i = 0; i < nImages; ++i) {
        Mat image = imread(vImageList[i]);
        Mat imageCopy;

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);

        // estimate board pose
        int markersOfBoardDetected = 0;
        if(ids.size() > 0)
            markersOfBoardDetected =
                aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        if(markersOfBoardDetected > 0)
            cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);


        namedWindow("out", WINDOW_NORMAL);
        imshow("out", imageCopy);
        waitKey(1000);

        // cin.get();

        Mat rotationMatrix;
        Rodrigues(rvec, rotationMatrix);


        oss << String("c_") + vImageList[i] << " ";
        oss << camMatrix.at<double>(0, 0) << " " << camMatrix.at<double>(0, 1) << " " << camMatrix.at<double>(0, 2) << " "
            << camMatrix.at<double>(1, 0) << " " << camMatrix.at<double>(1, 1) << " " << camMatrix.at<double>(1, 2) << " "
            << camMatrix.at<double>(2, 0) << " " << camMatrix.at<double>(2, 1) << " " << camMatrix.at<double>(2, 2) << " ";
        oss << rotationMatrix.at<double>(0, 0) << " " << rotationMatrix.at<double>(0, 1) << " " << rotationMatrix.at<double>(0, 2) << " "
            << rotationMatrix.at<double>(1, 0) << " " << rotationMatrix.at<double>(1, 1) << " " << rotationMatrix.at<double>(1, 2) << " "
            << rotationMatrix.at<double>(2, 0) << " " << rotationMatrix.at<double>(2, 1) << " " << rotationMatrix.at<double>(2, 2) << " ";
        oss << tvec(0) << " " << tvec(1) << " " << tvec(2) << "\n";
    }

    cout << oss.str() << endl;

    ofstream ofs;
    ofs.open("data/apple/apple_par.txt");
    ofs << oss.str();
    ofs.close();

    return 0;
}
