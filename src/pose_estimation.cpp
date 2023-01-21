#include "pose_estimation.h"

const char* about = "Pose estimation using a ArUco Planar Grid board";

int obtainPoses(vector<cv::Mat> &poses)
{
   
    int markersX = 5;
    int markersY = 7;
    float markerLength = 100;
    float markerSeparation = 20;
    int dictionaryId = 15;


    Mat camMatrix, distCoeffs;
    String cameraDataPath="../config/out_camera_data.yml";
    String imagesPath("../data/background_subtraction/day_green_apple*.jpg");

    bool readOk = readCameraParameters(cameraDataPath, camMatrix, distCoeffs);
    if(!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(0);
    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    float axisLength = 0.5f * ((float)min(markersX, markersY) * (markerLength + markerSeparation) +
                               markerSeparation);
    // create board object
    Ptr<aruco::GridBoard> gridboard =
        aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

    
    vector<String> names;
    vector<Mat> images;

    glob(imagesPath,names,true);

    for(int i = 0;i<16;i++)
    {
        Mat image, imageCopy,pose;
        image=imread(names[i],IMREAD_COLOR);

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);

        // estimate board pose
        int markersOfBoardDetected = 0;
        if(ids.size() > 0)
            markersOfBoardDetected =
                aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

        cv::Rodrigues(rvec,pose);
        hconcat(pose, tvec, pose);

	    pose = camMatrix * pose;
        // std::cout<<pose<<std::endl<<std::endl;
        poses.push_back(pose);
    }
    return 1;
}
