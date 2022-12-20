#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

const float chessboardSquareLength = 0.0185f;
const Size chessboardDimension = Size(9,6);

void createknownBoardPosition(Size boardSize, float squareLength, vector<Point3f>& corners)
{
    for(int i = 0; i< boardSize.height;i++)
    {
        for(int j = 0; j< boardSize.width;j++)
        {
            corners.push_back(Point3f(j*squareLength,i*squareLength,0.0f));
        }   
    }
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResult)
{
    for(vector<Mat>::iterator iter = images.begin(); iter!= images.end();iter++)
    {
        vector<Point2f> pointBuf;
        bool found = findChessboardCorners(*iter,Size(9,6),pointBuf,CALIB_CB_ADAPTIVE_THRESH|CALIB_CB_NORMALIZE_IMAGE);

        if(found)
        {
            allFoundCorners.push_back(pointBuf);
        }

        if(showResult)
        {
            drawChessboardCorners(*iter,Size(9,6),pointBuf,found);
            imshow("Photo with dectected corners",*iter);
            waitKey(0);
        }
    }
}

void cameraCalibrattion(vector<Mat> images, Size boardSize, float sLength, Mat& intrinsicMatrix, Mat & distanceCoeff)
{
    vector<vector<Point2f>> foundCornerPoints;
    getChessboardCorners(images,foundCornerPoints,false);

    vector<vector<Point3f>> knownCornerPoints(1);

    createknownBoardPosition(boardSize,sLength,knownCornerPoints[0]);
    knownCornerPoints.resize(images.size(),knownCornerPoints[0]);

    vector<Mat> rv, tv;
    distanceCoeff = Mat::zeros(8,1,CV_64F);

    calibrateCamera(knownCornerPoints,foundCornerPoints,boardSize,intrinsicMatrix,distanceCoeff,rv,tv);

}

void saveImages(vector<Mat>images)
{

    for(int i = 0;i<images.size();i++)
    {
        imwrite("calibrImg"+to_string(i)+".png",images[i]);
    }
}
void printResult(Mat & intrinsicMatrix, Mat & distortionCoeff) 
{
     cout<< "Intrinsic:\n"<<intrinsicMatrix<<"\ndCoeff:\n"<<distortionCoeff<<endl;
}
int main(int argv, char** argc)
{
    Mat frame;
    Mat drawToFrame;

    Mat intrinsicMatrix = Mat::eye(3,3,CV_64F);

    Mat distortionCoeff;

    vector<Mat> savedImages;

    vector<vector<Point2f>> markerCorners, rejectedCandidates;

    VideoCapture vid(0);

    if(!vid.isOpened())
    {
        return -1;
    }

    int fps = 20;

    namedWindow("Webcam",WINDOW_AUTOSIZE);

    while(vid.read(frame))
    {

        vector<Vec2f> foundPoints;
        bool found = false;

        found = findChessboardCorners(frame,chessboardDimension,foundPoints,CALIB_CB_ADAPTIVE_THRESH|CALIB_CB_NORMALIZE_IMAGE|CALIB_CB_FAST_CHECK);

        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame,chessboardDimension,foundPoints,found);
        if(found)
        {
            imshow("Webcam",drawToFrame);
        }else
        {
            imshow("Webcam",frame);
        }
        char key = waitKey(1000/fps);

        switch(key)
        {
            case ' ':
                if(found)
                {
                    Mat tmp;
                    frame.copyTo(tmp);
                    savedImages.push_back(tmp);
                }
                break;
            case 13:
                if(savedImages.size()>15)
                {
                    cameraCalibrattion(savedImages,chessboardDimension,chessboardSquareLength,intrinsicMatrix,distortionCoeff);
                    printResult(intrinsicMatrix,distortionCoeff);
                    saveImages(savedImages);
                }
                break;
            case 27:
                return 0;
                break;
        }
    }
    return 0;
}

