#include "VoxelCarving.hpp"
#include "aruco_samples_utility.hpp"
#include "SimpleMesh.h"
#include "Eigen.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace std;

VoxelCarving::VoxelCarving(const cv::String& c) : configFile(c)
{
}

VoxelCarving::~VoxelCarving()
{
    delete pVol;
}

bool VoxelCarving::initialize()
{
    if (!readParameters()) {
        return false;
    }
    if(!readCameraParameters(cameraDataPath, camMatrix, distCoeffs)) {
		return false;
	}
    cv::glob(imagesPath, imageNames, false);
    cv::glob(silPath, silNames, false);
    pVol = new Volume(Vector3d(-1, -1, -1), Vector3d(mc_length_x+1,mc_length_y+1,mc_length_z+1), mc_res, mc_res, mc_res, 1);
	pVol->clean();
    return true;
}

void VoxelCarving::obtainProjections()
{
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
	Ptr<aruco::GridBoard> gridboard =
		aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
	Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();
	
	vector< int > ids;
	vector< vector< Point2f > > corners, rejected;
	Vec3d rvec, tvec;

	Mat rotationMatrix;
	Mat transformationMatrix;

	// The rvec and tvec obtained by opencv is somehow downward, need to transform it before use it.
	Mat trans = Mat::zeros(4, 4, CV_64F);
	trans.at<double>(0,1)=1;
	trans.at<double>(1,0)=1;
	trans.at<double>(2,2)=-1;
	trans.at<double>(3,3)=1;

	for(unsigned int i = 0; i< imageNames.size(); i++)
	{
		Mat image = imread(imageNames[i],IMREAD_COLOR);

		// detect markers
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

		if(ids.size() > 0)
			aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);
		else
		{
			cout<<"No Markers detected!"<<endl;
			break;
		}
		cv::Rodrigues(rvec, rotationMatrix);
		hconcat(rotationMatrix, tvec, transformationMatrix);
		projections.push_back(camMatrix * transformationMatrix*trans);
	}
}

bool VoxelCarving::readParameters()
{
    cv::FileStorage fs(configFile, cv::FileStorage::READ);

    fs["cameraDataPath"] >> cameraDataPath;
    fs["silPath"] >> silPath;
    fs["imagesPath"] >> imagesPath;
    fs["filenameOut"] >> filenameOut;
    fs["resolution"] >> mc_res;
    fs["mc_length_x"] >> mc_length_x;
    fs["mc_length_y"] >> mc_length_y;
    fs["mc_length_z"] >> mc_length_z;

    fs["markersX"] >> markersX;
    fs["markersY"] >> markersY;
    fs["markerLength"] >> markerLength;
    fs["markerSeparation"] >> markerSeparation;
    fs["dictionaryId"] >> dictionaryId;

    fs.release();
    return true;
}

void VoxelCarving::carve() const
{
	size_t silCount = silNames.size();
	for(size_t i = 0; i < silCount; i++) {
		cv::Mat silImage = imread(silNames[i],IMREAD_GRAYSCALE);
		cv::Mat undistSilhouette;
		cv::undistort(silImage, undistSilhouette, camMatrix, distCoeffs);
		for (unsigned int x = 0; x < pVol->getDimX(); x++) {
			for (unsigned int y = 0; y < pVol->getDimY(); y++) {
				for (unsigned int z = 0; z < pVol->getDimZ(); z++) {
					if (pVol->get(x, y, z) > 0.9) {
						continue;
					}
					Eigen::Vector3d p = pVol->pos(x, y, z);
					double val = VoxelCarving::eval(p, undistSilhouette, projections[i]);

					pVol->set(x, y, z, val);
				}
			}
		}
		std::cout << "Image " << i+1 << "/" << silCount << std::endl;
	}
}

extern bool ProcessVolumeCell(const Volume* vol, int x, int y, int z, double iso, SimpleMesh* mesh);

bool VoxelCarving::writeMesh() const
{
	cout<<"Start building mesh..."<<endl;
	SimpleMesh mesh;
	for (unsigned int x = 0; x < pVol->getDimX() - 1; x++) {
		std::cerr << "Marching Cubes on slice " << x+1 << " of " << pVol->getDimX() << std::endl;

		for (unsigned int y = 0; y < pVol->getDimY() - 1; y++) {
			for (unsigned int z = 0; z < pVol->getDimZ() - 1; z++) {
				ProcessVolumeCell(pVol, x, y, z, 0.00f, &mesh);
			}
		}
	}
	cout << "Marching Cubes on slice " << pVol->getDimX() << " of " << pVol->getDimX() << std::endl;
	return mesh.WriteMesh(filenameOut);
}

double VoxelCarving::eval(const Eigen::Vector3d& p, const cv::Mat& undistSilhouette, const cv::Mat& projectionMatrix)
{
	if (p[0]<0||p[0]>300||p[1]<0||p[1]>300||p[2]<0||p[2]>300) {
		return 1;
	}

	Eigen::MatrixXd projectionMatrixEigen;
	cv2eigen(projectionMatrix, projectionMatrixEigen);
	Eigen::Vector3d  pPixel=projectionMatrixEigen.block(0, 0, 3, 3) * p + projectionMatrixEigen.block(0, 3, 3, 1); 
	pPixel /= pPixel.z();

	if (pPixel.x() < 0 || pPixel.x() > undistSilhouette.size[1] || pPixel.y() < 0 || pPixel.y() > undistSilhouette.size[0] ) {
		return 1.0;
	}
	return undistSilhouette.at<uchar>((int)pPixel.y(), (int)pPixel.x()) < 128 ? 1.0 : -1.0;
}

