#include <iostream>
#include <vector>

#include "Eigen.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

#include "Volume.h"
#include "MarchingCubes.h"
#include "aruco_samples_utility.hpp"

using namespace std;
using namespace cv;

double eval(const Eigen::Vector3d &p, const cv::Mat undistSilhouette, const cv::Mat projectionMatrix)
{
	if (p[0]<0||p[0]>300||p[1]<0||p[1]>300||p[2]<0||p[2]>300)
	{
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

void obtainProjections(vector<cv::Mat> &projections, const Mat& camMatrix, const Mat& distCoeffs, const vector<String>& imageNames,
		int markersX = 5, int markersY = 7, float markerLength = 33, 
		float markerSeparation = 7, int dictionaryId = 15)
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

void carve(Volume& vol, const Mat& camMatrix, const Mat& distCoeffs, const vector<Mat> projections, const vector<String>& silNames)
{
	cout<<"Start carving..."<<endl;
	size_t silCount = silNames.size();
	for(size_t i = 0; i < silCount; i++)
	{
		cv::Mat silImage=imread(silNames[i],IMREAD_GRAYSCALE);
		cv::Mat undistSilhouette;
		cv::undistort(silImage, undistSilhouette, camMatrix, distCoeffs);
		for (unsigned int x = 0; x < vol.getDimX(); x++) {
			for (unsigned int y = 0; y < vol.getDimY(); y++) {
				for (unsigned int z = 0; z < vol.getDimZ(); z++) {
					if (vol.get(x, y, z) > 0.9) {
						continue;
					}
					Eigen::Vector3d p = vol.pos(x, y, z);
					double val = eval(p, undistSilhouette, projections[i]);

					vol.set(x, y, z, val);
				}
			}
		}
		std::cout << "Image " << i+1 << "/" << silCount << std::endl;
	}
	cout<<"Carving done."<<endl;
}

bool writeMesh(const Volume& vol, const String& filenameOut = "result.off")
{
	cout<<"Start building mesh..."<<endl;
	SimpleMesh mesh;
	for (unsigned int x = 0; x < vol.getDimX() - 1; x++)
	{
		std::cerr << "Marching Cubes on slice " << x+1 << " of " << vol.getDimX() << std::endl;

		for (unsigned int y = 0; y < vol.getDimY() - 1; y++)
		{
			for (unsigned int z = 0; z < vol.getDimZ() - 1; z++)
			{
				ProcessVolumeCell(&vol, x, y, z, 0.00f, &mesh);
			}
		}
	}
	cout << "Marching Cubes on slice " << vol.getDimX() << " of " << vol.getDimX() << std::endl;
	cout<<"Mesh done."<<endl;
	return mesh.WriteMesh(filenameOut);
}

int main()
{
	const String cameraDataPath = "config/out_camera_data.yml";
	const String silPath = "data/background_subtraction/c_day_green_apple*.jpg";
	const String imagesPath = "data/background_subtraction/day_green_apple*.jpg";
	const String filenameOut = "result.off";

	Mat camMatrix, distCoeffs;
	if(!readCameraParameters(cameraDataPath, camMatrix, distCoeffs)) {
		cerr << "Invalid camera file" << endl;
		return -1;
	}
	vector<String> imageNames;
	glob(imagesPath, imageNames, false);
	vector<String> silNames;
	glob(silPath, silNames, false);

	std::vector<cv::Mat> projections;
	obtainProjections(projections, camMatrix, distCoeffs, imageNames);

	const unsigned int mc_res = 50; // resolution of the grid, for debugging you can reduce the resolution (-> faster)
	const unsigned int mc_length=300; //mm

	Volume vol(Vector3d(-1, -1, -1), Vector3d(mc_length+1,mc_length+1,mc_length+1), mc_res, mc_res, mc_res, 1);
	vol.clean();

	carve(vol, camMatrix, distCoeffs, projections, silNames);
	if (!writeMesh(vol, filenameOut)) {
		std::cout << "ERROR: unable to write mesh!" << std::endl;
		return -1;
	}

	return 0;
}
