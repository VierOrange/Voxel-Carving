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

double eval(const Eigen::Vector3d &p, const cv::Mat undist_sil, const cv::Mat projectionMatrix)
{
	Eigen::MatrixXd projectionMatrixEigen;
	cv2eigen(projectionMatrix, projectionMatrixEigen);
	Eigen::Vector3d pPixel = projectionMatrixEigen.block(0, 0, 3, 3) * p + projectionMatrixEigen.block(0, 3, 3, 1); 
	pPixel /= pPixel.z();

	if (pPixel.x() < 0 || pPixel.x() > undist_sil.size[0] || pPixel.y() < 0 || pPixel.y() > undist_sil.size[1] ) {
		return 1.0;
	}
	return undist_sil.at<uchar>((int)pPixel.y(), (int)pPixel.x()) < 128 ? 1.0 : -1.0;
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
	
	for(unsigned int i = 0; i< imageNames.size(); i++)
	{
		Mat image = imread(imageNames[i],IMREAD_COLOR);

		vector< int > ids;
		vector< vector< Point2f > > corners, rejected;
		Vec3d rvec, tvec;

		// detect markers
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

		if(ids.size() > 0)
			aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);
		
		Mat rotationMatrix;
		cv::Rodrigues(rvec, rotationMatrix);
		Mat transformationMatrix;
		hconcat(rotationMatrix, tvec, transformationMatrix);
		projections.push_back(camMatrix * transformationMatrix);
	}
}

void carve(Volume& vol, const Mat& camMatrix, const Mat& distCoeffs, const vector<Mat> projections, const vector<String>& silNames)
{
	size_t silCount = silNames.size();
	for(size_t i = 0; i < silCount; i++)
	{
		cv::Mat silImage=imread(silNames[i],IMREAD_GRAYSCALE);
		cv::Mat undist_sil;
		cv::undistort(silImage, undist_sil, camMatrix, distCoeffs);
		for (unsigned int x = 0; x < vol.getDimX(); x++) {
			for (unsigned int y = 0; y < vol.getDimY(); y++) {
				for (unsigned int z = 0; z < vol.getDimZ(); z++) {
					if (vol.get(x, y, z) > 0.9) {
						continue;
					}
					Eigen::Vector3d p = vol.pos(x, y, z);
					double val = eval(p, undist_sil, projections[i]);

					vol.set(x, y, z, val);
				}
			}
		}
		std::cout << "Image " << i << "/" << silCount << "done." << std::endl;
	}
}

bool writeMesh(const Volume& vol, const String& filenameOut = "result.off")
{
	SimpleMesh mesh;
	for (unsigned int x = 0; x < vol.getDimX() - 1; x++)
	{
		std::cerr << "Marching Cubes on slice " << x << " of " << vol.getDimX() << std::endl;

		for (unsigned int y = 0; y < vol.getDimY() - 1; y++)
		{
			for (unsigned int z = 0; z < vol.getDimZ() - 1; z++)
			{
				ProcessVolumeCell(&vol, x, y, z, 0.00f, &mesh);
			}
		}
	}
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

	Volume vol(Vector3d(0, 0, 0), Vector3d(300,300,-300), mc_res, mc_res, mc_res, 1);
	vol.clean();

	carve(vol, camMatrix, distCoeffs, projections, silNames);
	if (!writeMesh(vol, filenameOut)) {
		std::cout << "ERROR: unable to write mesh!" << std::endl;
		return -1;
	}

	return 0;
}
