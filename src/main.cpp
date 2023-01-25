#include <iostream>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "Eigen.h"
#include "ImplicitSurface.h"
#include "Volume.h"
#include "MarchingCubes.h"

#include "pose_estimation.h"

using namespace std;
using namespace cv;

int main()
{
	cv::Mat camMatrix, distCoeffs;
    const cv::String cameraDataPath="../config/out_camera_data.yml";
	const std::string filenameOut = "result.off";
	const String silPath("../data/background_subtraction/c_day_green_apple*.jpg");

	bool readOk = readCameraParameters(cameraDataPath, camMatrix, distCoeffs);

    if(!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }
    vector<String> silNames;
	glob(silPath,silNames,true);


    std::vector<cv::Mat> poses;
    // It's better to pass the file path to the obtainPoses function, the function should only get poses
    obtainPoses(poses);

	// implicit surface
	ImplicitSurface* surface = new VoxelCarve();

	const unsigned int mc_res = 50; // resolution of the grid, for debugging you can reduce the resolution (-> faster)

    // why define the positive z axis and then transform?
	Volume vol(Vector3d(-1,-1,-1), Vector3d(301,301,301), mc_res,mc_res,mc_res, 1);
	vol.clean();

    // meaning of variable name?
	cv::Mat undist_sil;
    size_t silCount = silNames.size();
	for(size_t i = 0; i < silCount; i++)
	{
		cv::Mat silImage=imread(silNames[i],IMREAD_GRAYSCALE);
		cv::undistort(silImage, undist_sil, camMatrix, distCoeffs);
		cv::Mat pose = poses[i];
		for (unsigned int x = 0; x < vol.getDimX(); x++)
		{
			for (unsigned int y = 0; y < vol.getDimY(); y++)
			{
				for (unsigned int z = 0; z < vol.getDimZ(); z++)
				{
					if(vol.get(x,y,z)==1)
					{
						continue;
					}
					Eigen::Vector3d p = vol.pos(x, y, z);
                    // better do transformation here
					double val = surface->Eval(p,undist_sil,pose);
					vol.set(x,y,z, val);
				}
			}
		}
		std::cout<<"one done"<<std::endl;
	}

	// extract the zero iso-surface using marching cubes
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

	// write mesh to file
	if (!mesh.WriteMesh(filenameOut))
	{
		std::cout << "ERROR: unable to write output file!" << std::endl;
		return -1;
	}

	delete surface;

	return 0;
}
