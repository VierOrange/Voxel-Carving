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

int main()
{
	cv::Mat camMatrix, distCoeffs;
    cv::String cameraDataPath="../config/out_camera_data.yml";

	 bool readOk = readCameraParameters(cameraDataPath, camMatrix, distCoeffs);
    if(!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }
	String path("../data/background_subtraction/c_day_green_apple*.jpg");
    vector<String> silNames;
	glob(path,silNames,true);

	std::string filenameOut = "result.off";

    std::vector<cv::Mat> popo;
    obtainPoses(popo);

	// implicit surface
	ImplicitSurface* surface;
	surface = new VoxelCarve();

	unsigned int mc_res = 100; // resolution of the grid, for debugging you can reduce the resolution (-> faster)

	Volume vol(Vector3d(-1,-1,-1), Vector3d(301,301,301), mc_res,mc_res,mc_res, 1);
	vol.clean();

	cv::Mat undist_mask;
	for(int i = 0;i<silNames.size();i++)
	{
		cv::Mat silImage=imread(silNames[i],IMREAD_GRAYSCALE);
		cv::undistort(silImage, undist_mask, camMatrix, distCoeffs);
		cv::Mat pose = popo[i];
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
					double val = surface->Eval(p,undist_mask,pose);
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
