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
	//std::string filenameIn = "../data/background_subtraction/c_day_green_apple00.jpg";

	String path("../data/background_subtraction/c_day_green_apple*.jpg");
    vector<String> silNames;
	glob(path,silNames,true);

	std::string filenameOut = "result.off";

    std::vector<cv::Mat> popo;
    obtainPoses(popo);

	// implicit surface
	ImplicitSurface* surface;
	surface = new VoxelCarve();

	// fill volume with signed distance values
	unsigned int mc_res = 50; // resolution of the grid, for debugging you can reduce the resolution (-> faster)

	Volume vol(Vector3d(-1,-1,-1), Vector3d(501,501,501), mc_res,mc_res,mc_res, 1);

	for(int i = 10;i<11;i++)
	{
		cv::Mat silImage=imread(silNames[i],IMREAD_GRAYSCALE);
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
					double val = surface->Eval(p,silImage,pose);
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
