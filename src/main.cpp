#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "Eigen.h"
#include "ImplicitSurface.h"
#include "Volume.h"
#include "MarchingCubes.h"

int main()
{
	std::string filenameIn = "../data/background_subtraction/c_night_green_apple00.jpg";
	std::string filenameOut = "result.off";

	cv::Mat inputImage,pose;
	inputImage=cv::imread(filenameIn,cv::IMREAD_GRAYSCALE);
	// implicit surface
	ImplicitSurface* surface;
	surface = new VoxelCarve(inputImage,pose);

	// fill volume with signed distance values
	unsigned int mc_res = 100; // resolution of the grid, for debugging you can reduce the resolution (-> faster)
	double margin=0.1;
	double ratio=inputImage.size[1]/inputImage.size[0];

	Volume vol(Vector3d(-0.1,-0.1,-0.1), Vector3d(1.1,1.1,1.1), (unsigned int)ratio*mc_res, mc_res, mc_res, 1);
	for (unsigned int x = 0; x < vol.getDimX(); x++)
	{
		for (unsigned int y = 0; y < vol.getDimY(); y++)
		{
			for (unsigned int z = 0; z < vol.getDimZ(); z++)
			{
				Eigen::Vector3d p = vol.pos(x, y, z);
				double val = surface->Eval(p);
				vol.set(x,y,z, val);
			}
		}
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
