#include <iostream>
#include "MarchingCubes.h"
#include "VoxelCarving.hpp"

using namespace std;


int main(int argc, char** argv)
{
    if (argc != 2) {
        cout << "Incorrect usage!" << endl;
        return -1;
    }

    VoxelCarving voxelCarving(argv[1]);

    cout << endl << "Initializing" << endl;
    if (!voxelCarving.initialize()) {
        cout << "Initialization failed" << endl;
        return -1;
    }
    cout << "Initilization complete!" << endl;

    voxelCarving.obtainProjections();
    voxelCarving.carve();
    if (!voxelCarving.writeMesh()) {
		std::cout << "ERROR: unable to write mesh!" << std::endl;
		return -1;
    }
	cout<<"Mesh done."<<endl;

	return 0;
}
