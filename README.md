# Introduction
Project of 3D Scanning and Motion Capture, using ArUco marker and voxel carving to reconstruct 3d model from real Object
# Install
## Dependency
This project use [openCV 4.6.0](https://docs.opencv.org/4.6.0/d7/d9f/tutorial_linux_install.html) and [Eigen 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page), make sure you have them installed.
## Clone the project 
```
git clone https://github.com/VierOrange/Voxel-Carving.git
```
## Build the project

You can build the whole project using cmake:
```
cmake -S . -B build
```
```
cmake --build build
```
This will generate some executables in the build directory.
# Usage
## Camera calibration
You can use the executable file `camera_calibration` to calibrate your camera and get the camera intrinsic matrix and the distortion coefficient.

First, you need to prepare the dataset for camera calibration. We use a set of pictures containing a chessboard to calibrate the camera. 

You can set the path of the pictures in the file `config/VID5.xml`. You can also make some configurations in the file `config/in_VID5.xml`.

Then you can run the program with `build/camera_calibration config/in_VID5.xml` to generate a .yml file which saves the camera intrinsic matrix and distortion coefficient in it.
## Creating ArUco board
We use ArUco board to help estimate the camera pose. To create an ArUco board, use the executable `build/create_aruco_board`.

Example usage:
```
build/create_aruco_board data/aruco_board.png -w=5 -h=7 -l=100 -s=20 -d=15
```
The explanation of the parameters:
| Parameter            | Explanation                                                                                                                                                                                                                                                                                                  |
|----------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| data/aruco_board.png | the path of the generated ArUco board                                                                                                                                                                                                                                                                        |
| -w                   | Number of markers in X direction                                                                                                                                                                                                                                                                             |
| -h                   | Number of markers in Y direction                                                                                                                                                                                                                                                                             |
| -l                   | Marker side length (in pixels)                                                                                                                                                                                                                                                                               |
| -s                   | Separation between two consecutive markers in the grid (in pixels)                                                                                                                                                                                                                                           |
| -d                   | dictionaryID: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16 |
## Background Segmentation
Run 
```
build/background_subtraction
```
You do not need to specify any parameters, but you may need to specify the path of pictures in the source file `src/backgroundSub.cpp`.
## Carving
First you need to prepare the pictures which contains the object to be carved on an ArUco board. 

Then you need to do background subtraction using `build/background_subtraction` to generate masks of the pictures.

The next step is to set a .yml config file which contains the following parameters.
| Parameter        | Explanation                                               |
|------------------|-----------------------------------------------------------|
| CameraDataPath   | the path of the camera data file before                   |
| silPath          | the path of the masks                                     |
| imagesPath       | the path of the images                                    |
| filenameOut      | the path of the generated .off file                       |
| resolution       | the resolution of the voxel (high resolution may be slow) |
| mc_length_x      | the to be carved voxel length in x directin (in mm)       |
| mc_length_y      | the to be carved voxel length in y directin (in mm)       |
| mc_length_z      | the to be carved voxel length in z directin (in mm)       |
| markersX         | Number of markers in X direction                          |
| markersY         | Number of markers in Y direction                          |
| markerLength     | the length of a marker (in mm)                            |
| markerSeparation | the separation between two markers (in mm)                |
| markersX         | Number of markers in X direction                          |
| dictionaryID     | the dictionary ID of the ArUco board                      |

Once set, you can use the `build/main` to do voxel carving. You need to specify the path of the .yml config file and the program will generate a .off file as the result of the voxel carving.

Example usage: 
```
build/main config/parameters.yml
```
