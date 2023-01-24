#pragma once

#ifndef IMPLICIT_SURFACE_H
#define IMPLICIT_SURFACE_H

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "Eigen.h"
#include "SimpleMesh.h"

class ImplicitSurface
{
public:
	virtual double Eval(const Eigen::Vector3d& x,cv::Mat silhouete,cv::Mat pose) = 0;
};
class VoxelCarve:public ImplicitSurface
{
public:
	VoxelCarve()
	{
		trans(0,1)=1;
		trans(1,0)=1;
		trans(2,2)=-1;
	}

	double Eval(const Eigen::Vector3d &_x,cv::Mat silhouete,cv::Mat pose)
	{
		if (_x[0]<0||_x[0]>300||_x[1]<0||_x[1]>300||_x[2]<0||_x[2]>300)
		{
			return 1;
		}
		Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> eigen_pose;
		cv2eigen(pose,eigen_pose);
	
		// Transform voxel's base frame
		Eigen::Vector3d tmp_coord =trans*_x;

		tmp_coord = eigen_pose.block(0,0,3,3)*tmp_coord + eigen_pose.block(0,3,3,1);
		tmp_coord /= tmp_coord[2];
		
		if(tmp_coord[0]<0||tmp_coord[1]<0||tmp_coord[0]>silhouete.size[0]||tmp_coord[1]>silhouete.size[1])
		{
			return 1;
		}

		double value = silhouete.at<uchar>((int)tmp_coord[1],(int)tmp_coord[0])<128?1.0:-1.0;//There is a bugs' dead body here.
		return value;
	}
	private:
		Eigen::Matrix3d trans = Matrix3d::Zero();
};
#endif //IMPLICIT_SURFACE_H
