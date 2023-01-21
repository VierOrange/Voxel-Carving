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
	}

	double Eval(const Eigen::Vector3d &_x,cv::Mat silhouete,cv::Mat pose)
	{
		if (_x[0]<0||_x[0]>500||_x[1]<0||_x[1]>500||_x[2]<0||_x[2]>500)
		{
			return 1;
		}
		Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> eigen_pose;
		cv2eigen(pose,eigen_pose);

		// Eigen::Matrix3d rM =Matrix3d::Zero();
		// rM(0,1)=1;
		// rM(1,0)=1;
		// rM(2,2)=-1;

		Eigen::Vector3d tmp_coord =_x;
		tmp_coord = eigen_pose.block(0,0,3,3)*tmp_coord+eigen_pose.block(0,3,3,1);
		tmp_coord /= tmp_coord[2];
		
		if(tmp_coord[0]<0||tmp_coord[1]<0||tmp_coord[0]>silhouete.size[0]||tmp_coord[1]>silhouete.size[1])
		{
			return 1;
		}
		double value = silhouete.at<uchar>((int)tmp_coord[0],(int)tmp_coord[1])<128?1.0:-1.0;
		return value;
	}
};
// class VoxelCarve:public ImplicitSurface
// {
// public:
// 	VoxelCarve(const cv::Mat image, const cv::Mat pose):m_image(image),m_pose(pose)
// 	{
// 		cv2eigen(m_pose,eigen_pose);
// 		m_x=image.size[0];
// 		m_y=image.size[1];
// 	}

// 	double Eval(const Eigen::Vector3d &_x)
// 	{
// 		// if (_x[0]<0||_x[0]>1||(_x[1]<(1-m_x/m_y))||_x[1]>1||_x[2]<0||_x[2]>1)
// 		// {
// 		// 	return 1;
// 		// }
// 		// int x = (1.0-_x[1])*m_y;
// 		// int y = _x[0]*m_y;
// 		// if (_x[0]<0||_x[0]>1||_x[1]<0||_x[1]>1||_x[2]<0||_x[2]>1)
// 		// {
// 		// 	return 1;
// 		// }
// 		// int x = _x[1]*m_x;
// 		// int y = _x[0]*m_y;
// 		// double value = m_image.at<uchar>(x,y)<128?1.0:-1.0;
// 		// return value;

// 		if (_x[0]<0||_x[0]>1||_x[1]<0||_x[1]>1||_x[2]<0||_x[2]>1)
// 		{
// 			return 1;
// 		}
		
// 		Eigen::Vector3d tmp_coord = _x*500;
// 		tmp_coord = eigen_pose.block(0,0,3,3)*tmp_coord+eigen_pose.block(0,3,3,1);
// 		tmp_coord /= tmp_coord[2];
		
// 		double value = m_image.at<uchar>((int)tmp_coord[0],(int)tmp_coord[1])<128?1.0:-1.0;
// 		return value;
// 	}
// private:
// 	cv::Mat m_image,m_pose;
// 	double m_x,m_y;
// 	Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> eigen_pose;
// };
///////////////////////////////////////////

#endif //IMPLICIT_SURFACE_H
