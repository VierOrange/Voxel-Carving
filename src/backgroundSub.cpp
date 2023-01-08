#include <iostream>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;

void removePixels(Mat &image)
{
    float r, g, b;
    for (int i = 0; i < image.rows; ++i)
    {
        Vec3b* pixel = image.ptr<Vec3b>(i);
        for (int j = 0; j < image.cols; ++j)
        {
            r = pixel[j][2];
            g = pixel[j][1];
            b = pixel[j][0];

            if (abs((r+1)/(b+1)-(r+1)/(g+1))<0.2)
            {
                pixel[j][2]=(uchar)255;
                pixel[j][1]=(uchar)255;
                pixel[j][0]=(uchar)255;
            }

        }
    }
}

void preProcess(Mat & image,int th)
{
    cvtColor(image,image,COLOR_BGR2GRAY);
    GaussianBlur(image,image, Size(3, 3),1);
    threshold(image,image,th, 255, THRESH_BINARY_INV);      
}

void grow(Mat & image)
{
    blur(image,image,Size(9,9));
    
    Mat kernel=getStructuringElement(MORPH_RECT, Size(1, 1));

    morphologyEx(image,image,MORPH_CLOSE, kernel);
    
    dilate(image,image,kernel,Point(-1,-1),4);
    erode(image,image,kernel,Point(-1,-1),4);
   
}

void contour(Mat &image,double minV,double maxV,bool inv=false,bool line=true)
{
    vector<Mat> contours;
    Mat hierarchy;
    findContours(image,contours,hierarchy,RETR_LIST, CHAIN_APPROX_NONE);

    for(auto itr : contours)
    {
        double area = contourArea(itr);
        double premier = arcLength(itr,true);

        if(maxV<area||area<minV||4*M_PI*area/pow(premier,2)<0.5)
        {
            itr.setTo(Scalar(0));
        }
        // cout<<"s"<<area<<endl;
        // cout<<"p"<<premier<<endl;
        // Moments M = moments(itr,true);
	    // cout<<"cx"<< M.m10 / M.m00<<endl;
	    // cout<<"cy"<<M.m01 / M.m00<<endl;
    }

    Mat temp;
    image.copyTo(temp);
    temp.setTo(Scalar(255));
    
    if(line)
        drawContours(temp,contours,-1,Scalar(0,0,0),1);
    else
        drawContours(temp,contours,-1,Scalar(0,0,0),-1);

    if(inv)temp=Scalar(255)-temp;
    image = temp;
}

Mat getMarkerMask(Mat image)
{
    Mat mask = image;
    preProcess(mask,150);
    contour(mask,1,5000,true);
    return mask;
}

Mat getMask(Mat image)
{
    preProcess(image,125);
    Mat mask = image;
    contour(mask,100000,500000,false);
    return mask;
}

Mat applyMask(Mat image,Mat mask)
{
    Mat masked=image;
    vector<Mat> channelsBGR;
    split(masked,channelsBGR);
    channelsBGR[0]=channelsBGR[0].mul(mask/255);
    channelsBGR[1]=channelsBGR[1].mul(mask/255);
    channelsBGR[2]=channelsBGR[2].mul(mask/255);
    merge(channelsBGR,masked);
    return masked;
}

Mat removePixelsbyMask(Mat image,Mat mask)
{
    Mat removed=image;
    vector<Mat> channelsBGR;
    split(removed,channelsBGR);
    channelsBGR[0]+=mask;
    channelsBGR[1]+=mask;
    channelsBGR[2]+=mask;
    merge(channelsBGR,removed);
    return removed;
}

void gradient(Mat & image)
{
    Mat gradX,gradY;
    Sobel(image,gradX, CV_32F,1, 0,-1);
    Sobel(image,gradY, CV_32F,0,1,-1);

    subtract(gradX, gradY,image);

    convertScaleAbs(image,image);   
}

int main(int argc, char* argv[])
{
    String path("../data/background_subtraction/night_green_apple0*.jpg");
    vector<String> names;
    vector<Mat> images;

    glob(path,names,true);

    for(int i = 0;i<names.size();i++)
    {
        Mat image = imread(names[i],IMREAD_COLOR);

        removePixels(image);
        
        preProcess(image,200);

        contour(image,10000,70000,true,false);

        ostringstream ss;
        ss << "../data/background_subtraction/c_night_green_apple0" << i<<".jpg";
        String filename = ss.str();
        imwrite(filename,image);
    }
    waitKey(0);
    return 0;
}