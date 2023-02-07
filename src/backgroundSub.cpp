#include <iostream>
#include <sstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;

void removePixels(Mat &image,int th)
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

            if ((r<th&&b<th&&g<th)||(abs((r+1)/(g+1)-(r+1)/(b+1))<0.1))
            {
                pixel[j][2]=(uchar)255;
                pixel[j][1]=(uchar)255;
                pixel[j][0]=(uchar)255;
            }

        }
    }
}
Mat removePixelsbyRange(Mat &image,Scalar min,Scalar max)
{
    Mat mask;
    inRange(image, min, max, mask);
    return mask;
}

Mat chosePixels(Mat image,Scalar min, Scalar max)
{
    Mat mask;
    inRange(image, min, max, mask); 
    return ~mask;
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
    
    Mat kernel=getStructuringElement(MORPH_RECT, Size(3, 3));

    morphologyEx(image,image,MORPH_CLOSE, kernel);
    
    erode(image,image,kernel,Point(-1,-1),5);
    dilate(image,image,kernel,Point(-1,-1),5);
   
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

        //||4*M_PI*area/pow(premier,2)<0.5
        if(maxV<area||area<minV)
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
    return ~mask;
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

float sub2HSVVec(Vec3f sub1,Vec3f sub2)
{
    Vec3f v = sub1-sub2;
    float h_w = 1;
    float s_w = 0.8;
    float v_w = 0.6;

    float tmph = abs(v[0]);
    if(tmph>230)
    {
        tmph=0.5*(255-tmph);
    }
    return (h_w*tmph+s_w*abs(v[1])+v_w*abs(v[2]))/3;
}
float sub2RGBVec(Vec3f sub1,Vec3f sub2)
{
    Vec3f v = sub1-sub2;

    return (abs(v[0])+abs(v[1])+abs(v[2]))/3;
}
void colorSegmentation(vector<String> names)
{
    for(size_t i = 0;i<names.size();i++)
    {
        Mat image=imread(names[i],IMREAD_COLOR);
        imwrite("step0.jpg",image);
        removePixels(image,20);
        imwrite("step1.jpg",image);

        preProcess(image,200);
        imwrite("step2.jpg",image);

        contour(image,100000,1000000,true,false);
        imwrite("step3.jpg",image);

        ostringstream ss;
        int pos = names[i].find("\\");
        ss <<names[i].substr(0,pos+1)<<"c_"<<names[i].substr(pos+1);
        String filename = ss.str();
        imwrite(filename,image);
    }
}

void framesSubtraction(vector<String> names,float th)
{
    for(size_t i = 0;i<names.size();i+=2)
    {
        Mat image0=imread(names[i],IMREAD_COLOR);
        Mat image1=imread(names[i+1],IMREAD_COLOR);

        // removePixels(image0,200);
        // removePixels(image1,200);
        // cv::cvtColor(image0, image0, cv::COLOR_BGR2HSV);
        // Mat mask = removePixelsbyRange(image0,Scalar(0,0,0),Scalar(255, 50, 90));
        // imwrite("step0.jpg",mask);

        cv::cvtColor(image0, image0, cv::COLOR_BGR2HSV);
        cv::cvtColor(image1, image1, cv::COLOR_BGR2HSV);

        // cv::cvtColor(image0, image0, cv::COLOR_BGR2RGB);
        // cv::cvtColor(image1, image1, cv::COLOR_BGR2RGB);

        blur(image0,image0,Size(9,9));
        blur(image1,image1,Size(9,9));
        
        for(int i=0; i<image0.rows; i++)
        {
            for(int j=0; j<image0.cols; j++) 
            {
                Vec3f sub1 = image0.at<Vec3b>(i,j);
                Vec3f sub2 = image1.at<Vec3b>(i,j);
                if (sub2HSVVec(sub1,sub2)<th)
                {
                    image1.at<Vec3b>(i,j)=Vec3b(0,0,0);
                }else
                {
                    image1.at<Vec3b>(i,j)=Vec3b(255,255,255);
                }
            }

        }
        //grow(image1);
        preProcess(image1,125);
        contour(image1,20000,450000000,false,false);

        ostringstream ss;
        int pos = names[i].find("\\");
        ss <<names[i].substr(0,pos+1)<<"c_"<<names[i].substr(pos+1);
        String filename = ss.str();
        imwrite(filename,image1);
    }
}
int main(int argc, char* argv[])
{
    String path("../data/background_subtraction/sub/day_green_apple*.jpg");
    vector<String> names;
    vector<Mat> images;

    glob(path,names,true);

    // framesSubtraction(names,18);
    colorSegmentation(names);
    waitKey(0);
    return 0;
}