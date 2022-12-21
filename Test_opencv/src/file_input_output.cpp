#include <opencv2/core.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

class MyData {
public:
    MyData() : A(6), X(6.6), id("6x6") 
    {}
    explicit MyData(int): A(97), X(CV_PI), id("mydata1234")
    {}
    void write(FileStorage& fs) const
    {
        fs << "{" << "A" << A << "X" << X << "id" << id <<  "}";
    }
    void read(const FileNode& node)
    {
        A = (int)node["A"];
        X = (double)node["X"];
        id = (string)node["id"];
    }
public:
    int A;
    double X;
    string id;
};

static void write(FileStorage& fs, const string&, const MyData& x)
{
    x.write(fs);
}
static void read(const FileNode& node, MyData& x, const MyData& default_value = MyData())
{
    if (node.empty()) {
        x = default_value;
    } else {
        x.read(node);
    }
}
static ostream& operator<<(ostream& out, const MyData& m)
{
    out << "{" << "id = " << m.id << "X = " << m.X << ", A = " << m.A << "}";
    return out;
}

static void help(char** av)
{

}
int main(int ac, char** av)
{
    if (ac != 2) {
        help(av);
    }

    string filename = av[1];
    {//write
        Mat R = Mat_<uchar>::eye(3, 3);
        Mat T = Mat_<double>::zeros(3, 1);
        MyData m(1);

        FileStorage fs(filename, FileStorage::WRITE);

        fs << "iterationNr" << 100;
        fs << "strings" << "[";
        fs << "image1.jpg" << "Awesomeness" << "../data/baboon.jpg";
        fs << "]";

        fs << "Mapping";
        fs << "{" << "One" << 1;
        fs << "Two" << 2 << "}";

        fs << "R" << R;
        fs << "T" << T;

        fs << "MyData" << m;
        fs.release();
        cout << "write Done." << endl;
    }
    {//read
        cout << endl << "Reading" << endl;
        FileStorage fs;
        fs.open(filename, FileStorage::READ);

        int itNr = fs["iterationNr"];
        cout << itNr;

        FileNode n = fs["strings"];
        for (FileNodeIterator it = n.begin(); it != n.end(); ++it) {
            cout << (string)*it << endl;
        }

        n = fs["Mapping"];
        cout << (int)n["Two"] << " " << (int)n["One"] << endl;

        MyData m;
        Mat R, T;

        fs["R"] >> R;
        fs["T"] >> T;
        fs["MyData"] >> m;
        
        cout << "R = " << R << endl;
        cout << "T = " << T << endl;
        cout << "MyData = " << m << endl;

        fs["NonExsisting"] >> m;
        cout << "NonExsisting = " << m << endl;
    }

    return 0;

}

