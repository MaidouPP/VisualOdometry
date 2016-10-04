#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
using namespace std;

// Eigen
// Attention!!! How to include eigen!!! Not just Eigen/Core !!!
// IMPORTANT!!!!
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

namespace rgbd_vo {

struct DATA {
    vector<string> rgbs, depths;
};

// DATA imData;

struct PnP {
    cv::Mat rvec, tvec;
    int inliers;
    bool useful;
};

// intrinsic camera parameters
struct CAMERA_INTRINSIC_PARAMETERS {
    double cx = 0, cy = 0, fx = 0, fy = 0, scale = 0;
    double d0 = 0, d1 = 0, d2 = 0, d3 = 0, d4 = 0;
};

class ParameterReader {

public:
    ParameterReader(const string& filename = "./parameters.txt") {
        ifstream fin(filename.c_str());
        if(!fin) {
            // look up to the upper directory
            fin.open("../parameters.txt");
            if (!fin) {
              cerr<<"no file: "<<filename<<endl;
              return;
            }
        }
        // start reading...
        while(!fin.eof()) {
            string str;
            getline(fin, str);
            if(str[0] == '#')
                continue;
            int pos = str.find('#');
            if(pos!=std::string::npos)
                str = str.substr(0, pos);
            pos = str.find("=");
            if (pos == std::string::npos)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                 break;
        }
    }

    // get data
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<< key <<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }

protected:
    map<string, string> data;
};

inline static CAMERA_INTRINSIC_PARAMETERS getCamera(ParameterReader& pd)
{
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.d0 = atof( pd.getData( "camera.d0" ).c_str());
    camera.d1 = atof( pd.getData( "camera.d1" ).c_str());
    camera.d2 = atof( pd.getData( "camera.d2" ).c_str());
    camera.d3 = atof( pd.getData( "camera.d3" ).c_str());
    camera.d4 = atof( pd.getData( "camera.d4" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}

class FRAME {
public:
    cv::Mat rgb, depth;
    vector<cv::KeyPoint> keypoints;
    cv::Mat desp;
    FRAME(const string addressRGB, const string addressDEP) {
        rgb = cv::imread(addressRGB.c_str());
        depth = cv::imread(addressDEP.c_str());
    }
};

void inline readFrames(ParameterReader& pd, DATA& imData) {
    string dir = pd.getData("data_source");
    cout << dir << endl;
    ifstream fin(dir + "associations.txt");
    if(!fin) {
        cerr << "cannot find association file. Please create one." << endl;
        return;
    }
    // reading file...
    while(!fin.eof()) {
        string rgbTime, rgbFile, depthTime, depthFile;
        fin >> rgbTime >> rgbFile >> depthTime >> depthFile;
        imData.rgbs.push_back(rgbFile);
        imData.depths.push_back(depthFile);
    }
    cout << "The number of images is " << imData.depths.size() << " ." << endl;
}

}

#endif // COMMON_HPP

/***
 * author: Shixin Li @ SCU
 * date of create: 10/02/2016
***/
