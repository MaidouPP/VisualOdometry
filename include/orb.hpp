#ifndef ORB_HPP
#define ORB_HPP

#include "common.hpp"
using namespace std;
using namespace cv;

namespace rgbd_vo {

class orbFeature {
public:
    void detectFeatures(FRAME& frame);
    vector<DMatch> matchFeatures(FRAME& f1, FRAME& f2);
private:
    BruteForceMatcher<HammingLUT> matcher;
    OrbFeatureDetector detector;
    OrbDescriptorExtractor extractor;
};

}


#endif // ORB_HPP

/***
 * author: Shixin Li @ SCU
 * date of create: 10/02/2016
***/
