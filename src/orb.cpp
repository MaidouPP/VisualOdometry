/***
 * author: Shixin Li @ SCU
 * date of create: 10/02/2016
***/

#include "orb.hpp"

using namespace std;
using namespace rgbd_vo;

void orbFeature::detectFeatures(FRAME& frame) {
    this->detector.detect(frame.rgb, frame.keypoints);
    this->extractor.compute(frame.rgb, frame.keypoints, frame.desp);
    // cout << "desp size: " << frame.desp.size() << endl;
}

vector<cv::DMatch> orbFeature::matchFeatures(FRAME& f1, FRAME& f2) {
    vector<cv::DMatch> matches;
    vector<cv::DMatch> goodMatches;
    if( f1.rgb.cols == f2.rgb.cols ) {
        this->matcher.match(f1.desp, f2.desp, matches);
    }
    else
        return goodMatches;
    //cout << "matches is " << matches.size() << endl;
    double good_match_threshold = 5;
    double minDis = 9999;
    for( auto i:matches) {
        // cannot let distance be 0! or you will end up with no good matches.
        if( i.distance < minDis && i.distance != 0)
            minDis = i.distance;
    }
    // cout << "min Dis is " << minDis << endl;
    for( auto j:matches) {
        //cout << " distance :" << j.distance << endl;
        if( j.distance < good_match_threshold * minDis )
            goodMatches.push_back(j);
    }
    return goodMatches;
}

