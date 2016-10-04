/***
 * author: Shixin Li @ SCU
 * date of create: 10/02/2016
***/

#include "common.hpp"
#include "orb.hpp"

using namespace std;

rgbd_vo::DATA imData;
//extern void rgbd_vo::readFrames(rgbd_vo::ParameterReader& pd);

int main() {
    rgbd_vo::orbFeature orb;
    rgbd_vo::ParameterReader pd;
    rgbd_vo::readFrames(pd, imData);
    string sourceAddress = pd.getData("data_source");
    // pick the first two images
    rgbd_vo::FRAME f1(sourceAddress+imData.rgbs[0], sourceAddress+imData.depths[0]);
    rgbd_vo::FRAME f2(sourceAddress+imData.rgbs[1], sourceAddress+imData.depths[1]);
    orb.detectFeatures(f1);
    orb.detectFeatures(f2);
    vector<cv::DMatch> goodMatches = orb.matchFeatures(f1, f2);
    cout << "good matches: " << goodMatches.size() << endl;
    cv::Mat imgMatches;
    // show the good matches
    cv::drawMatches( f1.rgb, f1.keypoints, f2.rgb, f2.keypoints, goodMatches, imgMatches);
    cv::imshow("matches", imgMatches);
    cv::waitKey(0);
}
