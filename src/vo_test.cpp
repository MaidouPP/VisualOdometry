/***
 * author: Shixin Li @ SCU
 * date of create: 10/02/2016
***/

#include "vo.hpp"

using namespace std;

rgbd_vo::DATA imData;

int main() {
    rgbd_vo::ParameterReader pd;
    rgbd_vo::CAMERA_INTRINSIC_PARAMETERS camera = getCamera(pd);
    rgbd_vo::readFrames(pd, imData);
    ofstream fout("output.txt");
    rgbd_vo::orbFeature orb;
    string srcAddress = pd.getData("data_source");
    rgbd_vo::FRAME lastFrame(srcAddress+imData.rgbs[0], srcAddress+imData.depths[0]);
    orb.detectFeatures(lastFrame);
    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double a[3] = {-2.3174,-2.1669,0.5873};
    cv::Mat rawCoordinate(3, 1, CV_64F, a);

    fout << rawCoordinate.ptr<double>(0)[0] << " " << rawCoordinate.ptr<double>(1)[0] << " " << rawCoordinate.ptr<double>(2)[0] << endl;

    for( unsigned short idx = 1; idx < imData.rgbs.size(); idx++) {
        rgbd_vo::FRAME currFrame(srcAddress+imData.rgbs[idx], srcAddress+imData.depths[idx]);
        orb.detectFeatures(currFrame);
        rgbd_vo::PnP result;
        result = rgbd_vo::poseEstimate( lastFrame, currFrame, camera, orb );
        if( !result.useful || result.inliers < min_inliers)
            continue;
//        if( result.inliers < min_inliers )
//            continue;
        cv::Mat rotationMatrix;
        cv::Rodrigues(result.rvec, rotationMatrix);
        result.tvec.t();   // transpose tvec to add later...
        rawCoordinate = rotationMatrix*rawCoordinate + result.tvec;
        fout << rawCoordinate.ptr<double>(0)[0] << " " << rawCoordinate.ptr<double>(1)[0] << " " << rawCoordinate.ptr<double>(2)[0] << endl;
        lastFrame = currFrame;
    }
    return 0;
}
