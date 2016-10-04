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
    rgbd_vo::FRAME lastFrame(srcAddress+imData.rgbs[700], srcAddress+imData.depths[700]);
    orb.detectFeatures(lastFrame);
    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double a[3] = {-2.3035,-2.1824,0.5878};
    cv::Mat rawCoordinate(3, 1, CV_64F, a);

    // diagonal matrix for initial rotation matrix
    // later the rvec will be transformed into 3*3 matrix, doing a chain matrix computation to express global rotatoin transform
    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_64F);

    fout << "700" << " " << rawCoordinate.ptr<double>(0)[0] << " " << rawCoordinate.ptr<double>(1)[0] << " " << rawCoordinate.ptr<double>(2)[0] << endl;

    cout << "imData.rgbs.size is " << imData.rgbs.size() << endl;

    for( unsigned short idx = 701; idx < imData.rgbs.size(); idx++) {
        rgbd_vo::FRAME currFrame(srcAddress+imData.rgbs[idx], srcAddress+imData.depths[idx]);
        orb.detectFeatures(currFrame);
        rgbd_vo::PnP result;
        result = rgbd_vo::poseEstimate( lastFrame, currFrame, camera, orb );
        if( !result.useful || result.inliers < min_inliers) {
            // cout << " skip it !!" << endl;
            continue;
        }

        // this frame(process)'s rotation matrix
        cv::Mat rtMat;
        // 1*3 -> 3*3
        cv::Rodrigues(result.rvec, rtMat);

        // tvec is not relative to the origin pos, cannot just add!!
        // cout << "rvec" << result.rvec.size() << endl;
        // cout << "tvec" << result.tvec << endl;
        // cout << "rvec" << result.rvec << endl;
        result.tvec.t();   // transpose tvec to 3*1, add later...

        // chain rotation
        rotationMatrix = rotationMatrix*rtMat;

        // rawCoordinate = rotationMatrix*rawCoordinate + result.tvec;
        rawCoordinate = rawCoordinate + (-rotationMatrix.t())*result.tvec;

        fout << idx << " " << rawCoordinate.ptr<double>(0)[0] << " " << rawCoordinate.ptr<double>(1)[0] << " " << rawCoordinate.ptr<double>(2)[0] << endl;
        lastFrame = currFrame;
        cout << "==============> " << idx << endl;
    }
    return 0;
}
