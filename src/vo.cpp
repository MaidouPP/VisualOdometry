/***
 * author: Shixin Li @ SCU
 * date of create: 10/02/2016
***/

#include "vo.hpp"

using namespace std;

//using namespace rgbd_vo;

/*
void computeKeyPointsDesp( FRAME& frame, rgbd::DATA& imData, rgbd_vo::orbFeature& orb, int index, string srcAddress) {
    frame(srcAddress+imData.rgbs[index], srcAddress+imData.depths[index]);
    orb.detectFeature(frame);
    vector<cv::DMatch> goodMatches = orb.matchFeatures(f1, f2);
}
*/

/* useful reminder....
 * *****************************************************
struct DMatch
{ //三个构造函数
DMatch():
queryIdx(-1),trainIdx(-1),imgIdx(-1),distance(std::numeric_limits<float>::max()) {}
DMatch(int _queryIdx, int _trainIdx, float _distance ) :
queryIdx( _queryIdx),trainIdx( _trainIdx), imgIdx(-1),distance( _distance) {}
DMatch(int _queryIdx, int _trainIdx, int _imgIdx, float _distance ) : queryIdx(_queryIdx), trainIdx( _trainIdx), imgIdx( _imgIdx),distance( _distance) {}
int queryIdx; //此匹配对应的查询图像的特征描述子索引 -> use this as first one!!
int trainIdx; //此匹配对应的训练(模板)图像的特征描述子索引 -> second
int imgIdx; //训练图像的索引(若有多个)
float distance; //两个特征向量之间的欧氏距离，越小表明匹配度越高。
booloperator < (const DMatch &m) const;
};

class KeyPoint
{
 Point2f pt; //坐标
float size; //特征点邻域直径
float angle; //特征点的方向，值为[零,三百六十)，负值表示不使用
float response;
int octave; //特征点所在的图像金字塔的组
int class_id; //用于聚类的id
};
 * *******************************************************
*/


namespace rgbd_vo {
// compute motion -> tvec & rvec stand for translation and rotation respectively

PnP poseEstimate( FRAME& f1, FRAME& f2, CAMERA_INTRINSIC_PARAMETERS& camera, orbFeature& orb) {
    vector<cv::DMatch> goodMatches = orb.matchFeatures(f1, f2);
    PnP result;
    if(goodMatches.size() == 0) {
        result.useful = false;
        return result;
    }

    // 3D point of the 1st frame
    vector<cv::Point3f> f1_3D;

    // 2D point of the 2nd frame
    // It's for solvePnPRansac later... It needs a 2D and a 3D.
    vector<cv::Point2f> f2_2D;

    // for loop iterates the matched points
    // frame1 uses 3D; frame2 uses 2D
    for( size_t i = 0; i < goodMatches.size(); i++) {
        // this is (u, v) image coordinate, not camera coordinate.
        cv::Point2f p = f1.keypoints[goodMatches[i].queryIdx].pt;
        // y-> row ; x-> column
        unsigned short d = f1.depth.ptr<unsigned short>(int(p.y))[int(p.x)];
        // if depth is 0, throw this useless point.
        if (d==0)
            continue;
        cv::Point3f temp(p.x, p.y, d);
        f1_3D.push_back(point2DTo3D(temp, camera));
        f2_2D.push_back(f2.keypoints[goodMatches[i].trainIdx].pt);
    }

    if( !f1_3D.size() ){
        result.useful = false;
        return result;
    }

    // provide cameraMatrix to do projection
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << camera.fx, 0, camera.cx, 0, camera.fy, camera.cy, 0, 0 , 1);
    // cv::Mat distortion = ()
    cv::Mat rvec, tvec, inliers;

    cv::solvePnPRansac(f1_3D, f2_2D, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 50, inliers, CV_P3P);
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;
    result.useful = true;

    return result;
}

}
