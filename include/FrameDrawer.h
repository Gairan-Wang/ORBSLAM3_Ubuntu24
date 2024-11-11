#ifndef FRAME_DRAWER_H_
#define FRAME_DRAWER_H_

#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <utility>

namespace ORB_SLAM3 {
class Tracking;
class Viewer;
class Atlas;
class FrameDrawer {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FrameDrawer(Atlas *pAtlas);

  // Update info from the last processed frame.
  void Update(Tracking *pTracker);

  // Draw last processed frame.
  cv::Mat DrawFrame(float imageScale = 1.f);
  cv::Mat DrawRightFrame(float imageScale = 1.f);

  bool both;

protected:
  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

  // Info of the frame to be drawn
  cv::Mat mIm, mImRight;
  int N;
  std::vector<cv::KeyPoint> mvCurrentKeys, mvCurrentKeysRight;
  std::vector<bool> mvbMap, mvbVO;
  bool mbOnlyTracking;
  int mnTracked, mnTrackedVO;
  std::vector<cv::KeyPoint> mvIniKeys;
  std::vector<int> mvIniMatches;
  int mState;
  std::vector<float> mvCurrentDepth;
  float mThDepth;

  Atlas *mpAtlas;

  std::mutex mMutex;
  std::vector<std::pair<cv::Point2f, cv::Point2f>> mvTracks;

  Frame mCurrentFrame;
  std::vector<MapPoint *> mvpLocalMap;
  std::vector<cv::KeyPoint> mvMatchedKeys;
  std::vector<MapPoint *> mvpMatchedMPs;
  std::vector<cv::KeyPoint> mvOutlierKeys;
  std::vector<MapPoint *> mvpOutlierMPs;

  std::map<long unsigned int, cv::Point2f> mmProjectPoints;
  std::map<long unsigned int, cv::Point2f> mmMatchedInImage;
};
} // namespace ORB_SLAM3

#endif // FRAME_DRAWER_H_