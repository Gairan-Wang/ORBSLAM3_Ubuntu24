#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "MapPoint.h"
#include "Settings.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3 {
class Settings;
class MapPoint;
class KeyFrame;
class Atlas;

class MapDrawer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapDrawer(Atlas *pAtlas, const std::string &strSettingPath,
            Settings *settings);

  void newParameterLoader(Settings *settings);

  Atlas *mpAtlas;

  void DrawMapPoints();
  void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph,
                     const bool bDrawInertialGraph, const bool bDrawOptLba);
  void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
  void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
  void SetReferenceKeyFrame(KeyFrame *pKF);
  void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M,
                                    pangolin::OpenGlMatrix &MOw);

private:
  bool ParseViewerParamFile(cv::FileStorage &fSettings);

  float mKeyFrameSize;
  float mKeyFrameLineWidth;
  float mGraphLineWidth;
  float mPointSize;
  float mCameraSize;
  float mCameraLineWidth;

  Sophus::SE3f mCameraPose;

  std::mutex mMutexCamera;

  float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f}, {0.8f, 0.4f, 1.0f},
                               {1.0f, 0.2f, 0.4f}, {0.6f, 0.0f, 1.0f},
                               {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 1.0f}};
};
} // namespace ORB_SLAM3

#endif // MAPDRAWER_H