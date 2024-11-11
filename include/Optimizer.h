#ifndef MY_SLAM_OPTIMIZER_H_
#define MY_SLAM_OPTIMIZER_H_

#include "Frame.h"
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"

#include <math.h>

namespace ORB_SLAM3 {

class LocalMapping;
class Optimizer {
public:
  void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF,
                               const std::vector<MapPoint *> &vpMP,
                               int nIterations = 5, bool *pbStopFlag = NULL,
                               const unsigned long nLoopKF = 0,
                               const bool bRobust = true);
  void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5,
                                     bool *pbStopFlag = NULL,
                                     const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);
  void static FullInertialBA(Map *pMap, int its, const bool bFixLocal = false,
                             const unsigned long nLoopKF = 0,
                             bool *pbStopFlag = NULL, bool bInit = false,
                             float priorG = 1e2, float priorA = 1e6,
                             Eigen::VectorXd *vSingVal = NULL,
                             bool *bHess = NULL);

  void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                                    int &num_fixedKF, int &num_OptKF,
                                    int &num_MPs, int &num_edges);

  int static PoseOptimization(Frame *pFrame);
  int static PoseInertialOptimizationLastKeyFrame(Frame *pFrame,
                                                  bool bRecInit = false);
  int static PoseInertialOptimizationLastFrame(Frame *pFrame,
                                               bool bRecInit = false);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
  // (mono)
  void static OptimizeEssentialGraph(
      Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
      const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose &CorrectedSim3,
      const std::map<KeyFrame *, std::set<KeyFrame *>> &LoopConnections,
      const bool &bFixScale);
  void static OptimizeEssentialGraph(
      KeyFrame *pCurKF, std::vector<KeyFrame *> &vpFixedKFs,
      std::vector<KeyFrame *> &vpFixedCorrectedKFs,
      std::vector<KeyFrame *> &vpNonFixedKFs,
      std::vector<MapPoint *> &vpNonCorrectedMPs);

  // For inertial loopclosing
  void static OptimizeEssentialGraph4DoF(
      Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
      const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
      const LoopClosing::KeyFrameAndPose &CorrectedSim3,
      const std::map<KeyFrame *, std::set<KeyFrame *>> &LoopConnections);

  // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
  // (NEW)
  static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2,
                          std::vector<MapPoint *> &vpMatches1,
                          g2o::Sim3 &g2oS12, const float th2,
                          const bool bFixScale,
                          Eigen::Matrix<double, 7, 7> &mAcumHessian,
                          const bool bAllPoints = false);

  // For inertial systems

  void static LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap,
                              int &num_fixedKF, int &num_OptKF, int &num_MPs,
                              int &num_edges, bool bLarge = false,
                              bool bRecInit = false);
  void static MergeInertialBA(KeyFrame *pCurrKF, KeyFrame *pMergeKF,
                              bool *pbStopFlag, Map *pMap,
                              LoopClosing::KeyFrameAndPose &corrPoses);

  // Local BA in welding area when two maps are merged
  void static LocalBundleAdjustment(KeyFrame *pMainKF,
                                    std::vector<KeyFrame *> vpAdjustKF,
                                    std::vector<KeyFrame *> vpFixedKF,
                                    bool *pbStopFlag);

  // Marginalize block element (start:end,start:end). Perform Schur complement.
  // Marginalized elements are filled with zeros.
  static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start,
                                     const int &end);

  // Inertial pose-graph
  void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg,
                                   double &scale, Eigen::Vector3d &bg,
                                   Eigen::Vector3d &ba, bool bMono,
                                   Eigen::MatrixXd &covInertial,
                                   bool bFixedVel = false, bool bGauss = false,
                                   float priorG = 1e2, float priorA = 1e6);
  void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg,
                                   Eigen::Vector3d &ba, float priorG = 1e2,
                                   float priorA = 1e6);
  void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg,
                                   double &scale);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} // namespace ORB_SLAM3
//
#endif // MY_SLAM_OPTIMIZER_H_