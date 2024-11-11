#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "Atlas.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "ORBVocabulary.h"
#include <boost/algorithm/string.hpp>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <mutex>
#include <thread>

namespace ORB_SLAM3 {
class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;

class LoopClosing {
public:
  typedef std::pair<std::set<KeyFrame *>, int> ConsistentGroup;
  typedef std::map<
      KeyFrame *, g2o::Sim3, std::less<KeyFrame *>,
      Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3>>>
      KeyFrameAndPose;

public:
  LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc,
              const bool bFixScale, const bool bActiveLC);

  void SetTracker(Tracking *pTracker);

  void SetLocalMapper(LocalMapping *pLocalMapper);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFrame *pKF);

  void RequestReset();
  void RequestResetActiveMap(Map *pMap);

  // This function will run in a separate thread
  void RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nLoopKF);

  bool isRunningGBA() {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    return mbRunningGBA;
  }
  bool isFinishedGBA() {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    return mbFinishedGBA;
  }

  void RequestFinish();

  bool isFinished();

  Viewer *mpViewer;

#ifdef REGISTER_TIMES

  vector<double> vdDataQuery_ms;
  vector<double> vdEstSim3_ms;
  vector<double> vdPRTotal_ms;

  vector<double> vdMergeMaps_ms;
  vector<double> vdWeldingBA_ms;
  vector<double> vdMergeOptEss_ms;
  vector<double> vdMergeTotal_ms;
  vector<int> vnMergeKFs;
  vector<int> vnMergeMPs;
  int nMerges;

  vector<double> vdLoopFusion_ms;
  vector<double> vdLoopOptEss_ms;
  vector<double> vdLoopTotal_ms;
  vector<int> vnLoopKFs;
  int nLoop;

  vector<double> vdGBA_ms;
  vector<double> vdUpdateMap_ms;
  vector<double> vdFGBATotal_ms;
  vector<int> vnGBAKFs;
  vector<int> vnGBAMPs;
  int nFGBA_exec;
  int nFGBA_abort;

#endif

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  bool CheckNewKeyFrames();

  // Methods to implement the new place recognition algorithm
  bool NewDetectCommonRegions();
  bool DetectAndReffineSim3FromLastKF(KeyFrame *pCurrentKF,
                                      KeyFrame *pMatchedKF, g2o::Sim3 &gScw,
                                      int &nNumProjMatches,
                                      std::vector<MapPoint *> &vpMPs,
                                      std::vector<MapPoint *> &vpMatchedMPs);
  bool DetectCommonRegionsFromBoW(std::vector<KeyFrame *> &vpBowCand,
                                  KeyFrame *&pMatchedKF,
                                  KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
                                  int &nNumCoincidences,
                                  std::vector<MapPoint *> &vpMPs,
                                  std::vector<MapPoint *> &vpMatchedMPs);
  bool DetectCommonRegionsFromLastKF(KeyFrame *pCurrentKF, KeyFrame *pMatchedKF,
                                     g2o::Sim3 &gScw, int &nNumProjMatches,
                                     std::vector<MapPoint *> &vpMPs,
                                     std::vector<MapPoint *> &vpMatchedMPs);
  int FindMatchesByProjection(KeyFrame *pCurrentKF, KeyFrame *pMatchedKFw,
                              g2o::Sim3 &g2oScw,
                              std::set<MapPoint *> &spMatchedMPinOrigin,
                              std::vector<MapPoint *> &vpMapPoints,
                              std::vector<MapPoint *> &vpMatchedMapPoints);

  void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
                     std::vector<MapPoint *> &vpMapPoints);
  void SearchAndFuse(const std::vector<KeyFrame *> &vConectedKFs,
                     std::vector<MapPoint *> &vpMapPoints);

  void CorrectLoop();

  void MergeLocal();
  void MergeLocal2();

  void CheckObservations(std::set<KeyFrame *> &spKFsMap1,
                         std::set<KeyFrame *> &spKFsMap2);

  void ResetIfRequested();
  bool mbResetRequested;
  bool mbResetActiveMapRequested;
  Map *mpMapToReset;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  Atlas *mpAtlas;
  Tracking *mpTracker;

  KeyFrameDatabase *mpKeyFrameDB;
  ORBVocabulary *mpORBVocabulary;

  LocalMapping *mpLocalMapper;

  std::list<KeyFrame *> mlpLoopKeyFrameQueue;

  std::mutex mMutexLoopQueue;

  // Loop detector parameters
  float mnCovisibilityConsistencyTh;

  // Loop detector variables
  KeyFrame *mpCurrentKF;
  KeyFrame *mpLastCurrentKF;
  KeyFrame *mpMatchedKF;
  std::vector<ConsistentGroup> mvConsistentGroups;
  std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
  std::vector<KeyFrame *> mvpCurrentConnectedKFs;
  std::vector<MapPoint *> mvpCurrentMatchedPoints;
  std::vector<MapPoint *> mvpLoopMapPoints;
  cv::Mat mScw;
  g2o::Sim3 mg2oScw;

  //-------
  Map *mpLastMap;

  bool mbLoopDetected;
  int mnLoopNumCoincidences;
  int mnLoopNumNotFound;
  KeyFrame *mpLoopLastCurrentKF;
  g2o::Sim3 mg2oLoopSlw;
  g2o::Sim3 mg2oLoopScw;
  KeyFrame *mpLoopMatchedKF;
  std::vector<MapPoint *> mvpLoopMPs;
  std::vector<MapPoint *> mvpLoopMatchedMPs;
  bool mbMergeDetected;
  int mnMergeNumCoincidences;
  int mnMergeNumNotFound;
  KeyFrame *mpMergeLastCurrentKF;
  g2o::Sim3 mg2oMergeSlw;
  g2o::Sim3 mg2oMergeSmw;
  g2o::Sim3 mg2oMergeScw;
  KeyFrame *mpMergeMatchedKF;
  std::vector<MapPoint *> mvpMergeMPs;
  std::vector<MapPoint *> mvpMergeMatchedMPs;
  std::vector<KeyFrame *> mvpMergeConnectedKFs;

  g2o::Sim3 mSold_new;
  //-------

  long unsigned int mLastLoopKFid;

  // Variables related to Global Bundle Adjustment
  bool mbRunningGBA;
  bool mbFinishedGBA;
  bool mbStopGBA;
  std::mutex mMutexGBA;
  std::thread *mpThreadGBA;

  // Fix scale in the stereo/RGB-D case
  bool mbFixScale;

  bool mnFullBAIdx;

  std::vector<double> vdPR_CurrentTime;
  std::vector<double> vdPR_MatchedTime;
  std::vector<int> vnPR_TypeRecogn;

  // DEBUG
  std::string mstrFolderSubTraj;
  int mnNumCorrection;
  int mnCorrectionGBA;

  // To (de)activate LC
  bool mbActiveLC = true;

#ifdef REGISTER_LOOP
  string mstrFolderLoop;
#endif
};

} // namespace ORB_SLAM3

#endif // LOOPCLOSING_H