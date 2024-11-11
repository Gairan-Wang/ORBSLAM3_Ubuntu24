#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "SerializationUtils.h"
#include <mutex>
#include <opencv2/core/core.hpp>

#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <set>

namespace ORB_SLAM3 {

class KeyFrame;
class Map;
class Frame;

class MapPoint {

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & mnId;
    ar & mnFirstKFid;
    ar & mnFirstFrame;
    ar & nObs;

    // Protected variables
    ar &boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
    ar &boost::serialization::make_array(mNormalVector.data(),
                                         mNormalVector.size());
    // ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
    // ar & mObservations;
    ar & mBackupObservationsId1;
    ar & mBackupObservationsId2;
    serializeMatrix(ar, mDescriptor, version);
    ar & mBackupRefKFId;
    // ar & mnVisible;
    // ar & mnFound;

    ar & mbBad;
    ar & mBackupReplacedId;

    ar & mfMinDistance;
    ar & mfMaxDistance;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MapPoint();

  MapPoint(const Eigen::Vector3f &Pos, KeyFrame *pRefKF, Map *pMap);
  MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame *pRefKF,
           KeyFrame *pHostKF, Map *pMap);
  MapPoint(const Eigen::Vector3f &Pos, Map *pMap, Frame *pFrame,
           const int &idxF);

  void SetWorldPos(const Eigen::Vector3f &Pos);
  Eigen::Vector3f GetWorldPos();

  Eigen::Vector3f GetNormal();
  void SetNormalVector(const Eigen::Vector3f &normal);

  KeyFrame *GetReferenceKeyFrame();

  std::map<KeyFrame *, std::tuple<int, int>> GetObservations();
  int Observations();

  void AddObservation(KeyFrame *pKF, int idx);
  void EraseObservation(KeyFrame *pKF);

  std::tuple<int, int> GetIndexInKeyFrame(KeyFrame *pKF);
  bool IsInKeyFrame(KeyFrame *pKF);

  void SetBadFlag();
  bool isBad();

  void Replace(MapPoint *pMP);
  MapPoint *GetReplaced();

  void IncreaseVisible(int n = 1);
  void IncreaseFound(int n = 1);
  float GetFoundRatio();
  inline int GetFound() { return mnFound; }

  void ComputeDistinctiveDescriptors();

  cv::Mat GetDescriptor();

  void UpdateNormalAndDepth();

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();
  int PredictScale(const float &currentDist, KeyFrame *pKF);
  int PredictScale(const float &currentDist, Frame *pF);

  Map *GetMap();
  void UpdateMap(Map *pMap);

  void PrintObservations();

  void PreSave(std::set<KeyFrame *> &spKF, std::set<MapPoint *> &spMP);
  void PostLoad(std::map<long unsigned int, KeyFrame *> &mpKFid,
                std::map<long unsigned int, MapPoint *> &mpMPid);

public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Variables used by the tracking
  float mTrackProjX;
  float mTrackProjY;
  float mTrackDepth;
  float mTrackDepthR;
  float mTrackProjXR;
  float mTrackProjYR;
  bool mbTrackInView, mbTrackInViewR;
  int mnTrackScaleLevel, mnTrackScaleLevelR;
  float mTrackViewCos, mTrackViewCosR;
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  Eigen::Vector3f mPosGBA;
  long unsigned int mnBAGlobalForKF;
  long unsigned int mnBALocalForMerge;

  // Variable used by merging
  Eigen::Vector3f mPosMerge;
  Eigen::Vector3f mNormalVectorMerge;

  // Fopr inverse depth optimization
  double mInvDepth;
  double mInitU;
  double mInitV;
  KeyFrame *mpHostKF;

  static std::mutex mGlobalMutex;

  unsigned int mnOriginMapId;

protected:
  // Position in absolute coordinates
  Eigen::Vector3f mWorldPos;

  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame *, std::tuple<int, int>> mObservations;
  // For save relation without pointer, this is necessary for save/load function
  std::map<long unsigned int, int> mBackupObservationsId1;
  std::map<long unsigned int, int> mBackupObservationsId2;

  // Mean viewing direction
  Eigen::Vector3f mNormalVector;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

  // Reference KeyFrame
  KeyFrame *mpRefKF;
  long unsigned int mBackupRefKFId;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  MapPoint *mpReplaced;
  // For save relation without pointer, this is necessary for save/load function
  long long int mBackupReplacedId;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  Map *mpMap;

  // Mutex
  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
  std::mutex mMutexMap;
};

} // namespace ORB_SLAM3

#endif // MAPPOINT_H