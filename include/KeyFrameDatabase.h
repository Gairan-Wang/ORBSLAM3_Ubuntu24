#ifndef KEYFRAME_DATABASE_H_
#define KEYFRAME_DATABASE_H_

#include <list>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ORBVocabulary.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <mutex>

namespace ORB_SLAM3 {
class KeyFrame;
class Frame;
class Map;

class KeyFrameDatabase {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & mvBackupInvertedFileId;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KeyFrameDatabase() {}
  KeyFrameDatabase(const ORBVocabulary &voc);

  void add(KeyFrame *pKF);

  void erase(KeyFrame *pKF);

  void clear();
  void clearMap(Map *pMap);

  // Loop Detection(DEPRECATED)
  std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

  // Loop and Merge Detection
  void DetectCandidates(KeyFrame *pKF, float minScore,
                        std::vector<KeyFrame *> &vpLoopCand,
                        std::vector<KeyFrame *> &vpMergeCand);
  void DetectBestCandidates(KeyFrame *pKF, std::vector<KeyFrame *> &vpLoopCand,
                            std::vector<KeyFrame *> &vpMergeCand,
                            int nMinWords);
  void DetectNBestCandidates(KeyFrame *pKF, std::vector<KeyFrame *> &vpLoopCand,
                             std::vector<KeyFrame *> &vpMergeCand,
                             int nNumCandidates);

  // Relocalization
  std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F, Map *pMap);

  void PreSave();
  void PostLoad(std::map<long unsigned int, KeyFrame *> mpKFid);
  void SetORBVocabulary(ORBVocabulary *pORBVoc);

protected:
  // Associated vocabulary
  const ORBVocabulary *mpVoc;

  // Inverted file
  std::vector<std::list<KeyFrame *>> mvInvertedFile;

  // For save relation without pointer, this is necessary for save/load function
  std::vector<std::list<long unsigned int>> mvBackupInvertedFileId;

  // Mutex
  std::mutex mMutex;
};

} // namespace ORB_SLAM3

#endif // KEYFRAME_DATABASE_H_