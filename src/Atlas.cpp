#include "Atlas.h"
#include "GeometricCamera.h"
#include "KannalaBrandt8.h"
#include "Pinhole.h"
#include "Viewer.h"

namespace ORB_SLAM3 {

Atlas::Atlas() { mpCurrentMap = static_cast<Map *>(NULL); }

Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid) {
  mpCurrentMap = static_cast<Map *>(NULL);
  CreateNewMap();
}

Atlas::~Atlas() {
  for (std::set<Map *>::iterator it = mspMaps.begin(), end = mspMaps.end();
       it != end;) {
    Map *pMi = *it;

    if (pMi) {
      delete pMi;
      pMi = static_cast<Map *>(NULL);

      it = mspMaps.erase(it);
    } else
      ++it;
  }
}

void Atlas::CreateNewMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  std::cout << "Creation of new map with id: " << Map::nNextId << std::endl;
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1; // The init KF is the next of current maximum

    mpCurrentMap->SetStoredMap();
    std::cout << "Stored map with ID: " << mpCurrentMap->GetId() << std::endl;
  }
  std::cout << "Creation of new map with last KF id: " << mnLastInitKFidMap
            << std::endl;

  mpCurrentMap = new Map(mnLastInitKFidMap);
  mpCurrentMap->SetCurrentMap();
  mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map *pMap) {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  std::cout << "Change to map with id: " << pMap->GetId() << std::endl;
  if (mpCurrentMap) {
    mpCurrentMap->SetStoredMap();
  }

  mpCurrentMap = pMap;
  mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mnLastInitKFidMap;
}

/**
 * @brief 可以将一个 Viewer 实例与 Atlas 关联，从而使得 Atlas
 * 可以在后续的操作中使用这个 Viewer 来展示或视图相关的功能。
 * @param pViewer The viewer to set
 * @note This function is called by the Viewer class when it is created
 */
void Atlas::SetViewer(Viewer *pViewer) {
  mpViewer = pViewer;
  mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame *pKF) {
  Map *pMapKF = pKF->GetMap();
  pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint *pMP) {
  Map *pMapMP = pMP->GetMap();
  pMapMP->AddMapPoint(pMP);
}

GeometricCamera *Atlas::AddCamera(GeometricCamera *pCam) {
  // Check if the camera already exists
  bool bAlreadyInMap = false;
  int index_cam = -1;
  for (size_t i = 0; i < mvpCameras.size(); ++i) {
    GeometricCamera *pCam_i = mvpCameras[i];
    if (!pCam)
      std::cout << "Not pCam" << std::endl;
    if (!pCam_i)
      std::cout << "Not pCam_i" << std::endl;
    if (pCam->GetType() != pCam_i->GetType())
      continue;

    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      if (((Pinhole *)pCam_i)->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam = i;
      }
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      if (((KannalaBrandt8 *)pCam_i)->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam = i;
      }
    }
  }

  if (bAlreadyInMap) {
    return mvpCameras[index_cam];
  } else {
    mvpCameras.push_back(pCam);
    return pCam;
  }
}

std::vector<GeometricCamera *> Atlas::GetAllCameras() { return mvpCameras; }

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs) {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame *> Atlas::GetAllKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint *> Atlas::GetAllMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint *> Atlas::GetReferenceMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetReferenceMapPoints();
}

/**
 * !@brief 获取所有地图的指针
 * !@return 所有地图的指针
 */
std::vector<Map *> Atlas::GetAllMaps() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  struct compFunctor {
    inline bool operator()(Map *elem1, Map *elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::vector<Map *> vMaps(mspMaps.begin(), mspMaps.end());
  sort(vMaps.begin(), vMaps.end(), compFunctor());
  return vMaps;
}

int Atlas::CountMaps() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mspMaps.size();
}

void Atlas::clearMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->clear();
}

void Atlas::clearAtlas() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end();
  it!=send; it++)
  {
      (*it)->clear();
      delete *it;
  }*/
  //* 删除手动释放内存，使用Set可以自动释放内存
  mspMaps.clear();
  mpCurrentMap = static_cast<Map *>(NULL);
  mnLastInitKFidMap = 0;
}

Map *Atlas::GetCurrentMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  if (!mpCurrentMap)
    CreateNewMap();
  while (mpCurrentMap->IsBad())
    usleep(3000);

  return mpCurrentMap;
}

void Atlas::SetMapBad(Map *pMap) {
  mspMaps.erase(pMap);
  pMap->SetBad();

  mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps() {
  /*for(Map* pMap : mspBadMaps)
  {
      delete pMap;
      pMap = static_cast<Map*>(NULL);
  }*/
  mspBadMaps.clear();
}

bool Atlas::isInertial() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->isImuInitialized();
}

/**
 * !@brief 保存当前地图
 * @param path 保存路径
 * @return 保存成功返回true，否则返回false
 */
void Atlas::PreSave() {
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
      mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                          1; // The init KF is the next of current maximum
  }

  struct compFunctor {
    inline bool operator()(Map *elem1, Map *elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
  sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

  std::set<GeometricCamera *> spCams(mvpCameras.begin(), mvpCameras.end());
  for (Map *pMi : mvpBackupMaps) {
    if (!pMi || pMi->IsBad())
      continue;

    if (pMi->GetAllKeyFrames().size() == 0) {
      // Empty map, erase before of save it.
      SetMapBad(pMi);
      continue;
    }
    pMi->PreSave(spCams);
  }
  RemoveBadMaps();
}

void Atlas::PostLoad() {
  std::map<unsigned int, GeometricCamera *> mpCams;
  for (GeometricCamera *pCam : mvpCameras) {
    mpCams[pCam->GetId()] = pCam;
  }

  mspMaps.clear();
  unsigned long int numKF = 0, numMP = 0;
  for (Map *pMi : mvpBackupMaps) {
    mspMaps.insert(pMi);
    pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
    numKF += pMi->GetAllKeyFrames().size();
    numMP += pMi->GetAllMapPoints().size();
  }
  mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase *pKFDB) {
  mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase *Atlas::GetKeyFrameDatabase() { return mpKeyFrameDB; }

void Atlas::SetORBVocabulary(ORBVocabulary *pORBVoc) {
  mpORBVocabulary = pORBVoc;
}

ORBVocabulary *Atlas::GetORBVocabulary() { return mpORBVocabulary; }

long unsigned int Atlas::GetNumLivedKF() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (Map *pMap_i : mspMaps) {
    num += pMap_i->GetAllKeyFrames().size();
  }

  return num;
}

long unsigned int Atlas::GetNumLivedMP() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  long unsigned int num = 0;
  for (Map *pMap_i : mspMaps) {
    num += pMap_i->GetAllMapPoints().size();
  }

  return num;
}

std::map<long unsigned int, KeyFrame *> Atlas::GetAtlasKeyframes() {
  std::map<long unsigned int, KeyFrame *> mpIdKFs;
  for (Map *pMap_i : mvpBackupMaps) {
    std::vector<KeyFrame *> vpKFs_Mi = pMap_i->GetAllKeyFrames();

    for (KeyFrame *pKF_j_Mi : vpKFs_Mi) {
      mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
    }
  }

  return mpIdKFs;
}

} // namespace ORB_SLAM3