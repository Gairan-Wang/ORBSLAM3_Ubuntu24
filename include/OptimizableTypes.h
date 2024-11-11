#ifndef OPTIMIZABLE_TYPES_H
#define OPTIMIZABLE_TYPES_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/sim3.h>

#include "Camera/GeometricCamera.h"
#include <Eigen/Geometry>

namespace ORB_SLAM3 {
class EdgeSE3ProjectXYZOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyPose() {}

  bool read(std::istream &is);

  bool write(std::ostream &os) const;

  void computeError() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs - pCamera->project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    return (v1->estimate().map(Xw))(2) > 0.0;
  }

  virtual void linearizeOplus();

  Eigen::Vector3d Xw;
  GeometricCamera *pCamera;
};

class EdgeSE3ProjectXYZOnlyPoseToBody
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyPoseToBody() {}

  bool read(std::istream &is);

  bool write(std::ostream &os) const;

  void computeError() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs - pCamera->project((mTrl * v1->estimate()).map(Xw));
  }

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
    return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
  }

  virtual void linearizeOplus();

  Eigen::Vector3d Xw;
  GeometricCamera *pCamera;

  g2o::SE3Quat mTrl;
};

class EdgeSE3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                 g2o::VertexSE3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ();

  bool read(std::istream &is);

  bool write(std::ostream &os) const;

  void computeError() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
    const g2o::VertexPointXYZ *v2 =
        static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error = obs - pCamera->project(v1->estimate().map(v2->estimate()));
  }

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
    const g2o::VertexPointXYZ *v2 =
        static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
    return ((v1->estimate().map(v2->estimate()))(2) > 0.0);
  }

  virtual void linearizeOplus();

  GeometricCamera *pCamera;
};

class EdgeSE3ProjectXYZToBody
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                 g2o::VertexSE3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZToBody();

  bool read(std::istream &is);

  bool write(std::ostream &os) const;

  void computeError() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
    const g2o::VertexPointXYZ *v2 =
        static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
    Eigen::Vector2d obs(_measurement);
    _error =
        obs - pCamera->project((mTrl * v1->estimate()).map(v2->estimate()));
  }

  bool isDepthPositive() {
    const g2o::VertexSE3Expmap *v1 =
        static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
    const g2o::VertexPointXYZ *v2 =
        static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);
    return ((mTrl * v1->estimate()).map(v2->estimate()))(2) > 0.0;
  }

  virtual void linearizeOplus();

  GeometricCamera *pCamera;
  g2o::SE3Quat mTrl;
};

class VertexSim3Expmap : public g2o::BaseVertex<7, g2o::Sim3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSim3Expmap();
  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  virtual void setToOriginImpl() { _estimate = g2o::Sim3(); }

  virtual void oplusImpl(const double *update_) {
    Eigen::Map<g2o::Vector7> update(const_cast<double *>(update_));

    if (_fix_scale)
      update[6] = 0;

    g2o::Sim3 s(update);
    setEstimate(s * estimate());
  }

  GeometricCamera *pCamera1, *pCamera2;

  bool _fix_scale;
};

class EdgeSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                 ORB_SLAM3::VertexSim3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSim3ProjectXYZ();
  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  void computeError() {
    const ORB_SLAM3::VertexSim3Expmap *v1 =
        static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
    const g2o::VertexPointXYZ *v2 =
        static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);

    Eigen::Vector2d obs(_measurement);
    _error = obs - v1->pCamera1->project(v1->estimate().map(v2->estimate()));
  }

  // virtual void linearizeOplus();
};

class EdgeInverseSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXYZ,
                                 VertexSim3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeInverseSim3ProjectXYZ();
  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  void computeError() {
    const ORB_SLAM3::VertexSim3Expmap *v1 =
        static_cast<const ORB_SLAM3::VertexSim3Expmap *>(_vertices[1]);
    const g2o::VertexPointXYZ *v2 =
        static_cast<const g2o::VertexPointXYZ *>(_vertices[0]);

    Eigen::Vector2d obs(_measurement);
    _error = obs - v1->pCamera2->project(
                       (v1->estimate().inverse().map(v2->estimate())));
  }

  // virtual void linearizeOplus();
};
} // namespace ORB_SLAM3

#endif // OPTIMIZABLE_TYPES_H