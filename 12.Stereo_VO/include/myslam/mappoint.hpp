#ifndef MYSLAM_MAPPOINT_HPP
#define MYSLAM_MAPPOINT_HPP

#include "myslam/common_include.hpp"
#include "myslam/feature.hpp"
#include "myslam/frame.hpp"

namespace myslam {
struct Feature;

struct MapPoint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  typedef std::shared_ptr<MapPoint> Ptr;

  MapPoint() = default;
  MapPoint(long id, const Eigen::Vec3 &position) : id_(id), pos_(position) {}

  Eigen::Vec3 getPos() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return pos_;
  }

  void setPos(const Eigen::Vec3 &pos) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    pos_ = pos;
  }

  void addObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
  }

  void removeObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end(); iter++) {
      if (iter->lock() == feature) {
        observations_.erase(iter);
        feature->map_point_.reset();
        observed_times_--;
        break;
      }
    }
  }

  std::list<std::weak_ptr<Feature>> getObservations() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return observations_;
  }

  static MapPoint::Ptr createMapPoint() {
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint = std::make_shared<MapPoint>();
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
  }

public:
  unsigned long id_ = 0;
  bool is_outlier_ = false;
  Eigen::Vec3 pos_ = Eigen::Vec3::Zero();
  std::mutex data_mutex_;
  int observed_times_ = 0; // being observed by feature matching algorithm
  std::list<std::weak_ptr<Feature>> observations_;

};
} // namespace myslam

#endif // MYSLAM_MAPPOINT_HPP
