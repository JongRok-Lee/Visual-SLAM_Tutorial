#include "myslam/mappoint.hpp"
#include "myslam/feature.hpp"

namespace myslam {

void MapPoint::removeObservation(std::shared_ptr<Feature> feature) {
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

} // namespace myslam