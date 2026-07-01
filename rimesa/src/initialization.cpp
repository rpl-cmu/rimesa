/** @brief Implementation initialization functionality
 *
 * @author Dan McGann
 */

#include "rimesa/initialization.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <risam/GraduatedFactor.h>

namespace rimesa {
namespace init {
/*********************************************************************************************************************/
boost::shared_ptr<gtsam::Value> computeInitialization(const gtsam::Value& local_est, const gtsam::Value& other_est,
                                                      Observability obs, InitializationStrategy strat) {
  // Case 1 Other - return other est
  if (strat == InitializationStrategy::OTHER) {
    return other_est.clone();
  }
  // Case 2 Local - return local est
  else if (strat == InitializationStrategy::LOCAL) {
    return local_est.clone();
  }
  // Case 3 Unobservable - merge the estimates based on observability
  else if (strat == InitializationStrategy::UNOBSERVABLE) {
    // Handle simple cases
    if (obs == Observability::NONE) {
      // The local measurement cannot directly observe any of the state so use the other estimate
      return other_est.clone();
    } else if (obs == Observability::ALL) {
      // The local measurement observes all state use the local estimate
      return local_est.clone();
    } else {
      if (typeid(local_est) == typeid(gtsam::GenericValue<gtsam::Pose2>)) {
        return computePartialObsPoseInit<gtsam::Pose2>(local_est, other_est, obs);
      } else if (typeid(local_est) == typeid(gtsam::GenericValue<gtsam::Pose3>)) {
        return computePartialObsPoseInit<gtsam::Pose3>(local_est, other_est, obs);
      } else {
        throw std::runtime_error(
            "rimesa::init::computeInitialization encountered invalid Observability as partial observability  (ROT, "
            "POS) is only possible for pose types.");
      }
    }
  } else {
    throw std::runtime_error("rimesa::init::computeInitialization encountered invalid InitializationStrategy.");
  }
}

/*********************************************************************************************************************/
TypeMap<Observability> defaultFactorObservability() {
  // clang-format off
  TypeMap<Observability>  factor_observability = {
  // Priors
  {typeid(gtsam::PriorFactor<gtsam::Vector>),                                                       Observability::ALL},
  {typeid(gtsam::PriorFactor<gtsam::Pose2>),                                                        Observability::ALL},
  {typeid(gtsam::PriorFactor<gtsam::Vector2>),                                                      Observability::ALL},
  {typeid(gtsam::PriorFactor<gtsam::Vector3>),                                                      Observability::ALL},
  {typeid(gtsam::PriorFactor<gtsam::Pose3>),                                                        Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::PriorFactor<gtsam::Vector>>),                        Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::PriorFactor<gtsam::Pose2>>),                         Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::PriorFactor<gtsam::Vector2>>),                       Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::PriorFactor<gtsam::Vector3>>),                       Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::PriorFactor<gtsam::Pose3>>),                         Observability::ALL},
  // Between                
  {typeid(gtsam::BetweenFactor<gtsam::Vector>),                                                     Observability::ALL},
  {typeid(gtsam::BetweenFactor<gtsam::Vector2>),                                                    Observability::ALL},
  {typeid(gtsam::BetweenFactor<gtsam::Vector3>),                                                    Observability::ALL},
  {typeid(gtsam::BetweenFactor<gtsam::Pose2>),                                                      Observability::ALL},
  {typeid(gtsam::BetweenFactor<gtsam::Pose3>),                                                      Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::BetweenFactor<gtsam::Vector>>),                      Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::BetweenFactor<gtsam::Vector2>>),                     Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::BetweenFactor<gtsam::Vector3>>),                     Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::BetweenFactor<gtsam::Pose2>>),                       Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::BetweenFactor<gtsam::Pose3>>),                       Observability::ALL},
  // Range                
  {typeid(gtsam::RangeFactor<gtsam::Vector>),                                                       Observability::NONE},
  {typeid(gtsam::RangeFactor<gtsam::Vector2>),                                                      Observability::NONE},
  {typeid(gtsam::RangeFactor<gtsam::Vector3>),                                                      Observability::NONE},
  {typeid(gtsam::RangeFactor<gtsam::Pose2>),                                                        Observability::NONE},
  {typeid(gtsam::RangeFactor<gtsam::Pose3>),                                                        Observability::NONE},
  {typeid(gtsam::RangeFactor<gtsam::Pose2, gtsam::Vector2>),                                        Observability::NONE},
  {typeid(gtsam::RangeFactor<gtsam::Pose3, gtsam::Vector3>),                                        Observability::NONE},
  {typeid(risam::GenericGraduatedFactor<gtsam::RangeFactor<gtsam::Vector>>),                        Observability::NONE},
  {typeid(risam::GenericGraduatedFactor<gtsam::RangeFactor<gtsam::Vector2>>),                       Observability::NONE},
  {typeid(risam::GenericGraduatedFactor<gtsam::RangeFactor<gtsam::Vector3>>),                       Observability::NONE},
  {typeid(risam::GenericGraduatedFactor<gtsam::RangeFactor<gtsam::Pose2>>),                         Observability::NONE},
  {typeid(risam::GenericGraduatedFactor<gtsam::RangeFactor<gtsam::Pose3>>),                         Observability::NONE},
  {typeid(risam::GenericGraduatedFactor<gtsam::RangeFactor<gtsam::Pose2, gtsam::Vector2>>),         Observability::NONE},
  {typeid(risam::GenericGraduatedFactor<gtsam::RangeFactor<gtsam::Pose3, gtsam::Vector3>>),         Observability::NONE},
  // Bearing-Range
  {typeid(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>),                                   Observability::POSITION},
  {typeid(gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>),                                   Observability::POSITION},
  {typeid(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Vector2>),                                 Observability::ALL},
  {typeid(gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Vector3>),                                 Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>>),    Observability::POSITION},
  {typeid(risam::GenericGraduatedFactor<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>>),    Observability::POSITION},
  {typeid(risam::GenericGraduatedFactor<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Vector2>>),  Observability::ALL},
  {typeid(risam::GenericGraduatedFactor<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Vector3>>),  Observability::ALL},
  };

  // clang-format on
  return factor_observability;
}

}  // namespace init
}  // namespace rimesa
