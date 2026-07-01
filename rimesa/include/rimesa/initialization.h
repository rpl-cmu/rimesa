#pragma once
/** @brief Implementation of initialization strategies that riMESA uses for external shared variables
 *
 * @author Dan McGann
 */
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Value.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

namespace rimesa {
namespace init {

/**
 * ######## ##    ## ########  ########  ######
 *    ##     ##  ##  ##     ## ##       ##    ##
 *    ##      ####   ##     ## ##       ##
 *    ##       ##    ########  ######    ######
 *    ##       ##    ##        ##             ##
 *    ##       ##    ##        ##       ##    ##
 *    ##       ##    ##        ########  ######
 */

/// @brief Enum for the types of observability a measurement has on its observed variables
/// i.e. A Local estimate will only be able to recover the specified information
enum Observability {
  NONE,      // The measurement observes none of the full state
  POSITION,  // The measurement observes only the position of the target variable
  ROTATION,  // The measurement observes only the rotation of the target variable
  ALL        // The measurement observes the entire state of the target variable
};

/// @brief Strategies for initializing a variable estimate given a local estimate and another solution
enum InitializationStrategy {
  OTHER,         // Use the value from the other solution
  UNOBSERVABLE,  // Use the other solution for all unobservable components
  LOCAL          // Use the local estimate
};

/// @brief Type shortcut to std::type_info reference
using TypeInfoRef = std::reference_wrapper<const std::type_info>;

/// @brief The hash function for TypeInfoRef objects
/// NOTE: I have no idea why this is not auto set to the default hash function for std::type_info
/// Ref: https://en.cppreference.com/w/cpp/types/type_info/hash_code
struct TypeHash {
  std::size_t operator()(const TypeInfoRef type_info) const { return type_info.get().hash_code(); }
};

/// @brief The equality function for TypeInfoRef objects
/// NOTE: I have no idea why this is not auto set to the default hash function for std::type_info
/// Ref: https://en.cppreference.com/w/cpp/types/type_info/hash_code
struct TypeEqual {
  bool operator()(TypeInfoRef lhs, TypeInfoRef rhs) const { return lhs.get() == rhs.get(); }
};

/// @brief Type for maps from TypeInfoRef to VAL_TYPE
template <typename VAL_TYPE>
using TypeMap = std::unordered_map<TypeInfoRef, VAL_TYPE, TypeHash, TypeEqual>;

/**
 * ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##  ######
 * ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ## ##    ##
 * ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ## ##
 * ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##  ######
 * ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####       ##
 * ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ### ##    ##
 * ##        #######  ##    ##  ######     ##    ####  #######  ##    ##  ######
 */

/** @brief Computes initialization of pose values from partial observability
 * @param local_est: The local estimate of the value
 * @param other_est: The other estimate of the value (i.e. from another robot)
 * @param obs: The local observability of the value
 * @returns The initialization for the pose
 */
template <class POSE_TYPE>
boost::shared_ptr<gtsam::Value> computePartialObsPoseInit(const gtsam::Value& local_est, const gtsam::Value& other_est,
                                                          Observability obs) {
  POSE_TYPE local_pose = dynamic_cast<const gtsam::GenericValue<POSE_TYPE>&>(local_est).value();
  POSE_TYPE other_pose = dynamic_cast<const gtsam::GenericValue<POSE_TYPE>&>(other_est).value();

  if (obs == Observability::POSITION) {
    // Retrieve the rotation from the other estimate
    return boost::make_shared<gtsam::GenericValue<POSE_TYPE>>(
        POSE_TYPE(other_pose.rotation(), local_pose.translation()));
  } else if (obs == Observability::ROTATION) {
    // Retrieve the translation from the other robot
    return boost::make_shared<gtsam::GenericValue<POSE_TYPE>>(
        POSE_TYPE(local_pose.rotation(), other_pose.translation()));
  } else {
    throw std::runtime_error("rimesa::init::computeInitialization unknown Observability provided.");
  }
}

/** @brief Computes the initialization for a variable using the provided strategy and observability
 * @param local_est: The local estimate of the value
 * @param other_est: The other estimate of the value (i.e. from another robot)
 * @param obs: The local observability of the value
 * @param strat: The initialization strategy to use to initialize this value
 * @returns The initialization for the value
 */
boost::shared_ptr<gtsam::Value> computeInitialization(const gtsam::Value& local_est, const gtsam::Value& other_est,
                                                      Observability obs, InitializationStrategy strat);

/// @brief The observability definitions for default factors
/// NOTE: Observability is for the "target" variable
/// i.e. the second variable in the factor as that is the observability that matters for multi-robot problems
TypeMap<Observability> defaultFactorObservability();

}  // namespace init
}  // namespace rimesa