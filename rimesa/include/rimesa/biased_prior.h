#pragma once
/** @brief Implementation of Biased Priors used by the rimesa algorithm to
 * enforce equality of shared variables between robots in a multi-robot team.
 *
 * @author Dan McGann
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <risam/GraduatedFactor.h>
#include <risam/GraduatedKernel.h>

#include <optional>

#include "rimesa/types.h"

namespace rimesa {
namespace biased_priors {
/**
 * ########     ###     ######  ########
 * ##     ##   ## ##   ##    ## ##
 * ##     ##  ##   ##  ##       ##
 * ########  ##     ##  ######  ######
 * ##     ## #########       ## ##
 * ##     ## ##     ## ##    ## ##
 * ########  ##     ##  ######  ########
 */

/// @brief Type for a generic constraint function
typedef std::function<gtsam::Vector(const gtsam::Value&, const gtsam::Value&, boost::optional<gtsam::Matrix&>)>
    ConstraintFunction;

/// @brief Type for a function that interpolates a new shared estimate
typedef std::function<boost::shared_ptr<gtsam::Value>(const gtsam::Value&, const gtsam::Value&)>
    SharedEstimateInterpolator;

/// @brief The types of biased priors we can use
enum BiasedPriorType { GEODESIC, SPLIT };
/// @brief Converts string form of BiasedPriorType to enumeration type
BiasedPriorType BPTypeFromString(const std::string& str);
/// @brief Converts enumeration of BiasedPriorType to string representation
std::string BPTypeToString(const BiasedPriorType& type);

/** @brief The values required by the based prior
 * These values are modified externally (during robot communication) and therefore
 * they are stored in biased priors as a pointer and accessed during relinearlization.
 */
struct BiasedPriorInfo {
  /** TYPES **/
  typedef std::shared_ptr<BiasedPriorInfo> shared_ptr;
  typedef std::shared_ptr<const BiasedPriorInfo> const_ptr;

  /** FIELDS **/
  /// @brief The other robot that we shared associated variable with that induces this BP
  rimesa::RobotId other_robot;
  /// @brief The current shared estimate for this biased prior ($z$)
  boost::shared_ptr<gtsam::Value> shared_estimate;
  /// @brief The current shared estimate used for linearization for this biased prior ($z$)
  boost::shared_ptr<gtsam::Value> shared_estimate_lin_point;
  /// @brief The dual term accumulated for the biased prior ($\lambda$)
  gtsam::Vector dual;
  /// @brief The dual term accumulated for linearization for the biased prior ($\lambda$)
  gtsam::Vector dual_lin_point;
  /// @brief The penalty term accumulated for the biased prior ($\beta$)
  double penalty;

  /** FUNCTIONS **/
  /// @brief Computes the shared estimate update for the biased prior $z = interpolate(x_i, x_j)$
  SharedEstimateInterpolator interpolate;
  /// @brief Computes the constraint function for the biased prior $q(x,z)$
  ConstraintFunction constraint_func;

  /** INTERFACE **/
  /// @brief default constructor
  BiasedPriorInfo() = default;
  /// @brief Constructor from initial est and initial penalty, dual and functions are auto-populated
  BiasedPriorInfo(const rimesa::RobotId& other_robot, const BiasedPriorType& type, const gtsam::Value& init_estimate,
                  double init_penalty);

  /// @brief Output to string
  friend std::ostream& operator<<(std::ostream& o, BiasedPriorInfo const& bpi);
};

/** @brief A biased prior is the combination of Augmented Lagrangian Dual and Penalty Terms
 * We augment the factor-graph with these factors to enforce equality constraints.
 */
class BiasedPrior {
  /** TYPES **/
 public:
  typedef boost::shared_ptr<BiasedPrior> shared_ptr;

  /** FIELDS **/
 protected:
  /// @brief Pointer to struct containing current edge and dual variables
  BiasedPriorInfo::shared_ptr vals_ptr_;

  /** INTERFACE **/
 public:
  /** @brief Constructor for base BiasedPrior
   * @param vals_ptr: Pointer to the structure containing the current edge and dual variables for this biased prior
   */
  BiasedPrior(const BiasedPriorInfo::shared_ptr& vals_ptr) : vals_ptr_(vals_ptr) {}

  /// @brief Const access for internal biased_prior info
  const BiasedPriorInfo::const_ptr info() const { return vals_ptr_; }
};

/**
 *  ######   ########  #######  ########  ########  ######  ####  ######
 * ##    ##  ##       ##     ## ##     ## ##       ##    ##  ##  ##    ##
 * ##        ##       ##     ## ##     ## ##       ##        ##  ##
 * ##   #### ######   ##     ## ##     ## ######    ######   ##  ##
 * ##    ##  ##       ##     ## ##     ## ##             ##  ##  ##
 * ##    ##  ##       ##     ## ##     ## ##       ##    ##  ##  ##    ##
 *  ######   ########  #######  ########  ########  ######  ####  ######
 */
/// @brief Implementation of a Biased Prior using a Geodesic Constraint Function
template <class VAR_TYPE>
class GeodesicBiasedPrior : public BiasedPrior, public gtsam::NoiseModelFactor1<VAR_TYPE> {
 public:
  /// @brief Constructor for GeodesicBiasedPrior. For params see BiasedPrior
  GeodesicBiasedPrior(const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
                      const gtsam::SharedNoiseModel& noise_model)
      : BiasedPrior(vals_ptr), gtsam::NoiseModelFactor1<VAR_TYPE>(noise_model, key) {}

  /// @brief The non-squared biased prior error $\sqrt{\beta} q(\theta) - \frac{\lambda}{\beta}$
  gtsam::Vector evaluateError(const VAR_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const override;

  /// @brief The constraint function error $q(x)$
  static gtsam::Vector constraintError(const VAR_TYPE& x, const VAR_TYPE& z,
                                       boost::optional<gtsam::Matrix&> Hx = boost::none);
  /// @brief The constraint function error $q(x)$ that accepts a generic value
  static gtsam::Vector genericConstraintError(const gtsam::Value& x, const gtsam::Value& z,
                                              boost::optional<gtsam::Matrix&> Hx = boost::none);
};

/**
 *  ######  ########  ##       #### ########
 * ##    ## ##     ## ##        ##     ##
 * ##       ##     ## ##        ##     ##
 *  ######  ########  ##        ##     ##
 *       ## ##        ##        ##     ##
 * ##    ## ##        ##        ##     ##
 *  ######  ##        ######## ####    ##
 */
/// @brief Implementation of a Biased Prior using a SPLIT Constraint Function
template <class VAR_TYPE>
class SplitBiasedPrior : public BiasedPrior, public gtsam::NoiseModelFactor1<VAR_TYPE> {
 public:
  /// @brief Constructor for SplitBiasedPrior. For params see BiasedPrior
  SplitBiasedPrior(const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
                   const gtsam::SharedNoiseModel& noise_model)
      : BiasedPrior(vals_ptr), gtsam::NoiseModelFactor1<VAR_TYPE>(noise_model, key) {}

  /// @brief The non-squared biased prior error $\sqrt{\beta} q(\theta) - \frac{\lambda}{\beta}$
  gtsam::Vector evaluateError(const VAR_TYPE& val, boost::optional<gtsam::Matrix&> H = boost::none) const override;

  /// @brief The constraint function error $q(x)$
  static gtsam::Vector constraintError(const VAR_TYPE& x, const VAR_TYPE& z,
                                       boost::optional<gtsam::Matrix&> Hx = boost::none);
  /// @brief The constraint function error $q(x)$ that accepts a generic value
  static gtsam::Vector genericConstraintError(const gtsam::Value& x, const gtsam::Value& z,
                                              boost::optional<gtsam::Matrix&> Hx = boost::none);
};

/**
 * #### ##    ## ######## ######## ########  ########   #######  ##          ###    ######## ########
 *  ##  ###   ##    ##    ##       ##     ## ##     ## ##     ## ##         ## ##      ##    ##
 *  ##  ####  ##    ##    ##       ##     ## ##     ## ##     ## ##        ##   ##     ##    ##
 *  ##  ## ## ##    ##    ######   ########  ########  ##     ## ##       ##     ##    ##    ######
 *  ##  ##  ####    ##    ##       ##   ##   ##        ##     ## ##       #########    ##    ##
 *  ##  ##   ###    ##    ##       ##    ##  ##        ##     ## ##       ##     ##    ##    ##
 * #### ##    ##    ##    ######## ##     ## ##         #######  ######## ##     ##    ##    ########
 */

/** @brief Interpolates a Lie Group Object spherically.
 * @param pa: The start object of interpolation
 * @param pb: The end object of interpolation
 * @param alpha: The interpolation factor
 * @returns The interpolated object
 */
template <class LIE_TYPE>
LIE_TYPE baseInterpolateSLERP(const LIE_TYPE& pa, const LIE_TYPE& pb, double alpha);

/** @brief Interpolates a POSE Object: linearly for its translation and spherically for its rotation
 * @param pa: The start pose of interpolation
 * @param pb: The end pose of interpolation
 * @param alpha: The interpolation factor
 * @returns The interpolated pose
 */
template <class POSE_TYPE>
POSE_TYPE baseInterpolateSPLIT(const POSE_TYPE& start_pose, const POSE_TYPE& end_pose, double alpha);

/** @brief Interpolates the generic values to compute a new shared estimate.
 * Actual implementation handled in template specialization functions
 * @param this_estimate: The current solution to the variable held by this robot
 * @param other_estimate: The current solution to the variable held by the other robot
 * @returns The new shared estimate (z) held jointly by both agents
 */
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_vector(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_point2(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_point3(const gtsam::Value& this_estimate,
                                                                   const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_pose2(const gtsam::Value& this_estimate,
                                                                  const gtsam::Value& other_estimate);
boost::shared_ptr<gtsam::Value> interpolate_shared_estimate_pose3(const gtsam::Value& this_estimate,
                                                                  const gtsam::Value& other_estimate);

/**
 *  ######   ########     ###    ########          ########     ###    ########     ###    ##     ##  ######
 * ##    ##  ##     ##   ## ##   ##     ##         ##     ##   ## ##   ##     ##   ## ##   ###   ### ##    ##
 * ##        ##     ##  ##   ##  ##     ##         ##     ##  ##   ##  ##     ##  ##   ##  #### #### ##
 * ##   #### ########  ##     ## ##     ## ####### ########  ##     ## ########  ##     ## ## ### ##  ######
 * ##    ##  ##   ##   ######### ##     ##         ##        ######### ##   ##   ######### ##     ##       ##
 * ##    ##  ##    ##  ##     ## ##     ##         ##        ##     ## ##    ##  ##     ## ##     ## ##    ##
 *  ######   ##     ## ##     ## ########          ##        ##     ## ##     ## ##     ## ##     ##  ######
 */
/// @brief The parameters to use with the SIGKernel for Graduated Biased Priors
/// NOTE: This custom parameterization makes it easier to adjust shape params according to dimensionality
struct GraduatedBiasedPriorParams {
  /// @brief The Chi2 threshold to use for Chi2-based shape parameter computation
  double chi2_threshold{0.9};
  /// @brief The update strategy to use for mu updates
  risam::SIGKernel::MuUpdateStrategy mu_update_strat{risam::SIGKernel::MuUpdateStrategy::STABLE};
  /// @brief The update strategy to use for mu_init increments
  risam::SIGKernel::MuInitIncrementStrategy mu_init_inc_strat{risam::SIGKernel::MuInitIncrementStrategy::EQUAL_5};

  /// @brief Parameterized Constructor
  GraduatedBiasedPriorParams(double chi2_threshold, risam::SIGKernel::MuUpdateStrategy mu_update_strat,
                             risam::SIGKernel::MuInitIncrementStrategy mu_init_inc_strat)
      : chi2_threshold(chi2_threshold), mu_update_strat(mu_update_strat), mu_init_inc_strat(mu_init_inc_strat) {}
};

/**
 * ########    ###     ######  ########  #######  ########  ##    ##
 * ##         ## ##   ##    ##    ##    ##     ## ##     ##  ##  ##
 * ##        ##   ##  ##          ##    ##     ## ##     ##   ####
 * ######   ##     ## ##          ##    ##     ## ########     ##
 * ##       ######### ##          ##    ##     ## ##   ##      ##
 * ##       ##     ## ##    ##    ##    ##     ## ##    ##     ##
 * ##       ##     ##  ######     ##     #######  ##     ##    ##
 */

/** @brief Constructs a biased prior factor.
 * Factors are templatized, but we want to construct biased priors without templatizing the RISAM class.
 * So that the class can handle generic shared measurements of any type. therefore we need to deduce the type at runtime
 * Note: no macro usage to make things easier to read (though much longer code wise)
 * @param type: The type of biased prior to construct
 * @param vals_ptr: The BPInfo that this biased prior will reference
 * @param biased_prior_noise_model_sigmas: (rotation, translation) the weights with which to construct the noise model
 * of the biased prior. If none the noise model defaults to identity. Also note the weights are only used for POSE(2/3)
 * biased priors.
 * @param kernel_params: The params for the SIGKernel to wrap around this biased prior. If provided will construct a
 * graduated biased prior
 * @returns A biased prior factor
 */
gtsam::NonlinearFactor::shared_ptr factory(
    const BiasedPriorType& type, const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
    const std::optional<std::pair<double, double>>& biased_prior_noise_model_sigmas = std::nullopt,
    const std::optional<GraduatedBiasedPriorParams>& kernel_params = std::nullopt);

/// @brief Constructs a biased prior factor templatized
template <template <typename> typename BIASED_PRIOR>
gtsam::NonlinearFactor::shared_ptr factory(
    const gtsam::Key& key, const BiasedPriorInfo::shared_ptr& vals_ptr,
    const std::optional<std::pair<double, double>>& biased_prior_noise_model_sigmas = std::nullopt);

/// @brief Constructs a graduated biased prior factor templatized
template <template <typename> typename BIASED_PRIOR>
gtsam::NonlinearFactor::shared_ptr graduated_factory(
    const risam::SIGKernel::Parameters& bp_sig_kernel_params, const gtsam::Key& key,
    const BiasedPriorInfo::shared_ptr& vals_ptr,
    const std::optional<std::pair<double, double>>& biased_prior_noise_model_sigmas);

/** @brief Constructs a shared estimate interpolator function
 * Each shared variable needs its own interpolator function as the interpolation depends on type so we use this
 * factory to construct the corresponding interpolator for the given biased prior
 */
SharedEstimateInterpolator interpolator_factory(const boost::shared_ptr<gtsam::Value>& val);

/** @brief Constructs the appropriate constraint function for the value
 * To update the dual variable we need access to the constraint function q.
 */
ConstraintFunction constraint_func_factory(const BiasedPriorType& type, const boost::shared_ptr<gtsam::Value>& val);

}  // namespace biased_priors
}  // namespace rimesa

#include "rimesa/biased_prior-inl.h"