#pragma once
/** @brief Interface of the rimesa algorithm via an individual robot interface.
 * This solver is to be run on-board individual robots in the multi-robot team to
 * collaboratively solve the collaborative simultaneous localization and mapping problem.
 *
 * This algorithm is described in detail in the following publication
 *
 * D. McGann and M. Kaess, "riMESA: Consensus ADMM for Real-World Collaborative SLAM," 
 * in IEEE Transactions on Robotics, 2026
 *
 * @author Dan McGann
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <risam/GraduatedKernel.h>
#include <risam/RISAM.h>

#include "rimesa/biased_prior.h"
#include "rimesa/initialization.h"
#include "rimesa/types.h"

namespace rimesa {

/**
 * ######## ##    ## ########  ########  ######
 *    ##     ##  ##  ##     ## ##       ##    ##
 *    ##      ####   ##     ## ##       ##
 *    ##       ##    ########  ######    ######
 *    ##       ##    ##        ##             ##
 *    ##       ##    ##        ##       ##    ##
 *    ##       ##    ##        ########  ######
 */

/// @brief Container for sharing new shared variables between robots
struct DeclaredVariables {
  /// @brief The set shared variables known by this robot
  gtsam::KeySet shared_variables;
  /// @brief List of all global variables observed, which may be shared
  gtsam::KeySet observed_global_variables;
};

/// @brief The data communicated between two rimesa agents
struct CommunicationData {
  typedef std::shared_ptr<CommunicationData> shared_ptr;
  /// @brief The current solution to all variables shared between robots
  gtsam::Values shared_var_solution;
  /// @brief The variables that this robot needs to initialize from the other robot and their local observability
  std::map<gtsam::Key, init::Observability> vars_requested_for_init;
};

/// @brief The cache of riMESA state data required to perform a Non-Blocking Communication
struct CommunicationStateCache {
  /// @brief The stamp at which the communication was initiated.
  const size_t start_stamp;
  /// @brief The id of the robot with which we are communicating
  const RobotId other_robot;
  /// @brief The set variables known to be shared with other_robot
  const gtsam::KeySet shared_variables;
  /// @brief The current set of observed global variables, some of which may be shared with other_robot
  const gtsam::KeySet observed_global_variables;
  /// @brief The current estimate for all values
  const gtsam::Values current_estimate;
  /// @brief The current set of variables that we need to get from other robots for initialization
  const std::map<gtsam::Key, RobotId> shared_initialization_variables;
  /// @brief The local observability of variables marked to be initialized by their owner robot
  const std::map<gtsam::Key, init::Observability> shared_init_var_observability;
  /// @brief The variable name mappings for `other_robot`
  std::optional<std::map<gtsam::Key, gtsam::Key>> outgoing_variable_map{std::nullopt};
  std::optional<std::map<gtsam::Key, gtsam::Key>> incoming_variable_map{std::nullopt};

  CommunicationStateCache(
      const size_t start_stamp, const RobotId other_robot, const gtsam::KeySet shared_variables,
      const gtsam::KeySet observed_global_variables, const gtsam::Values current_estimate,
      const std::map<gtsam::Key, RobotId> shared_initialization_variables,
      const std::map<gtsam::Key, init::Observability> shared_init_var_observability,
      const std::optional<std::pair<std::map<gtsam::Key, gtsam::Key>, std::map<gtsam::Key, gtsam::Key>>>
          variable_name_maps)
      : start_stamp(start_stamp),
        other_robot(other_robot),
        shared_variables(shared_variables),
        observed_global_variables(observed_global_variables),
        current_estimate(current_estimate),
        shared_initialization_variables(shared_initialization_variables),
        shared_init_var_observability(shared_init_var_observability) {
    if (variable_name_maps) {
      outgoing_variable_map = variable_name_maps->first;
      incoming_variable_map = variable_name_maps->second;
    }
  }
};

/// @brief Data required to update the internal state after a successful communication
struct CommunicationResult {
  /// @brief The stamp when this communication started
  size_t start_stamp;
  /// @brief The id of the robot with which the communication occurred
  RobotId other_robot;
  /// @brief The set of new shared variables identified during this communication
  gtsam::KeySet new_shared_vars;
  /// @brief The total set of shared variables known to both robots after this communication
  gtsam::KeySet known_shared_vars;
  /// @brief The variables we sent to the other robot for initialization
  std::map<gtsam::Key, init::Observability> vars_sent_for_initialization;
  /// @brief The variables for which we have received initialization values from the other robot
  std::map<gtsam::Key, init::Observability> vars_received_for_initialization;
  /// @brief The solution we sent to the other robot
  gtsam::Values sent_solution;
  /// @brief The other robot's solution to all shared variables in known_shared_vars
  gtsam::Values received_solution;
};

/**
 * ########  #### ##     ## ########  ######     ###
 * ##     ##  ##  ###   ### ##       ##    ##   ## ##
 * ##     ##  ##  #### #### ##       ##        ##   ##
 * ########   ##  ## ### ## ######    ######  ##     ##
 * ##   ##    ##  ##     ## ##             ## #########
 * ##    ##   ##  ##     ## ##       ##    ## ##     ##
 * ##     ## #### ##     ## ########  ######  ##     ##
 */

/// @brief Configuration parameters for the rimesa algo, all agents MUST use the same params
struct RIMESAParams {
  /// @brief The type of biased prior to use for pose variables
  /// Note: Using SPLIT was partially explored but not rigorously evaluated or discussed in [2]. Use with caution.
  biased_priors::BiasedPriorType pose_biased_prior_type{biased_priors::BiasedPriorType::GEODESIC};
  /// @brief The initial penalty parameter used before the shared variable is initialized (should be very small)
  double initial_penalty_param{0.001};
  /// @brief The constant penalty parameter to use for biased priors of initialzed shared variables
  double penalty_param{1};
  /// @brief The threshold for a change in shared estimate or dual variable value required to force reelimination
  /// Note: Setting  > 0.0 was partially explored, but not rigorously evaluated or discussed in [2]. Use with caution.
  double shared_var_wildfire_threshold{0.0};
  /// @brief The sigmas to use for biased prior noise models on poses (rotation_sigma, translation sigma)
  std::optional<std::pair<double, double>> biased_prior_noise_model_sigmas{{0.1, 1.0}};
  /// @brief The decay rate for dual variables Expect: [0.0 - 1.0]
  /// The current dual variable is multiplied by this value before adding the update to provide greater weight to new
  /// dual variables
  double dual_decay_rate{0.9};
  /// @brief The strategy to use to initialize external shared variables
  init::InitializationStrategy external_shared_var_init_strat{init::InitializationStrategy::UNOBSERVABLE};
  /// @brief The kernel parameters to use for graduated biased priors.
  /// If provided riMESA will wrap biased priors with graduated kernels
  std::optional<biased_priors::GraduatedBiasedPriorParams> bp_kernel_params{
      std::in_place, 0.5, risam::SIGKernel::MuUpdateStrategy::STABLE,
      risam::SIGKernel::MuInitIncrementStrategy::EQUAL_5};
};

/// @brief Implementation of the rimesa Algorithm
class RIMESA {
  /** FIELDS **/
 protected:
  /// @brief The unique identifier for this agent
  RobotId robot_id_;
  /// @brief The current solution of the smoother
  gtsam::Values current_estimate_;
  /// @brief The hyper-parameters for the rimesa algorithm
  RIMESAParams params_;
  /// @brief The underlying incremental smoother instance
  risam::RISAM smoother_;

  /// @brief The set of all observed global variables
  gtsam::KeySet observed_global_variables_;
  /// @brief Mapping from other agents to all variables shared between this and the other agent
  std::map<RobotId, gtsam::KeySet> robot_shared_vars_;
  /// @brief The Dual and Penalty terms for each shared variable's biased prior
  std::map<RobotId, std::map<gtsam::Key, std::shared_ptr<biased_priors::BiasedPriorInfo>>> biased_prior_values_;
  /// @brief Mapping from external (other/global) shared vars to set of local variables connected to that shared vars
  std::map<gtsam::Key, std::set<gtsam::Key>> external_shared_variable_connections_;
  /// @brief Variables marked to be initialized from their owner robot
  std::map<gtsam::Key, RobotId> shared_initialization_variables_;
  /// @brief The local observability of variables marked to be initialized by their owner robot
  std::map<gtsam::Key, init::Observability> shared_init_var_observability_;
  /// @brief The observability for different types of factors
  init::TypeMap<init::Observability> factor_observability_;
  /// @brief Shared variables updated during the last communication that need to be reeliminated to account for their
  /// new shared estimates and dual variables
  gtsam::FastList<gtsam::Key> reelim_keys_;
  /// @brief Variables involved with any inter-robot measurements initialized during the last communication that need to
  /// be considered involved in the next update
  std::set<gtsam::Key> force_involved_keys_;
  /// @brief Aggregated biased priors declared during communication with other robots
  gtsam::NonlinearFactorGraph new_other_robot_declared_biased_priors_;
  /// @brief The internal clock of the algorithm used to ensure communication linearity
  size_t stamp_;
  /// @brief Mapping from the other agent to the stamp marking the completion of their last communication
  std::map<RobotId, size_t> robot_last_comm_stamps_;
  /// @brief Mapping from internal names to external (first) and vice-versa (second) for each robot
  std::map<RobotId, std::pair<std::map<gtsam::Key, gtsam::Key>, std::map<gtsam::Key, gtsam::Key>>>
      robot_variable_name_maps_;

  /** INTERFACE **/
 public:
  /** @brief Constructor
   * @param rid: The robot id for this agent
   * @param params: The rimesa specific parameters
   * @param isam2_params: The parameters for the underlying iSAM2 solver
   * @param sigkernel_param: The parameters for graduated factors
   * WARN: This class will override some isam2_params (relinearizeSkip, cacheLinearizedFactors) to ensure proper
   * functionality of the rimesa algorithm
   */
  RIMESA(const RobotId& rid, RIMESAParams params = RIMESAParams(),
         risam::RISAM::Parameters risam_params = risam::RISAM::Parameters())
      : robot_id_(rid), params_(params), smoother_(risam_params), stamp_(0) {
    factor_observability_ = init::defaultFactorObservability();
  }

  /** @brief Update interface for new measurements.
   * @param new_factors: The new measurements. These measurements may be intra or inter robot measurements.
   * @param new_theta: Any new values affected by the new_factors that are not already part of the system.
   * @returns The ISAM2Result containing information on the update.
   */
  risam::RISAM::UpdateResult update(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                                    const gtsam::Values& new_theta = gtsam::Values());

  /// @brief Returns the current solution of the distributed incremental smoother
  gtsam::Values getEstimate() { return current_estimate_; }

  /** @brief Returns the set of measurements identified as outliers
   * riMESA defines a measurement as an outlier if its current residual is greater than the chi2_threshold provided
   * @param chi2_outlier_thresh - The chi2 threshold used to define outliers
   */
  std::set<size_t> getOutliers(double chi2_outlier_threshold);

  /** @brief Returns identifiers for all biased priors that are currently classified as outliers
   * We uniquely identify a biased prior by the variable that is affected with the other robot with which it is shared
   * We represent this as a pair of [rid, key]
   * @param outliers - The current set of outlier factors from getOutliers
   */
  std::set<std::pair<RobotId, gtsam::Key>> getOutlierBiasedPriors(const std::set<size_t>& outlier_factors);

  /** @brief Constructs a CommunicationStateCache from the current state for communication with other_robot
   * @param other_robot: The id of the other robot with which we are initializing a communication
   * @returns A cache built from the current state data required to for a CommunicationHandler to perform the
   * communication
   */
  CommunicationStateCache getCommunicationStateCache(const RobotId other_robot);

  /** @brief Incorporates data received during a communication from another robot.
   * Updates relevant shared estimates and dual variables, and adds any sets up any new biased priors
   * @param CommunicationResult from a successful communication with another robot
   * @note Should only be called if it is guaranteed that the other robot has also successfully completed the
   * communication
   */
  void incorporateCommunicationResult(const CommunicationResult communication_result);

  /// @brief Returns the underlying factors of the system
  gtsam::NonlinearFactorGraph getFactorsUnsafe() { return smoother_.getFactorsUnsafe(); }

  /// @brief Returns the Robot id for this solver
  RobotId getRobotId() { return robot_id_; }

  /** @brief Adds a variable name mapping for a specific other robot
   * This is to be used if "other_robot" refers to a specific variable by "other_key" but we want to internally refer
   * to this variable as "internal_key". This is typically not needed by a majority of users.
   */
  void addVariableNameMapping(const gtsam::Key& internal_key, const RobotId& other_robot, const gtsam::Key& other_key) {
    if (robot_variable_name_maps_.count(other_robot) == 0) {
      robot_variable_name_maps_[other_robot] =
          std::make_pair(std::map<gtsam::Key, gtsam::Key>(), std::map<gtsam::Key, gtsam::Key>());
    }
    robot_variable_name_maps_[other_robot].first[internal_key] = other_key;
    robot_variable_name_maps_[other_robot].second[other_key] = internal_key;
  }

  /** HELPERS **/
 protected:
  /** @brief Finds any new shared variables observed in an update
   * @param new_factors - The new factors involved in the update
   * @returns A list of the new shared variables and the robots with whom they are shared
   */
  std::vector<std::tuple<RobotId, gtsam::Key, init::Observability>> findNewSharedVariables(
      const gtsam::NonlinearFactorGraph& new_factors);

  /** @brief Adds new shared variables updating the internal book keeping and constructing biased priors.
   * This routine is called both during communication procedures, and during update procedures
   * @param new_shared_vars: New shared variables and the robot they are shared with induced by a inter-robot update
   * @param new_shared_var_connections: New local variables connected to external shared variables
   * @param new_theta: Values containing initial estimates for all new shared variables
   * @param undeclared: Flag indicating that these shared variables are "undeclared" and not yet know to the other
   * robot
   * @returns Biased priors for each new shared variable
   */
  gtsam::NonlinearFactorGraph addNewSharedVariables(
      const std::vector<std::tuple<RobotId, gtsam::Key, init::Observability>>& new_shared_vars,
      const gtsam::Values& new_theta, bool undeclared);

  /** @brief Records any new connections from observed external shared variables in the update
   * @param new_factors - The new factors involved in the update
   * @mutates external_shared_variable_connections_ to record the external connections
   */
  void addNewExternalSharedVariableConnections(const gtsam::NonlinearFactorGraph& new_factors);

  /** COMMUNICATION INTERFACE **/
 public:
  /** @brief Class that manages the riMESA communication handshake with another robot
   * Implemented as a separate class to permit communication to occur in a non-blocking fashion
   * This handler is intended to run in a separate thread from the riMESA instance
   */
  class CommunicationHandler {
    /** TYPES **/
   public:
    /// @brief Shortcut to shared pointer
    typedef std::shared_ptr<CommunicationHandler> shared_ptr;
    /// @brief The state of communication, progresses in order
    enum CommState { INIT, DEC_VARS, REC_VARS, SENT_DATA, REC_DATA };

    /** FIELDS **/
   protected:
    /// @brief Cache of state data copied from riMESA instance when communication is initiated
    const CommunicationStateCache state_cache_;

    /// @brief The current state of the communication handler
    CommState comm_state_{CommState::INIT};

    /// @brief Data required to update the internal state if this communication is successful
    CommunicationResult result_;

    /** INTERFACE **/
   public:
    /// @brief  Constructor requires a state cache of riMESA from when the communication is initialized
    CommunicationHandler(CommunicationStateCache state_cache) : state_cache_(state_cache) {}

    /** @brief Interface for sending new shared variables
     * @returns Any new variables shared with the other robot since last communication, and all observed global
     * variables that may be shared, according to the state_cache_
     */
    DeclaredVariables declareNewSharedVariables();

    /** @brief Interface for receiving new shared variables from another robot.
     * @param declared_variables: The declared variables of the robot with whom we are communicating
     */
    void receiveNewSharedVariables(const DeclaredVariables& declared_variables);

    /** @brief Interface to construct data to send to another robot during inter-robot communication,
     * @returns The data to send to the other_robot, , according to the state_cache_
     */
    CommunicationData sendCommunication();

    /** @brief Interface for receiving shared estimates from inter-robot communication.
     * @param comm_data: The data received from the robot with whom we are communicating
     */
    void receiveCommunication(const CommunicationData& comm_data);

    /** @brief Gets the result from a successful communication\
     * @returns the Communication Result if the communication is complete otherwise std::nullopt
     */
    std::optional<CommunicationResult> getResult();

    /** @brief Gets the size of the data communicated during the communication
     * @returns The amount of data sent by this agent in KiloBytes
     */
    std::optional<double> commSizeKB();

   private:
    /// @brief Returns the input set of keys updated with translation for any key in name_mapping
    gtsam::KeySet translateKeySet(const gtsam::KeySet& keys,
                                  const std::optional<std::map<gtsam::Key, gtsam::Key>>& name_mapping) const;
    /// @brief Returns the input values updated with translation for any key in name_mapping
    gtsam::Values translateValues(const gtsam::Values& values,
                                  const std::optional<std::map<gtsam::Key, gtsam::Key>>& name_mapping) const;
    /// @brief Returns the input observability map updated with translation for any key in name_mapping
    std::map<gtsam::Key, init::Observability> translateInitialization(
        const std::map<gtsam::Key, init::Observability>& init,
        const std::optional<std::map<gtsam::Key, gtsam::Key>>& name_mapping) const;
  };
};

}  // namespace rimesa