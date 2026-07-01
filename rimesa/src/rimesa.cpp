/** @brief Implementation of the rimesa algorithm interface.
 *
 * @author Dan McGann
 */

#include "rimesa/rimesa.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <risam/GraduatedFactor.h>
#include <risam/GraduatedKernel.h>

namespace rimesa {
/**
 * #### ##    ## ######## ######## ########  ########    ###     ######  ########
 *  ##  ###   ##    ##    ##       ##     ## ##         ## ##   ##    ## ##
 *  ##  ####  ##    ##    ##       ##     ## ##        ##   ##  ##       ##
 *  ##  ## ## ##    ##    ######   ########  ######   ##     ## ##       ######
 *  ##  ##  ####    ##    ##       ##   ##   ##       ######### ##       ##
 *  ##  ##   ###    ##    ##       ##    ##  ##       ##     ## ##    ## ##
 * #### ##    ##    ##    ######## ##     ## ##       ##     ##  ######  ########
 */

/*********************************************************************************************************************/
risam::RISAM::UpdateResult RIMESA::update(const gtsam::NonlinearFactorGraph& new_factors,
                                          const gtsam::Values& new_theta) {
  // Determine if we observed new variables of other robots
  std::vector<std::tuple<RobotId, gtsam::Key, init::Observability>> new_shared_variables =
      findNewSharedVariables(new_factors);

  // Create new biased priors for all newly observed other robot variables
  gtsam::NonlinearFactorGraph new_biased_priors(addNewSharedVariables(new_shared_variables, new_theta, true));

  // Record any new connections to observed external shared variables
  addNewExternalSharedVariableConnections(new_factors);

  // Augment the new factors with new biased priors and any biased priors for variables declared by other robots
  gtsam::NonlinearFactorGraph augmented_new_factors(new_factors);
  augmented_new_factors.push_back(new_biased_priors);
  augmented_new_factors.push_back(new_other_robot_declared_biased_priors_);
  new_other_robot_declared_biased_priors_ = gtsam::NonlinearFactorGraph();

  // Do the update
  gtsam::ISAM2UpdateParams update_params;
  update_params.force_relinearize = true;
  update_params.extraReelimKeys = reelim_keys_;
  boost::optional<std::set<gtsam::Key>> extra_involved_keys = boost::none;

  if (force_involved_keys_.size() > 0) extra_involved_keys = force_involved_keys_;
  risam::RISAM::UpdateResult result =
      smoother_.update(augmented_new_factors, new_theta, extra_involved_keys, update_params);
  current_estimate_ = smoother_.calculateEstimate();
  reelim_keys_.clear();
  force_involved_keys_.clear();

  // Increment the Internal Clock
  stamp_++;
  return result;
}

/*********************************************************************************************************************/
CommunicationStateCache RIMESA::getCommunicationStateCache(const RobotId other_robot) {
  std::optional<std::pair<std::map<gtsam::Key, gtsam::Key>, std::map<gtsam::Key, gtsam::Key>>> name_mappings =
      std::nullopt;
  if (robot_variable_name_maps_.count(other_robot) != 0) {
    name_mappings = robot_variable_name_maps_[other_robot];
  }
  return CommunicationStateCache(stamp_, other_robot, robot_shared_vars_[other_robot], observed_global_variables_,
                                 current_estimate_, shared_initialization_variables_, shared_init_var_observability_,
                                 name_mappings);
}

/*********************************************************************************************************************/
void RIMESA::incorporateCommunicationResult(CommunicationResult comm_result) {
  // 1. Ensure pairwise linearity of communication
  if (robot_last_comm_stamps_.count(comm_result.other_robot) == 0 ||
      robot_last_comm_stamps_[comm_result.other_robot] < comm_result.start_stamp) {
    // Communication is linear so we can just update the last completed comm stamp
    robot_last_comm_stamps_[comm_result.other_robot] = stamp_;
  } else {
    std::stringstream ss;
    ss << "Robot: " << robot_id_ << " attempting nonlinear communication with Robot: " << comm_result.other_robot;
    ss << std::endl;
    ss << "Communication started at " << comm_result.start_stamp << std::endl;
    ss << "Previous communication completed at " << robot_last_comm_stamps_[comm_result.other_robot];
    throw std::runtime_error(ss.str());
  }

  // 2. Update book keeping and create biased priors, aggregate biased priors to be added on next update
  std::vector<std::tuple<RobotId, gtsam::Key, init::Observability>> new_shared_vars_list;
  for (const gtsam::Key& key : comm_result.new_shared_vars) {
    new_shared_vars_list.push_back(std::tuple(comm_result.other_robot, key, init::Observability::ALL));
  }
  new_other_robot_declared_biased_priors_.push_back(
      addNewSharedVariables(new_shared_vars_list, current_estimate_, false));

  // 3: For all vars for which we received an initialization estimate for update current state + mark initialized
  for (const auto& [key, obs] : comm_result.vars_received_for_initialization) {
    // Compute the value to use for initialization
    const gtsam::Value& local_val = comm_result.sent_solution.at(key);
    const gtsam::Value& other_val = comm_result.received_solution.at(key);
    boost::shared_ptr<gtsam::Value> init_val =
        init::computeInitialization(local_val, other_val, obs, params_.external_shared_var_init_strat);
    // Update the internal value
    current_estimate_.update(key, *init_val);
    comm_result.sent_solution.update(key, *init_val);
    smoother_.forceSetLinearization(key, *init_val);
    // Update house keeping
    shared_initialization_variables_.erase(key);
    shared_init_var_observability_.erase(key);
  }

  // 4: For all shared variables known after this communication update dual and edge-variables
  bool initialized_new_shared_variables = false;
  for (const gtsam::Key& key : comm_result.known_shared_vars) {
    biased_priors::BiasedPriorInfo::shared_ptr bpi = biased_prior_values_[comm_result.other_robot][key];

    // Grab the value that we sent to the other robot
    const gtsam::Value& this_val = comm_result.sent_solution.at(key);
    // Grab the value we received from the other robot
    boost::shared_ptr<gtsam::Value> other_val = comm_result.received_solution.at(key).clone();
    // If the variable was sent for initialization we need to compute the initialization the other robot will compute
    if (comm_result.vars_sent_for_initialization.count(key)) {
      init::Observability obs = comm_result.vars_sent_for_initialization.at(key);
      other_val = init::computeInitialization(*other_val, this_val, obs, params_.external_shared_var_init_strat);
    }

    // Compute the new shared estimate and dual variable
    boost::shared_ptr<gtsam::Value> new_shared_estimate = bpi->interpolate(this_val, *other_val);
    gtsam::Vector new_dual_variable = (params_.dual_decay_rate * bpi->dual) +
                                      bpi->penalty * bpi->constraint_func(this_val, *new_shared_estimate, boost::none);

    // Compute the change in the shared estimate and dual variable relative to the current linearization point
    double estimate_delta = new_shared_estimate->localCoordinates_(*bpi->shared_estimate_lin_point).norm();
    double dual_delta = (new_dual_variable - bpi->dual_lin_point).norm();

    // If new, or there is sufficient change, mark for reelim and update linearization points
    if ((estimate_delta + dual_delta) >= params_.shared_var_wildfire_threshold ||
        bpi->penalty == params_.initial_penalty_param) {
      reelim_keys_.push_back(key);
      bpi->shared_estimate_lin_point = new_shared_estimate->clone();
      bpi->dual_lin_point = gtsam::Vector(new_dual_variable);
    }

    // Update shared and dual estimates
    bpi->shared_estimate = new_shared_estimate;
    bpi->dual = new_dual_variable;
    if (bpi->penalty < params_.penalty_param) {
      bpi->penalty = params_.penalty_param;
      initialized_new_shared_variables = true;
    }
  }

  // 5. If we initialized any new shared variables with this other robot, force involvement for all connected variables
  if (initialized_new_shared_variables) {
    for (auto& sk : comm_result.known_shared_vars) {
      force_involved_keys_.insert(sk);
      if (external_shared_variable_connections_.count(sk)) {
        std::set<gtsam::Key> connections = external_shared_variable_connections_[sk];
        force_involved_keys_.insert(connections.begin(), connections.end());
      }
    }
  }
}

/*********************************************************************************************************************/
std::set<size_t> RIMESA::getOutliers(double chi2_outlier_threshold) {
  // Get the outliers from the internal riSAM solver
  return smoother_.getOutliers(chi2_outlier_threshold);
}

/*********************************************************************************************************************/
std::set<std::pair<RobotId, gtsam::Key>> RIMESA::getOutlierBiasedPriors(const std::set<size_t>& outlier_factors) {
  gtsam::NonlinearFactorGraph factors = smoother_.getFactorsUnsafe();

  std::set<std::pair<RobotId, gtsam::Key>> outlier_biased_priors;
  for (const size_t& fidx : outlier_factors) {
    auto factor_ptr = factors.at(fidx);
    auto bp_ptr = boost::dynamic_pointer_cast<biased_priors::BiasedPrior>(factor_ptr);
    if (bp_ptr) outlier_biased_priors.insert(std::make_pair(bp_ptr->info()->other_robot, factor_ptr->front()));
  }
  return outlier_biased_priors;
}

/**
 * ##     ## ######## ##       ########  ######## ########   ######
 * ##     ## ##       ##       ##     ## ##       ##     ## ##    ##
 * ##     ## ##       ##       ##     ## ##       ##     ## ##
 * ######### ######   ##       ########  ######   ########   ######
 * ##     ## ##       ##       ##        ##       ##   ##         ##
 * ##     ## ##       ##       ##        ##       ##    ##  ##    ##
 * ##     ## ######## ######## ##        ######## ##     ##  ######
 */

/*********************************************************************************************************************/
std::vector<std::tuple<RobotId, gtsam::Key, init::Observability>> RIMESA::findNewSharedVariables(
    const gtsam::NonlinearFactorGraph& new_factors) {
  size_t n_factors = new_factors.size();
  // We identify shared variables by the character of the variable key as it is used to signify the owner robot.
  std::vector<std::tuple<RobotId, gtsam::Key, init::Observability>> new_shared_variables;

  for (const auto& factor : new_factors) {
    for (const auto& key : factor->keys()) {
      gtsam::Symbol sym(key);
      if (sym.chr() != robot_id_ && !current_estimate_.exists(key)) {
        if (sym.chr() == GLOBAL_ID) {
          observed_global_variables_.insert(key);
        } else {
          new_shared_variables.push_back(std::make_tuple(sym.chr(), key, factor_observability_.at(typeid(*factor))));
        }
      }
    }
  }
  return new_shared_variables;
}

/*********************************************************************************************************************/
gtsam::NonlinearFactorGraph RIMESA::addNewSharedVariables(
    const std::vector<std::tuple<RobotId, gtsam::Key, init::Observability>>& new_shared_vars,
    const gtsam::Values& new_theta, bool undeclared) {
  gtsam::NonlinearFactorGraph new_biased_priors;

  // Iterate over all the new shared variables for each update bookkeeping
  for (const auto& [rid, key, obs] : new_shared_vars) {
    // Construct containers if first instance of a variable or a robot
    if (robot_shared_vars_.count(rid) == 0) robot_shared_vars_[rid] = gtsam::KeySet();

    // Record this shared variable
    robot_shared_vars_[rid].insert(key);

    // Mark for initialization if we dont already have the variable initialized
    if (!current_estimate_.exists(key) && gtsam::Symbol(key).chr() != robot_id_) {
      shared_initialization_variables_[key] = rid;
      shared_init_var_observability_[key] = obs;
    }

    // Construct the biased prior
    biased_priors::BiasedPriorInfo::shared_ptr bpi = std::make_shared<biased_priors::BiasedPriorInfo>(
        rid, params_.pose_biased_prior_type, new_theta.at(key), params_.initial_penalty_param);

    // Book keep the new biased prior and its corresponding values wrapping with kernel if configured
    new_biased_priors.push_back(biased_priors::factory(
        params_.pose_biased_prior_type, key, bpi, params_.biased_prior_noise_model_sigmas, params_.bp_kernel_params));
    biased_prior_values_[rid][key] = bpi;
  }
  return new_biased_priors;
}

/*********************************************************************************************************************/
void RIMESA::addNewExternalSharedVariableConnections(const gtsam::NonlinearFactorGraph& new_factors) {
  size_t nr_new_factors = new_factors.nrFactors();

  // The shared variables observed in this update and the factor(s) that observed them
  std::map<gtsam::Key, std::set<size_t>> observed_shared_variables;
  for (size_t fidx = 0; fidx < nr_new_factors; fidx++) {
    for (auto key : new_factors.at(fidx)->keys()) {
      gtsam::Symbol sym(key);
      if (sym.chr() != robot_id_) {
        if (observed_shared_variables.count(key) == 0) observed_shared_variables[key] = std::set<size_t>();
        observed_shared_variables[key].insert(fidx);
      }
    }
  }

  // For any shared variable observed mark the local variables that are connected to it
  for (auto& [sv_key, sv_factor_idxes] : observed_shared_variables) {
    // Extend the container if the external shared variable is new
    if (external_shared_variable_connections_.count(sv_key) == 0) {
      external_shared_variable_connections_[sv_key] = std::set<gtsam::Key>();
    }
    // Add all connections from this update
    for (auto& fidx : sv_factor_idxes) {
      for (auto& key : new_factors.at(fidx)->keys()) {
        // Add any local keys connected to this shared variable
        if (gtsam::Symbol(key).chr() == robot_id_) external_shared_variable_connections_[sv_key].insert(key);
      }
    }
  }
}

/**
 *  ######   #######  ##     ## ##     ## ##     ## ##    ## ####  ######     ###    ######## ####  #######  ##    ##
 * ##    ## ##     ## ###   ### ###   ### ##     ## ###   ##  ##  ##    ##   ## ##      ##     ##  ##     ## ###   ##
 * ##       ##     ## #### #### #### #### ##     ## ####  ##  ##  ##        ##   ##     ##     ##  ##     ## ####  ##
 * ##       ##     ## ## ### ## ## ### ## ##     ## ## ## ##  ##  ##       ##     ##    ##     ##  ##     ## ## ## ##
 * ##       ##     ## ##     ## ##     ## ##     ## ##  ####  ##  ##       #########    ##     ##  ##     ## ##  ####
 * ##    ## ##     ## ##     ## ##     ## ##     ## ##   ###  ##  ##    ## ##     ##    ##     ##  ##     ## ##   ###
 *  ######   #######  ##     ## ##     ##  #######  ##    ## ####  ######  ##     ##    ##    ####  #######  ##    ##
 */
/*********************************************************************************************************************/
DeclaredVariables RIMESA::CommunicationHandler::declareNewSharedVariables() {
  if (comm_state_ != CommState::INIT) {
    throw std::runtime_error(
        "RIMESA::CommunicationHandler::declareNewSharedVariables called out of order. Handler not in INIT State.");
  }

  DeclaredVariables dec_vars;
  dec_vars.shared_variables = translateKeySet(state_cache_.shared_variables, state_cache_.outgoing_variable_map);
  dec_vars.observed_global_variables =
      translateKeySet(state_cache_.observed_global_variables, state_cache_.outgoing_variable_map);

  comm_state_ = CommState::DEC_VARS;
  return dec_vars;
}

/*********************************************************************************************************************/
void RIMESA::CommunicationHandler::receiveNewSharedVariables(const DeclaredVariables& declared_variables) {
  if (comm_state_ != CommState::DEC_VARS) {
    throw std::runtime_error(
        "RIMESA::CommunicationHandler::receiveNewSharedVariables called out of order. Handler not in DEC_VARS State.");
  }

  for (const auto& key : translateKeySet(declared_variables.shared_variables, state_cache_.incoming_variable_map)) {
    // If the other knows of shared variables not known by this robot mark them as new shared variables
    if (state_cache_.shared_variables.count(key) == 0) {
      result_.new_shared_vars.insert(key);
    }
  }

  for (const auto& key :
       translateKeySet(declared_variables.observed_global_variables, state_cache_.incoming_variable_map)) {
    // If we have observed the key, but it is not marked as shared with the other_robot add it
    if (state_cache_.observed_global_variables.count(key) != 0 && state_cache_.shared_variables.count(key) == 0) {
      result_.new_shared_vars.insert(key);
    }
  }

  comm_state_ = CommState::REC_VARS;
}

/*********************************************************************************************************************/
CommunicationData RIMESA::CommunicationHandler::sendCommunication() {
  if (comm_state_ != CommState::REC_VARS) {
    throw std::runtime_error(
        "RIMESA::CommunicationHandler::sendCommunication called out of order. Handler not in REC_VARS State.");
  }

  gtsam::Values shared_var_solution;
  std::map<gtsam::Key, init::Observability> vars_requested_for_init;
  // Add all estimates to previously known shared variables
  for (const gtsam::Key& key : state_cache_.shared_variables) {
    // Add the local solution to the shared variable
    shared_var_solution.insert(key, state_cache_.current_estimate.at(key));

    // Mark the variable if we need to initialize it from the other robot
    if (state_cache_.shared_initialization_variables.count(key) &&
        state_cache_.shared_initialization_variables.at(key) == state_cache_.other_robot) {
      init::Observability obs = state_cache_.shared_init_var_observability.at(key);
      vars_requested_for_init.insert({key, obs});
      result_.vars_received_for_initialization.insert({key, obs});
    }
  }

  // Add all estimates for newly identified shared variables
  // These are this robot's poses + landmarks observed by the other_robot so none will need to be initialized
  for (const gtsam::Key& key : result_.new_shared_vars) {
    shared_var_solution.insert(key, state_cache_.current_estimate.at(key));
  }
  
  // Record the solution that we are sending to the other robot
  result_.sent_solution = shared_var_solution;

  // Construct Result
  CommunicationData comm_data;
  comm_data.shared_var_solution = translateValues(shared_var_solution, state_cache_.outgoing_variable_map);
  comm_data.vars_requested_for_init =
      translateInitialization(vars_requested_for_init, state_cache_.outgoing_variable_map);

  comm_state_ = CommState::SENT_DATA;
  return comm_data;
}

/*********************************************************************************************************************/
void RIMESA::CommunicationHandler::receiveCommunication(const CommunicationData& comm_data) {
  if (comm_state_ != CommState::SENT_DATA) {
    throw std::runtime_error(
        "RIMESA::CommunicationHandler::receiveCommunication called out of order. Handler not in SENT_DATA State.");
  }

  result_.vars_sent_for_initialization =
      translateInitialization(comm_data.vars_requested_for_init, state_cache_.incoming_variable_map);
  result_.received_solution = translateValues(comm_data.shared_var_solution, state_cache_.incoming_variable_map);

  comm_state_ = CommState::REC_DATA;
}

/*********************************************************************************************************************/
std::optional<CommunicationResult> RIMESA::CommunicationHandler::getResult() {
  if (comm_state_ == REC_DATA) {
    // Fill out remaining items in result
    result_.other_robot = state_cache_.other_robot;
    result_.start_stamp = state_cache_.start_stamp;
    result_.known_shared_vars = state_cache_.shared_variables;
    result_.known_shared_vars.insert(result_.new_shared_vars.begin(), result_.new_shared_vars.end());
    return result_;
  } else {
    return std::nullopt;
  }
}

/*********************************************************************************************************************/
std::optional<double> RIMESA::CommunicationHandler::commSizeKB() {
  if (comm_state_ == REC_DATA) {
    // All data sent during variable declarations
    double num_vars_declared = state_cache_.shared_variables.size() + state_cache_.observed_global_variables.size();
    double size_dec_vars_bytes = num_vars_declared * 8.0;  // 8 Bytes used for each variable key
    // Variables requested for Initialization
    double num_vars_req_for_init = result_.vars_received_for_initialization.size();
    double size_init_vars_bytes = num_vars_req_for_init * (8.0 + 1.0);  // 8 Bytes/key + 1 Byte for Observability Flag
    // Local Solution
    double size_sent_data_bytes = 0.0;
    for (const auto& kvp : result_.sent_solution) {
      // 8 Bytes for Key + 8 Bytes for each dim of the variable
      size_sent_data_bytes += 8.0 + 8.0 * kvp.value.dim();
    }
    // Convert from bytes to KB and return
    return (size_dec_vars_bytes + size_init_vars_bytes + size_sent_data_bytes) / 1000.0;
  } else {
    return std::nullopt;
  }
}

/*********************************************************************************************************************/
gtsam::KeySet RIMESA::CommunicationHandler::translateKeySet(
    const gtsam::KeySet& keys, const std::optional<std::map<gtsam::Key, gtsam::Key>>& name_mapping) const {
  if (!name_mapping) return keys;
  gtsam::KeySet result;
  for (const auto& key : keys) {
    const auto iter = name_mapping->find(key);
    if (iter != name_mapping->end()) {
      result.insert(iter->second);
    } else {
      result.insert(key);
    }
  }
  return result;
}

/*********************************************************************************************************************/
gtsam::Values RIMESA::CommunicationHandler::translateValues(
    const gtsam::Values& values, const std::optional<std::map<gtsam::Key, gtsam::Key>>& name_mapping) const {
  if (!name_mapping) return values;
  gtsam::Values result;
  for (const auto& kvp : values) {
    const auto iter = name_mapping->find(kvp.key);
    if (iter != name_mapping->end()) {
      result.insert(iter->second, kvp.value);
    } else {
      result.insert(kvp.key, kvp.value);
    }
  }
  return result;
}

/*********************************************************************************************************************/
std::map<gtsam::Key, init::Observability> RIMESA::CommunicationHandler::translateInitialization(
    const std::map<gtsam::Key, init::Observability>& init,
    const std::optional<std::map<gtsam::Key, gtsam::Key>>& name_mapping) const {
  if (!name_mapping) return init;
  std::map<gtsam::Key, init::Observability> result;
  for (const auto& kvp : init) {
    const auto iter = name_mapping->find(kvp.first);
    if (iter != name_mapping->end()) {
      result[iter->second] = kvp.second;
    } else {
      result[kvp.first] = kvp.second;
    }
  }
  return result;
}

}  // namespace rimesa