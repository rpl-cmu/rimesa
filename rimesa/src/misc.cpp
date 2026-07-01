/** @brief Implementation of Misc Utilities.
 *
 * @author Dan McGann
 */

#include "rimesa/misc.h"

namespace rimesa {
namespace misc {

/**
 * ##     ## ####  ######   ######
 * ###   ###  ##  ##    ## ##    ##
 * #### ####  ##  ##       ##
 * ## ### ##  ##   ######  ##
 * ##     ##  ##        ## ##
 * ##     ##  ##  ##    ## ##    ##
 * ##     ## ####  ######   ######
 */
/*********************************************************************************************************************/
void serializeGlobalGraph(std::map<char, std::shared_ptr<rimesa::RIMESA>>& robot_solvers,
                          const std::string factors_filename, const std::string values_filename) {
  gtsam::NonlinearFactorGraph global_graph;
  gtsam::Values global_values;
  for (auto& [rid, solver] : robot_solvers) {
    const gtsam::NonlinearFactorGraph factors = solver->getFactorsUnsafe();
    // Aggregate the global factor-graph
    for (const auto& factor_ptr : factors) {
      // Skip all biased priors
      biased_priors::BiasedPrior::shared_ptr biased_prior =
          boost::dynamic_pointer_cast<biased_priors::BiasedPrior>(factor_ptr);
      if (biased_prior) continue;

      // Remove kernel from all graduated factors
      auto grad_factor = boost::dynamic_pointer_cast<risam::GraduatedFactor>(factor_ptr);
      if (grad_factor) {
        global_graph.push_back(grad_factor->cloneUngraduated());
        continue;
      }

      // Add all remaining factors
      global_graph.push_back(factor_ptr);
    }

    // Aggregate the values
    global_values.insert(
        solver->getEstimate().filter([rid](gtsam::Key key) { return rid == gtsam::Symbol(key).chr(); }));
    global_values.insert_or_assign(
        solver->getEstimate().filter([](gtsam::Key key) { return gtsam::Symbol(key).chr() == GLOBAL_ID; }));
  }

  // Serialize the graph data to file
  // Use gtsam serialization helpers
  gtsam::serializeToFile<gtsam::NonlinearFactorGraph>(global_graph, factors_filename);
  gtsam::serializeToFile<gtsam::Values>(global_values, values_filename);
}

void serializeGlobalGraph(std::shared_ptr<rimesa::RIMESA>& solver, const std::string factors_filename,
                          const std::string values_filename) {
  std::map<char, std::shared_ptr<rimesa::RIMESA>> solver_map;
  solver_map[solver->getRobotId()] = solver;
  return serializeGlobalGraph(solver_map, factors_filename, values_filename);
}

/*********************************************************************************************************************/
void dot(std::ostream& os, const gtsam::NonlinearFactorGraph graph, const gtsam::Values& values,
         const std::set<size_t> outliers) {
  // Find bounds and set preamble
  auto [min, max] = vis_internal::findBounds(values);
  vis_internal::graphPreamble(&os);

  // All Keys
  gtsam::KeyVector keys = graph.keyVector();

  // Create nodes for each variable in the graph
  std::map<gtsam::Key, gtsam::Vector2> var_positions;
  for (gtsam::Key key : keys) {
    auto position = vis_internal::transformPositionToFig(vis_internal::extractPosition(values.at(key)), min);
    vis_internal::drawVariable(&os, key, position);
    var_positions[key] = position;
  }
  os << "\n";

  // Create factors, variable connections, and biased prior info
  for (size_t i = 0; i < graph.size(); i++) {
    const gtsam::NonlinearFactor::shared_ptr& factor = graph.at(i);
    biased_priors::BiasedPrior::shared_ptr bp_factor = boost::dynamic_pointer_cast<biased_priors::BiasedPrior>(factor);

    // Color the factor based on classification status
    std::optional<std::string> color = {};
    if (outliers.count(i)) color = "red";
    if (bp_factor) color = "black";
    if (bp_factor && outliers.count(i)) color = "purple";

    // Draw the factor
    if (factor) {
      const gtsam::KeyVector& factorKeys = factor->keys();
      // Determine the factors position
      gtsam::Vector2 position = gtsam::Vector2::Zero();
      for (auto& key : factorKeys) position += var_positions[key];
      position = position / double(factorKeys.size());
      // Overwrite if for biased_priors
      if (bp_factor) {
        position = vis_internal::transformPositionToFig(
            vis_internal::extractPosition(*(bp_factor->info()->shared_estimate)), min);
      }

      vis_internal::drawFactor(&os, i, factorKeys, position, color);
    }
  }

  os << "}\n";
  std::flush(os);
}

/*********************************************************************************************************************/
void saveGraph(const std::string& filename, rimesa::RIMESA& rimesa) {
  std::ofstream of(filename);
  dot(of, rimesa.getFactorsUnsafe(), rimesa.getEstimate(), rimesa.getOutliers(0.95));
  of.close();
}

/**
 * ##     ## ####  ######     #### ##    ## ######## ######## ########  ##    ##    ###    ##
 * ##     ##  ##  ##    ##     ##  ###   ##    ##    ##       ##     ## ###   ##   ## ##   ##
 * ##     ##  ##  ##           ##  ####  ##    ##    ##       ##     ## ####  ##  ##   ##  ##
 * ##     ##  ##   ######      ##  ## ## ##    ##    ######   ########  ## ## ## ##     ## ##
 *  ##   ##   ##        ##     ##  ##  ####    ##    ##       ##   ##   ##  #### ######### ##
 *   ## ##    ##  ##    ##     ##  ##   ###    ##    ##       ##    ##  ##   ### ##     ## ##
 *    ###    ####  ######     #### ##    ##    ##    ######## ##     ## ##    ## ##     ## ########
 */

namespace vis_internal {
/*********************************************************************************************************************/
void graphPreamble(std::ostream* os) { *os << "graph {\n"; }

/*********************************************************************************************************************/
void drawVariable(std::ostream* os, gtsam::Key key, const gtsam::Vector2& fig_position) {
  *os << "  var" << key << "[label=\"" << gtsam::DefaultKeyFormatter(key) << "\"";
  *os << ", pos=\"" << fig_position.x() << "," << fig_position.y() << "!\"";
  *os << "];\n";
}

/*********************************************************************************************************************/
void drawFactor(std::ostream* os, size_t i, const gtsam::KeyVector& keys,
                const std::optional<gtsam::Vector2>& fig_position, const std::optional<std::string>& color) {
  // Create dot for the factor.
  *os << "  factor" << i << "[label=\"\", shape=square, style=filled, fixedsize=true, width=0.1";
  if (fig_position) *os << ", pos=\"" << fig_position->x() << "," << fig_position->y() << "!\"";
  if (color) *os << ", color=\"" << *color << "\"";
  *os << "];\n";
  // Make factor-variable connections
  for (gtsam::Key key : keys) {
    *os << "  var" << key << "--" << "factor" << i << ";\n";
  }
}

/*********************************************************************************************************************/
gtsam::Vector2 extractPosition(const gtsam::Value& value) {
  gtsam::Vector3 t;
  if (const gtsam::GenericValue<gtsam::Pose2>* p = dynamic_cast<const gtsam::GenericValue<gtsam::Pose2>*>(&value)) {
    t << p->value().x(), p->value().y(), 0;
  } else if (const gtsam::GenericValue<gtsam::Vector2>* p =
                 dynamic_cast<const gtsam::GenericValue<gtsam::Vector2>*>(&value)) {
    t << p->value().x(), p->value().y(), 0;
  } else if (const gtsam::GenericValue<gtsam::Vector>* p =
                 dynamic_cast<const gtsam::GenericValue<gtsam::Vector>*>(&value)) {
    if (p->dim() == 2) {
      const Eigen::Ref<const gtsam::Vector2> p_2d(p->value());
      t << p_2d.x(), p_2d.y(), 0;
    } else if (p->dim() == 3) {
      const Eigen::Ref<const gtsam::Vector3> p_3d(p->value());
      t = p_3d;
    } else {
      return {};
    }
  } else if (const gtsam::GenericValue<gtsam::Pose3>* p =
                 dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&value)) {
    t = p->value().translation();
  } else if (const gtsam::GenericValue<gtsam::Point3>* p =
                 dynamic_cast<const gtsam::GenericValue<gtsam::Point3>*>(&value)) {
    t = p->value();
  } else {
    throw std::runtime_error("rimesa::misc::vis_internal::extractPosition provided invalid value");
  }
  return gtsam::Vector2(t.x(), t.y());
}

/*********************************************************************************************************************/
std::pair<gtsam::Vector2, gtsam::Vector2> findBounds(const gtsam::Values& values) {
  gtsam::Vector2 min;
  min.x() = std::numeric_limits<double>::infinity();
  min.y() = std::numeric_limits<double>::infinity();

  gtsam::Vector2 max;
  max.x() = std::numeric_limits<double>::lowest();
  max.y() = std::numeric_limits<double>::lowest();

  for (const auto& [key, value] : values) {
    gtsam::Vector2 xy = extractPosition(values.at(key));
    if (xy.x() < min.x()) min.x() = xy.x();
    if (xy.y() < min.y()) min.y() = xy.y();

    if (xy.x() > max.x()) max.x() = xy.x();
    if (xy.y() > max.y()) max.y() = xy.y();
  }
  return std::make_pair(min, max);
}

/*********************************************************************************************************************/
gtsam::Vector2 transformPositionToFig(const gtsam::Vector2 pos, const gtsam::Vector2& min) {
  return gtsam::Vector2((pos.x() - min.x()), (pos.y() - min.y()));
}

}  // namespace vis_internal
}  // namespace misc
}  // namespace rimesa
