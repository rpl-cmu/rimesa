#pragma once
/** @brief Miscellaneous utilities for use with riMESA
 *
 * @author Dan McGann
 */

#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "rimesa/biased_prior.h"
#include "rimesa/rimesa.h"
#include "risam/GraduatedFactor.h"
#include "risam/GraduatedKernel.h"

namespace rimesa {
namespace misc {

/** @brief Serializes the global joint factor-graph from a team of robots running riMESA
 * WARN: Removes all biased priors since they dont make sense to include in a joint-graph
 * WARN: Removes kernels from potential outlier factors to permit one to load with another robust solver
 * WARN: For any shared variable, saves the estimate from the owner robot
 * @param robot_solvers: The riMESA solver for each robot in the team
 * @param factors_filename: The file which to serialize the factor_graph to
 * @param values_filename: The file which to serialize the current estimate to
 */
void serializeGlobalGraph(std::map<char, std::shared_ptr<rimesa::RIMESA>>& robot_solvers,
                          const std::string factors_filename, const std::string values_filename);

/// @brief Overload of the above for the case that the process does not have access to all solvers and each must be
/// serialized individually and combined later.
void serializeGlobalGraph(std::shared_ptr<rimesa::RIMESA>& robot_solver, const std::string factors_filename,
                          const std::string values_filename);

/** @brief Writes a visualization of the riMESA graph to the given stream
 * Positions:
 *   - Variables are drawn at their estimated positions
 *   - Factors are drawn between the involved variables
 *   - Biased Priors are drawn at the current shared estimate location
 *
 * Colors:
 *   - Variables: white
 *   - Factors | Inlier: grey , Outlier: red
 *   - BiasedPriors | Inlier: black, Outlier: purple
 */
void dot(std::ostream& os, const gtsam::NonlinearFactorGraph graph, const gtsam::Values& values,
         const std::set<size_t> outliers);

/// @brief Saves a visualization of the riMESA graph to file
void saveGraph(const std::string& filename, rimesa::RIMESA& rimesa);

namespace vis_internal {
/** @brief Internal helper functions for rimesaAsDot
 * These provide modified functionality similar to gtsam::GraphvizFormatting and gtsam::DotWriter
 *
 * Original Copyright:
 * GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 */
/// @brief Write out preamble for graph, including size.
void graphPreamble(std::ostream* os);

/// @brief Create a variable dot fragment.
void drawVariable(std::ostream* os, gtsam::Key key, const gtsam::Vector2& fig_position);
/// @brief Draw a single factor, specified by its index i and its variable keys.
void drawFactor(std::ostream* os, size_t i, const gtsam::KeyVector& keys,
                const std::optional<gtsam::Vector2>& fig_position = {}, const std::optional<std::string>& color = {});

/// @brief retrieve an x-y position from a value [Vector, Point2, Point3, Pose2, Pose3]
gtsam::Vector2 extractPosition(const gtsam::Value& value);

/// @brief Retrieves the min bound of the a set of values
std::pair<gtsam::Vector2, gtsam::Vector2> findBounds(const gtsam::Values& values);

/// @brief Figure coordinates must be positive, so transform into the positive x,y space and optionally scale
gtsam::Vector2 transformPositionToFig(const gtsam::Vector2 pos, const gtsam::Vector2& min);
}  // namespace vis_internal
}  // namespace misc
}  // namespace rimesa
