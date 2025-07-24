/*
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Nature Robots GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  abstract_plan_refiner.h
 *
 *  author: Georg John <georg.john@naturerobots.com>
 *
 */

#ifndef MBF_ABSTRACT_CORE__ABSTRACT_PLAN_REFINER_H_
#define MBF_ABSTRACT_CORE__ABSTRACT_PLAN_REFINER_H_

#include "mbf_abstract_core/abstract_plan_refiner.h"
#include <rclcpp/rclcpp.hpp>

namespace mbf_simble_core
{
class SimplePlanRefiner : public mbc_abstract_core::AbstractPlanRefiner
{
public:
  typedef std::shared_ptr< ::mbf_simple_core::SimplePlanRefiner> Ptr;

  /**
   * @brief Destructor
   */
  virtual ~SimplePlanRefiner() = default;

  /**
   * @brief Given a plan refine it until convergence
   * @param plan The plan... filled by the planner
   * @param epsilon Convergence threshold, e.g. for smoothing
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result:
   *         SUCCESS          = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE          = 200  # Unspecified failure, only used for old, non-mfb_core based plugins
   *         CANCELED         = 201  # The action has been canceled by a action client
   *         DEPLETED         = 202  # The action has been run out of attempts
   *         MAP_ERROR        = 210  # The map is not available or not running properly
   *         INTERNAL_ERROR   = 211  # An internal error occurred in the plan refiner
   *         STOPPED          = 212  # The refiner execution has been stopped rigorously
   *         221..249 are reserved as plugin specific errors
   */
  virtual uint32_t applyRefinement(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & refined_plan,
    std::string & message) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

  /**
   * @brief Initialization function for the SimplePlanner
   * @param name The name of this planner
   * @param node_handle Node handle instance
   */
  virtual void initialize(const std::string name, const rclcpp::Node::SharedPtr & node_handle) = 0;

protected:
  /**
   * @brief Constructor
   */
  SimplePlanRefiner() = default;


};  /* SimplePlanRefner*/
}  /* namespace mbf_abstract_core */

#endif /* MBF_ABSTRACT_CORE__ABSTRACT_PLAN_REFINER_H_ */
