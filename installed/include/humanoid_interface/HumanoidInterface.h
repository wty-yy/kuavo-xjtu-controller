/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include "humanoid_interface/common/ModelSettings.h"
#include "humanoid_interface/initialization/HumanoidInitializer.h"
#include "humanoid_interface/reference_manager/SwitchedModelReferenceManager.h"
// #include "humanoid_interface_drake/humanoid_interface_drake.h"

/**
 * HumanoidInterface class
 * General interface for mpc implementation on the legged robot model
 */
namespace ocs2
{
  namespace humanoid
  {

    class HumanoidInterface final : public RobotInterface
    {
    public:
      /**
       * Constructor
       *
       * @throw Invalid argument error if input task file or urdf file does not exist.
       *
       * @param [in] taskFile: The absolute path to the configuration file for the MPC.
       * @param [in] urdfFile: The absolute path to the URDF file for the robot.
       * @param [in] referenceFile: The absolute path to the reference configuration file.
       * @param [in] useHardFrictionConeConstraint: Which to use hard or soft friction cone constraints.
       */
      HumanoidInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitFile,
                        int robot_version_int, bool useHardFrictionConeConstraint = false);

      ~HumanoidInterface() override = default;

      const OptimalControlProblem &getOptimalControlProblem() const override { return *problemPtr_; }

      const ModelSettings &modelSettings() const { return modelSettings_; }
      const ddp::Settings &ddpSettings() const { return ddpSettings_; }
      const mpc::Settings &mpcSettings() const { return mpcSettings_; }
      const rollout::Settings &rolloutSettings() const { return rolloutSettings_; }
      const sqp::Settings &sqpSettings() { return sqpSettings_; }
      const ipm::Settings &ipmSettings() { return ipmSettings_; }

      const vector_t &getInitialState() const { return initialState_; }
      const RolloutBase &getRollout() const { return *rolloutPtr_; }
      PinocchioInterface &getPinocchioInterface() { return *pinocchioInterfacePtr_; }
      const CentroidalModelInfo &getCentroidalModelInfo() const { return centroidalModelInfo_; }
      PinocchioGeometryInterface &getGeometryInterface() { return *geometryInterfacePtr_; }
      std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

      const HumanoidInitializer &getInitializer() const override { return *initializerPtr_; }
      std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

      void setupOptimalControlProblem(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitFile, bool verbose);
      void setupCPUconfig();
    private:
      std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string &referenceFile, const std::string &gaitFile, bool verbose) const;

      std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string &taskFile, const CentroidalModelInfo &info, bool verbose);
      std::unique_ptr<StateCost> getBaseTrackingTerminalCost(const std::string &taskFile, const CentroidalModelInfo &info, bool verbose);
      
      matrix_t initializeInputCostWeight(const std::string &taskFile, const CentroidalModelInfo &info);
      matrix_t initializeStateCostWeight(const std::string &taskFile, const CentroidalModelInfo &info, std::string feildName = "Q");

      std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string &taskFile, bool verbose) const;
      std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);
      std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                    const RelaxedBarrierPenalty::Config &barrierPenaltyConfig);
      std::unique_ptr<StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);
      std::unique_ptr<StateInputConstraint> getZeroSixDofForceConstraint(size_t sixDofContactPointIndex);
      std::unique_ptr<StateInputConstraint> getZeroSixDofTorqueConstraint(size_t sixDofContactPointIndex);
      std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                      size_t contactPointIndex, bool useAnalyticalGradients);
      std::unique_ptr<StateInputCost> getZeroVelocitySoftConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                    size_t contactPointIndex, bool useAnalyticalGradients,
                                                                    const scalar_t &scale);
      std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                        size_t contactPointIndex, bool useAnalyticalGradients);
      std::unique_ptr<StateInputCost> getSoftSwingTrajConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                             size_t contactPointIndex, const std::string& taskFile,
                                                             bool verbose);
      std::unique_ptr<StateInputConstraint> getFootRollConstraint(size_t contactPointIndex);
      std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface &pinocchioInterface, const std::string &taskFile,
                                                            const std::string &prefix, bool verbose);

      std::pair<scalar_t, bool> loadZeroVelocityConstraintScale(const std::string &taskFile, bool verbose) const;

      std::unique_ptr<StateInputCost> getBasePitchSoftConstraint(const std::string& taskFile, bool verbose);
      
      std::unique_ptr<StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& prefix, const std::string& eeName, const int eeIndex, bool verbose);
      
      std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string> &endEffectorIds, const std::string &modelName);

      ModelSettings modelSettings_;
      ddp::Settings ddpSettings_;
      mpc::Settings mpcSettings_;
      sqp::Settings sqpSettings_;
      ipm::Settings ipmSettings_;
      const bool useHardFrictionConeConstraint_;

      std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
      CentroidalModelInfo centroidalModelInfo_;

      std::unique_ptr<OptimalControlProblem> problemPtr_;
      std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

      rollout::Settings rolloutSettings_;
      std::unique_ptr<RolloutBase> rolloutPtr_;
      std::unique_ptr<HumanoidInitializer> initializerPtr_;
      std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;

      vector_t initialState_;
      int build_cppad_status_ {0};

      size_t joint_Num_ = 12;
      int robot_version_int_=34;//default to 34
    };

  } // namespace humanoid
} // namespace ocs2
