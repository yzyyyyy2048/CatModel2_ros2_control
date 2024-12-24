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

#include <iostream>
#include <memory>
#include <string>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <CatModel2_v2_interface/LeggedInterface.h>
#include <CatModel2_v2_interface/LeggedRobotPreComputation.h>
#include <CatModel2_v2_interface/constraint/FrictionConeConstraint.h>
#include <CatModel2_v2_interface/constraint/NormalVelocityConstraintCppAd.h>
#include <CatModel2_v2_interface/constraint/ZeroForceConstraint.h>
#include <CatModel2_v2_interface/constraint/ZeroVelocityConstraintCppAd.h>
#include <CatModel2_v2_interface/cost/LeggedRobotQuadraticTrackingCost.h>
#include <CatModel2_v2_interface/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ocs2_centroidal_model/FactoryFunctions.h>


namespace ocs2::legged_robot {
    LeggedInterface::LeggedInterface(const std::string &taskFile,
                                               const std::string &urdfFile,
                                               const std::string &referenceFile,
                                               bool useHardFrictionConeConstraint)
        : useHardFrictionConeConstraint_(useHardFrictionConeConstraint) {
        // check that task file exists
        const boost::filesystem::path taskFilePath(taskFile);
        if (exists(taskFilePath)) {
            std::cerr << "[LeggedInterface] Loading task file: " << taskFilePath
                    << std::endl;
        } else {
            throw std::invalid_argument("[LeggedInterface] Task file not found: " +
                                        taskFilePath.string());
        }
        // check that urdf file exists
        boost::filesystem::path urdfFilePath(urdfFile);
        if (exists(urdfFilePath)) {
            std::cerr << "[LeggedInterface] Loading Pinocchio model from: "
                    << urdfFilePath << std::endl;
        } else {
            throw std::invalid_argument("[LeggedInterface] URDF file not found: " +
                                        urdfFilePath.string());
        }
        // check that targetCommand file exists
        const boost::filesystem::path referenceFilePath(referenceFile);
        if (exists(referenceFilePath)) {
            std::cerr << "[LeggedInterface] Loading target command settings from: "
                    << referenceFilePath << std::endl;
        } else {
            throw std::invalid_argument(
                "[LeggedInterface] targetCommand file not found: " +
                referenceFilePath.string());
        }

        bool verbose;
        loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose",
                                  verbose);

        // load setting from loading file
        modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
        mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
        ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
        sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
        ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
        rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);

        // OptimalConrolProblem
        setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

        // initial state
        initialState_.setZero(centroidalModelInfo_.stateDim);
        loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
    }


    void LeggedInterface::setupOptimalControlProblem(
        const std::string &taskFile,
        const std::string &urdfFile,
        const std::string &referenceFile,
        bool verbose) {
        // PinocchioInterface
        pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
            centroidal_model::createPinocchioInterface(
                urdfFile, modelSettings_.jointNames));

        // CentroidalModelInfo
        centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
            *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
            centroidal_model::loadDefaultJointState(
                pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
            modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);

        // Swing trajectory planner
        auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(
            loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose),
            4);

        // Mode schedule manager
        referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(
            loadGaitSchedule(referenceFile, verbose),
            std::move(swingTrajectoryPlanner));

        // Optimal control problem
        problemPtr_ = std::make_unique<OptimalControlProblem>();

        // Dynamics
        bool useAnalyticalGradientsDynamics = false;
        loadData::loadCppDataType(
            taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics",
            useAnalyticalGradientsDynamics);
        std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
        if (useAnalyticalGradientsDynamics) {
            throw std::runtime_error(
                "[LeggedInterface::setupOptimalConrolProblem] The analytical "
                "dynamics class is not yet implemented!");
        }
        const std::string modelName = "dynamics";
        dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(*pinocchioInterfacePtr_,
                                                              centroidalModelInfo_, modelName,
                                                              modelSettings_);

        problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

        // Cost terms
        problemPtr_->costPtr->add(
            "baseTrackingCost",
            getBaseTrackingCost(taskFile, centroidalModelInfo_, false));

        // Constraint terms
        // friction cone settings
        scalar_t frictionCoefficient = 0.7;
        RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        std::tie(frictionCoefficient, barrierPenaltyConfig) =
                loadFrictionConeSettings(taskFile, verbose);

        bool useAnalyticalGradientsConstraints = false;
        loadData::loadCppDataType(
            taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints",
            useAnalyticalGradientsConstraints);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
            const std::string &footName = modelSettings_.contactNames3DoF[i];

            std::unique_ptr<EndEffectorKinematics<scalar_t> > eeKinematicsPtr;
            if (useAnalyticalGradientsConstraints) {
                throw std::runtime_error(
                    "[LeggedInterface::setupOptimalConrolProblem] The analytical "
                    "end-effector linear constraint is not implemented!");
            }
            const auto infoCppAd = centroidalModelInfo_.toCppAd();
            const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(
                infoCppAd);
            auto velocityUpdateCallback =
                    [&infoCppAd](const ad_vector_t &state,
                                 PinocchioInterfaceCppAd &pinocchioInterfaceAd) {
                const ad_vector_t q =
                        centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
                updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
            };
            eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(
                *pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

            if (useHardFrictionConeConstraint_) {
                problemPtr_->inequalityConstraintPtr->add(
                    footName + "_frictionCone",
                    getFrictionConeConstraint(i, frictionCoefficient));
            } else {
                problemPtr_->softConstraintPtr->add(
                    footName + "_frictionCone",
                    getFrictionConeSoftConstraint(i, frictionCoefficient,
                                                  barrierPenaltyConfig));
            }
            problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce",
                                                    getZeroForceConstraint(i));
            problemPtr_->equalityConstraintPtr->add(
                footName + "_zeroVelocity",
                getZeroVelocityConstraint(*eeKinematicsPtr, i,
                                          useAnalyticalGradientsConstraints));
            problemPtr_->equalityConstraintPtr->add(
                footName + "_normalVelocity",
                getNormalVelocityConstraint(*eeKinematicsPtr, i,
                                            useAnalyticalGradientsConstraints));
        }

        // Pre-computation
        problemPtr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
            *pinocchioInterfacePtr_, centroidalModelInfo_,
            *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);

        // Rollout
        rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(
            *problemPtr_->dynamicsPtr, rolloutSettings_);

        // Initialization
        constexpr bool extendNormalizedMomentum = true;
        initializerPtr_ = std::make_unique<LeggedRobotInitializer>(
            centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum);
    }


    std::shared_ptr<GaitSchedule> LeggedInterface::loadGaitSchedule(
        const std::string &file,
        bool verbose) const {
        const auto initModeSchedule =
                loadModeSchedule(file, "initialModeSchedule", false);
        const auto defaultModeSequenceTemplate =
                loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

        const auto defaultGait = [&] {
            Gait gait{};
            gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
            // Events: from time -> phase
            std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1,
                          defaultModeSequenceTemplate.switchingTimes.end() - 1,
                          [&](double eventTime) {
                              gait.eventPhases.push_back(eventTime / gait.duration);
                          });
            // Modes:
            gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
            return gait;
        }();

        // display
        if (verbose) {
            std::cerr << "\n#### Modes Schedule: ";
            std::cerr << "\n#### "
                    "============================================================="
                    "================\n";
            std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
            std::cerr << "Default Modes Sequence Template: \n"
                    << defaultModeSequenceTemplate;
            std::cerr << "#### "
                    "============================================================="
                    "================\n";
        }

        return std::make_shared<GaitSchedule>(
            initModeSchedule, defaultModeSequenceTemplate,
            modelSettings_.phaseTransitionStanceTime);
    }


    matrix_t LeggedInterface::initializeInputCostWeight(
        const std::string &taskFile,
        const CentroidalModelInfo &info) {
        const size_t totalContactDim = 3 * info.numThreeDofContacts;

        vector_t initialState(centroidalModelInfo_.stateDim);
        loadData::loadEigenMatrix(taskFile, "initialState", initialState);

        const auto &model = pinocchioInterfacePtr_->getModel();
        auto &data = pinocchioInterfacePtr_->getData();
        const auto q = centroidal_model::getGeneralizedCoordinates(
            initialState, centroidalModelInfo_);
        computeJointJacobians(model, data, q);
        updateFramePlacements(model, data);

        matrix_t baseToFeetJacobians(totalContactDim, info.actuatedDofNum);
        for (size_t i = 0; i < info.numThreeDofContacts; i++) {
            matrix_t jacobianWorldToContactPointInWorldFrame =
                    matrix_t::Zero(6, info.generalizedCoordinatesNum);
            getFrameJacobian(
                model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]),
                pinocchio::LOCAL_WORLD_ALIGNED,
                jacobianWorldToContactPointInWorldFrame);

            baseToFeetJacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
                    jacobianWorldToContactPointInWorldFrame.block(0, 6, 3,
                                                                  info.actuatedDofNum);
        }

        matrix_t R_taskspace(totalContactDim + totalContactDim,
                             totalContactDim + totalContactDim);
        loadData::loadEigenMatrix(taskFile, "R", R_taskspace);

        matrix_t R = matrix_t::Zero(info.inputDim, info.inputDim);
        // Contact Forces
        R.topLeftCorner(totalContactDim, totalContactDim) =
                R_taskspace.topLeftCorner(totalContactDim, totalContactDim);
        // Joint velocities
        R.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
                baseToFeetJacobians.transpose() *
                R_taskspace.bottomRightCorner(totalContactDim, totalContactDim) *
                baseToFeetJacobians;
        return R;
    }


    std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(
        const std::string &taskFile,
        const CentroidalModelInfo &info,
        bool verbose) {
        matrix_t Q(info.stateDim, info.stateDim);
        loadData::loadEigenMatrix(taskFile, "Q", Q);
        matrix_t R = initializeInputCostWeight(taskFile, info);

        if (verbose) {
            std::cerr << "\n #### Base Tracking Cost Coefficients: ";
            std::cerr << "\n #### "
                    "============================================================="
                    "================\n";
            std::cerr << "Q:\n" << Q << "\n";
            std::cerr << "R:\n" << R << "\n";
            std::cerr << " #### "
                    "============================================================="
                    "================\n";
        }

        return std::make_unique<LeggedRobotStateInputQuadraticCost>(
            std::move(Q), std::move(R), info, *referenceManagerPtr_);
    }


    std::pair<scalar_t, RelaxedBarrierPenalty::Config>
    LeggedInterface::loadFrictionConeSettings(const std::string &taskFile,
                                                   bool verbose) const {
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        const std::string prefix = "frictionConeSoftConstraint.";

        scalar_t frictionCoefficient = 1.0;
        RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        if (verbose) {
            std::cerr << "\n #### Friction Cone Settings: ";
            std::cerr << "\n #### "
                    "============================================================="
                    "================\n";
        }
        loadData::loadPtreeValue(pt, frictionCoefficient,
                                 prefix + "frictionCoefficient", verbose);
        loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
        loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta",
                                 verbose);
        if (verbose) {
            std::cerr << " #### "
                    "============================================================="
                    "================\n";
        }

        return {frictionCoefficient, std::move(barrierPenaltyConfig)};
    }


    std::unique_ptr<StateInputConstraint>
    LeggedInterface::getFrictionConeConstraint(size_t contactPointIndex,
                                                    scalar_t frictionCoefficient) {
        FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
        return std::make_unique<FrictionConeConstraint>(
            *referenceManagerPtr_, std::move(frictionConeConConfig),
            contactPointIndex, centroidalModelInfo_);
    }


    std::unique_ptr<StateInputCost>
    LeggedInterface::getFrictionConeSoftConstraint(
        size_t contactPointIndex,
        scalar_t frictionCoefficient,
        const RelaxedBarrierPenalty::Config &barrierPenaltyConfig) {
        return std::make_unique<StateInputSoftConstraint>(
            getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
            std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
    }


    std::unique_ptr<StateInputConstraint>
    LeggedInterface::getZeroForceConstraint(size_t contactPointIndex) {
        return std::make_unique<ZeroForceConstraint>(
            *referenceManagerPtr_, contactPointIndex, centroidalModelInfo_);
    }


    std::unique_ptr<StateInputConstraint>
    LeggedInterface::getZeroVelocityConstraint(
        const EndEffectorKinematics<scalar_t> &eeKinematics,
        size_t contactPointIndex,
        bool useAnalyticalGradients) {
        auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
            EndEffectorLinearConstraint::Config config;
            config.b.setZero(3);
            config.Av.setIdentity(3, 3);
            if (!numerics::almost_eq(positionErrorGain, 0.0)) {
                config.Ax.setZero(3, 3);
                config.Ax(2, 2) = positionErrorGain;
            }
            return config;
        };

        if (useAnalyticalGradients) {
            throw std::runtime_error(
                "[LeggedInterface::getZeroVelocityConstraint] The analytical "
                "end-effector zero velocity constraint is not implemented!");
        }
        return std::make_unique<ZeroVelocityConstraintCppAd>(
            *referenceManagerPtr_, eeKinematics, contactPointIndex,
            eeZeroVelConConfig(modelSettings_.positionErrorGain));
    }


    std::unique_ptr<StateInputConstraint>
    LeggedInterface::getNormalVelocityConstraint(
        const EndEffectorKinematics<scalar_t> &eeKinematics,
        size_t contactPointIndex,
        bool useAnalyticalGradients) {
        if (useAnalyticalGradients) {
            throw std::runtime_error(
                "[LeggedInterface::getNormalVelocityConstraint] The analytical "
                "end-effector normal velocity constraint is not implemented!");
        }
        return std::make_unique<NormalVelocityConstraintCppAd>(
            *referenceManagerPtr_, eeKinematics, contactPointIndex);
    }
} // namespace ocs2::legged_robot
