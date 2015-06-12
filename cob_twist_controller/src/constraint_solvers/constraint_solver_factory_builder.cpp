/*
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2015
 *
 * \brief
 *   This header contains the description of a class providing a static method to create constraint solver factory objects.
 *
 ****************************************************************/

#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>
#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"
#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/wln_joint_limit_avoidance_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"

#include "cob_twist_controller/constraint_solvers/solvers/gradient_projection_method_solver.h"

#include "cob_twist_controller/constraint_solvers/constraint_solver_factory_builder.h"
#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/constraint_solvers/factories/solver_factory.h"

#include "cob_twist_controller/constraints/constraint.h"

/**
 * Out of the parameters generates a damping method (e.g. constant or manipulability) and calculates the damping factor.
 * Dependent on JLA active flag a JointLimitAvoidanceSolver or a UnconstraintSolver is generated to solve the IK problem.
 * The objects are generated for each solve-request. After calculation the objects are deleted.
 */
int8_t ConstraintSolverFactoryBuilder::calculateJointVelocities(AugmentedSolverParams &asParams,
                                                                Matrix6Xd &jacobianData,
                                                                const Vector6d &inCartVelocities,
                                                                const KDL::JntArray& q,
                                                                const KDL::JntArray& last_q_dot,
                                                                Eigen::MatrixXd &outJntVelocities)
{
    outJntVelocities = Eigen::MatrixXd::Zero(last_q_dot.rows(), last_q_dot.columns());
    boost::shared_ptr<DampingBase> db (DampingBuilder::create_damping(asParams, jacobianData));
    if(NULL == db)
    {
        ROS_ERROR("Returning NULL factory due to damping creation error.");
        return -1; // error
    }

    std::set<tConstraintBase> constraints = ConstraintsBuilder<>::createConstraints(asParams,
                                                                                     q,
                                                                                     jacobianData,
                                                                                     this->jnt_to_jac_,
                                                                                     this->data_mediator_);

    boost::shared_ptr<ISolverFactory> sf;
    switch(asParams.constraint)
    {
        case None:
            sf.reset(new SolverFactory<UnconstraintSolver>());
            break;
        case WLN:
            sf.reset(new SolverFactory<WeightedLeastNormSolver>());
            break;
        case WLN_JLA:
            sf.reset(new SolverFactory<WLN_JointLimitAvoidanceSolver>());
            break;
        case GPM_JLA:
            sf.reset(new SolverFactory<GradientProjectionMethodSolver>());
            break;
        case GPM_JLA_MID:
            sf.reset(new SolverFactory<GradientProjectionMethodSolver>());
            break;
        case GPM_CA:
            sf.reset(new SolverFactory<GradientProjectionMethodSolver>());
            break;
        default:
            ROS_ERROR("Returning NULL factory due to constraint solver creation error. There is no solver method for %d implemented.", asParams.constraint);
            break;
    }

    if (NULL != sf)
    {
        outJntVelocities = sf->calculateJointVelocities(asParams,
                                                        jacobianData,
                                                        inCartVelocities,
                                                        q,
                                                        last_q_dot,
                                                        db,
                                                        constraints);
    }
    else
    {
        return -2; // error: no valid selection for constraint
    }

    sf.reset();
    db.reset();

    return 0; // success
}
