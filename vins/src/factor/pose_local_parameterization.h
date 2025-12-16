/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include "../utility/utility.h"

#include <memory>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include "../utility/utility.h"

class PoseLocalParameterization : public ceres::Manifold {
public:
    PoseLocalParameterization() {
        manifold_ = std::unique_ptr<ceres::Manifold>(
            new ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::EigenQuaternionManifold>(
                ceres::EuclideanManifold<3>(), ceres::EigenQuaternionManifold()));
    }
    
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        return manifold_->Plus(x, delta, x_plus_delta);
    }
    bool PlusJacobian(const double* x, double* jacobian) const override {
        return manifold_->PlusJacobian(x, jacobian);
    }
    bool Minus(const double* y, const double* x, double* delta) const override {
        return manifold_->Minus(y, x, delta);
    }
    bool MinusJacobian(const double* x, double* jacobian) const override {
        return manifold_->MinusJacobian(x, jacobian);
    }
    int AmbientSize() const override { return manifold_->AmbientSize(); }
    int TangentSize() const override { return manifold_->TangentSize(); }

private:
    std::unique_ptr<ceres::Manifold> manifold_;
};

/*
class PoseLocalParameterization : public ceres::Manifold
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    virtual bool PlusJacobian(const double *x, double *jacobian) const override;
    virtual int AmbientSize() const override { return 7; };
    virtual int TangentSize() const override { return 6; };
};
*/
