//
// Created by hc on 2020/3/14.
//

#ifndef ARUCO_LOCALIZATION_G2O_TYPES_H
#define ARUCO_LOCALIZATION_G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <sophus/se3.hpp>

using namespace Sophus;
using namespace g2o;

namespace Aruco_LOCA{

    Matrix6d JRInv(const SE3d &e);

    class VertexSE3LieAlegebra : public g2o::BaseVertex<6, SE3d> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual bool read(std::istream &in) override {}
        virtual bool write(std::ostream &out) const override {}

        virtual void setToOriginImpl() override {
            _estimate = SE3d();
        }

        virtual void oplusImpl(const double *update){
            Vector6d upd;
            upd <<update[0],update[1],update[2],update[3],update[4],update[5];
            _estimate = SE3d::exp(upd) * _estimate;
        }
    };

    //两个李代数节点之边
    class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, SE3d , VertexSE3LieAlegebra, VertexSE3LieAlegebra> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static int i;

        virtual bool read(std::istream &in) override {}
        virtual bool write(std::ostream &out) const override {}

        virtual void computeError() override {
            SE3d v1 = (static_cast<VertexSE3LieAlegebra*>(_vertices[0]))->estimate();
            SE3d v2 = (static_cast<VertexSE3LieAlegebra*>(_vertices[1]))->estimate();
            _error = (_measurement.inverse() * v2.inverse() * v1).log();
            //LOG(INFO) << _error.transpose() << " i: " <<i++;
        }

//    virtual void linearizeOplus() override {
//        SE3d v1 = (static_cast<VertexSE3LieAlegebra*>(_vertices[0]))->estimate();
//        SE3d v2 = (static_cast<VertexSE3LieAlegebra*>(_vertices[1]))->estimate();
//        Matrix6d J = JRInv(SE3d::exp(_error));
//
//        _jacobianOplusXi = -J * v2.inverse().Adj();
//        _jacobianOplusXj = J * v2.inverse().Adj();
//    }
    };
}
#endif //ARUCO_LOCALIZATION_G2O_TYPES_H
