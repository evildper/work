#include "RBF_warper.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;
namespace USTC_CG
{
    void RBFWarper::set_control_points(
        const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points)
    {
        start_points_ = start_points;
        end_points_ = end_points;
    }
    double find_min(const std::vector<std::pair<float, float>>& start_points, float x, float y)
    {
        double min = 1000000;
        for (int i = 0; i < start_points.size(); i++)
        {
            double dis = sqrt(pow(start_points[i].first - x, 2) + pow(start_points[i].second - y, 2));
            if (dis < min && dis != 0)
            {
                min = dis;
            }
        }
        return min;
    }
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> RBFWarper::estimate_affine(
        const std::vector<Eigen::Vector2d>& src,
        const std::vector<Eigen::Vector2d>& dst) const
    {
        int n = src.size();
    
        Eigen::MatrixXd P(n, 3);
        Eigen::MatrixXd Q(n, 2);
        for(int i = 0; i < n; i++)
        {
            P(i, 0) = src[i](0);
            P(i, 1) = src[i](1);
            P(i, 2) = 1.0f;
            Q(i, 0) = dst[i](0);
            Q(i, 1) = dst[i](1);
        }
        
    
        Eigen::MatrixXd X = (P.transpose() * P).ldlt().solve(P.transpose() * Q);
    
        Eigen::Matrix2d A = X.block<2, 2>(0, 0);
        Eigen::Vector2d b = X.block<1, 2>(2, 0).transpose();//block<行数，列数>(起始行，起始列)
    
        return {A, b};
    }
    std::pair<float, float> RBFWarper::warp(float x, float y) const 
    {
        int n = start_points_.size();
        
        MatrixXd K(n, n);
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                double dx = start_points_[i].first - start_points_[j].first;
                double dy = start_points_[i].second - start_points_[j].second;
                double r = std::sqrt(dx * dx + dy * dy);
                double r_i = find_min(start_points_, start_points_[i].first, start_points_[i].second);
                K(i, j) = std::sqrt(r * r + r_i * r_i);
            }
        }
        Matrix2d A;
        Vector2d b;
        std::vector<Vector2d> start_points_vector2d;
        std::vector<Vector2d> end_points_vector2d;

        // 转换 start_points_ 到 start_points_vector2d
        for (const auto& point : start_points_) {
            start_points_vector2d.emplace_back(point.first, point.second);
        }

        // 转换 end_points_ 到 end_points_vector2d
        for (const auto& point : end_points_) {
            end_points_vector2d.emplace_back(point.first, point.second);
        }
        std::tie(A, b) = estimate_affine(start_points_vector2d, end_points_vector2d);
        Vector2d p;
        Eigen::MatrixXd P(2, n);
        Eigen::MatrixXd Q(2, n);
        for(int i = 0;i<n;i++){
            p=end_points_vector2d[i] - A * start_points_vector2d[i] - b;
            P(0,i)=p(0);
            P(1,i)=p(1);
        }
        Eigen::MatrixXd X = (K.transpose() ).ldlt().solve(P.transpose());
        Q = X.transpose();
        Vector2d src(x, y);
        Eigen::MatrixXd W(n, 1);
        for (int i = 0; i < n; ++i)
        {
            double dx = start_points_[i].first - x;
            double dy = start_points_[i].second - y;
            double r = std::sqrt(dx * dx + dy * dy);
            double r_i = find_min(start_points_, start_points_[i].first, start_points_[i].second);
            W(i, 0) = std::sqrt(r * r + r_i * r_i);
        }
        src = A * src + b + Q * W;
        return {src(0), src(1)};
    }
}  // namespace USTC_CG    

