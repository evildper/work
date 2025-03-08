#include "IDW_warper.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;
namespace USTC_CG
{
    void IDWWarper::get_size(int x,int y) 
    {
        width_ = x;
        height_ = y;
    }
    void IDWWarper::set_control_points(
        const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points)
    {
        start_points_ = start_points;
        end_points_ = end_points;
        if(start_points_.size()<=2){
            start_points_.push_back({0,0});
            start_points_.push_back({0,1});
            start_points_.push_back({width_-1,height_-1});
            end_points_.push_back({width_-1,height_-1});
            end_points_.push_back({0,0});
            end_points_.push_back({0,1});
        }
    }
    std::vector<Matrix2d> IDWWarper::get_matrix(const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points) const
    {
        int n = start_points.size();
        std::vector<Matrix2d> T_list(n);
        for (int i = 0; i < n; ++i)
        {
            Matrix2d A = Matrix2d::Zero();
            Matrix2d B = Matrix2d::Zero();
            Vector2d p_i(start_points[i].first, start_points[i].second);
            Vector2d q_i(end_points[i].first, end_points[i].second);
            for(int j = 0;j<n;j++){
                if(i==j)
                    continue;
                Vector2d p_j(start_points[j].first, start_points[j].second);
                Vector2d q_j(end_points[j].first, end_points[j].second);
                Vector2d diff_p = p_j - p_i;
                Vector2d diff_q = q_j - q_i;
                double r = diff_p.norm();
                
                double sigma_i_j = 1.0 / (r*r);
                A += sigma_i_j * diff_p * diff_p.transpose();
                B += sigma_i_j * diff_q * diff_p.transpose();

            }
            T_list[i] = B * A.inverse();
        }
        return T_list;


    }
    double IDWWarper::sum_of_sigma(const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points,Vector2d p) const
    {
        double sum = 0;
        for(int i = 0;i<start_points.size();i++){
            Vector2d p_i(start_points[i].first, start_points[i].second);
            Vector2d q_i(end_points[i].first, end_points[i].second);
            Vector2d diff_p = p_i - p;
            double r = diff_p.norm();
            double sigma_i = 1.0 / (r*r);
            sum += sigma_i;
        }
        return sum;
    }
    std::pair<float, float> IDWWarper::warp(float x, float y) const 
    {
        if (start_points_.empty() || end_points_.empty() || start_points_.size() != end_points_.size())
        {
            std::cerr << "Invalid control points" << std::endl;
            return {x, y};
        }
        std::vector<Matrix2d> T_list = get_matrix(start_points_,end_points_);
        Vector2d p(x, y);
        Vector2d result = Vector2d::Zero();
        for(int i = 0;i<start_points_.size();i++){
            Vector2d p_i(start_points_[i].first, start_points_[i].second);
            Vector2d q_i(end_points_[i].first, end_points_[i].second);
            Vector2d diff_p = p - p_i;
            double r = diff_p.norm();
            double sigma_i = 1.0 / (r*r+1e-6);
            double sum = sum_of_sigma(start_points_,end_points_,p);
            result += (sigma_i/sum)*(q_i + T_list[i]*diff_p);

                        
        }

        return {result(0),result(1)};
    }
    
}