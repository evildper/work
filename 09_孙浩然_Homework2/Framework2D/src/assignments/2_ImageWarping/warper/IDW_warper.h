// HW2_TODO: Implement the IDWWarper class
#pragma once

#include "warper.h"
#include <Eigen/Dense>
using namespace Eigen;
namespace USTC_CG
{
class IDWWarper : public Warper
{
   public:
    IDWWarper() = default;
    virtual ~IDWWarper() = default;
    // HW2_TODO: Implement the warp(...) function with IDW interpolation
    std::pair<float,float> warp(float x,float y) const override;
    void set_control_points(
        const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points) override; // Set control points
    // HW2_TODO: other functions or variables if you need
    std::vector<Matrix2d> get_matrix(const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points) const;
    double sum_of_sigma(const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points,Vector2d p) const;
    void get_size(int x,int y) ;
   private:

    // HW2_TODO: other functions or variables if you need
    std::vector<std::pair<float, float>> start_points_;
    std::vector<std::pair<float, float>> end_points_;
    int width_;
    int height_;
};
}  // namespace USTC_CG