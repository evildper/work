#pragma once
#include "warper.h"
#include <Eigen/Dense>
namespace USTC_CG{


class RBFWarper : public Warper
{

  public:
    RBFWarper() = default;
    virtual ~RBFWarper() = default;
    std::pair<float,float> warp(float x,float y) const override;
    void set_control_points(
        const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points) override; // Set control points
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> estimate_affine(
            const std::vector<Eigen::Vector2d>& src,
            const std::vector<Eigen::Vector2d>& dst) const;    
  private:
    std::vector<std::pair<float, float>> start_points_;
    std::vector<std::pair<float, float>> end_points_;
    /* data */    
};
} // namespace USTC_CG

    
