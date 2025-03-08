#pragma once
#include "warper.h"
#include <dlib/dnn.h>
#include <dlib/rand.h>

using namespace dlib;
namespace USTC_CG{

  using net_type = loss_mean_squared_multioutput<
  fc<2,
  relu<fc<100,
  relu<fc<50,
  input<matrix<float>>
  >>>>>>;
class NNWarper : public Warper
{

  public:
    NNWarper() = default;
    virtual ~NNWarper() = default;
    std::pair<float,float> warp(float x,float y) const override;
    net_type train();  // 新增接口
    void set_control_points(
        const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points) override; // Set control points

  private:
    std::vector<std::pair<float, float>> start_points_;
    std::vector<std::pair<float, float>> end_points_;
    net_type net = net_type();
    /* data */    
};
} // namespace USTC_CG

    