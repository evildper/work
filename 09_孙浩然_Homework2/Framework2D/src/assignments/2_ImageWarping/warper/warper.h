// HW2_TODO: Please implement the abstract class Warper
// 1. The Warper class should abstract the **mathematical mapping** involved in
// the warping problem, **independent of image**.
// 2. The Warper class should have a virtual function warp(...) to be called in
// our image warping application.
//    - You should design the inputs and outputs of warp(...) according to the
//    mathematical abstraction discussed in class.
//    - Generally, the warping map should map one input point to another place.
// 3. Subclasses of Warper, IDWWarper and RBFWarper, should implement the
// warp(...) function to perform the actual warping.
#pragma once

#include <vector>
#include <utility>
namespace USTC_CG
{
class Warper
{
   public:
    virtual ~Warper() = default;
    
    // HW2_TODO: A virtual function warp(...)
    virtual std::pair<float,float>  warp(float x,float y) const= 0;
    virtual void set_control_points(
        const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points) = 0; // Set control points

   protected:
    std::vector<std::pair<float, float>> start_points_;
    std::vector<std::pair<float, float>> end_points_;
    // HW2_TODO: other functions or variables if you need
};
}  // namespace USTC_CG