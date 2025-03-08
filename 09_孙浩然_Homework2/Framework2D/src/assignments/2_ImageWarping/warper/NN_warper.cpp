#include "NN_warper.h"
#include <iostream>
#include <cmath>
using namespace dlib;

namespace USTC_CG
{


    void NNWarper::set_control_points(
        const std::vector<std::pair<float, float>>& start_points,
        const std::vector<std::pair<float, float>>& end_points)
    {
        start_points_ = start_points;
        end_points_ = end_points;
    }
    
    net_type NNWarper::train()
    {
        const int num_samples = start_points_.size();
        dlib::rand rnd;
        std::vector<matrix<float>> inputs;
        std::vector<matrix<float>> outputs;

        for (int i = 0; i < num_samples; ++i) {
            matrix<float> input(2,1);
            input(0,0) = start_points_[i].first;
            input(1,0) = start_points_[i].second;
            
            matrix<float> output(2,1);
            output(0,0) = end_points_[i].first;
            output(1,0) = end_points_[i].second;
            
            inputs.push_back(input);
            outputs.push_back(output);
        }

        // 3. Split dataset
        auto train_inputs = std::vector<matrix<float>>(inputs.begin(), inputs.begin()+num_samples);
        auto train_outputs = std::vector<matrix<float>>(outputs.begin(), outputs.begin()+num_samples);

        net_type net;
        dnn_trainer<net_type, adam> trainer(net);

        trainer.set_learning_rate(0.001);
        trainer.set_min_learning_rate(1e-6);
        trainer.set_mini_batch_size(128);
        trainer.set_iterations_without_progress_threshold(100);
        trainer.be_verbose();

        // 6. Training process
        std::cout << "Starting training..." << std::endl;
        trainer.train(train_inputs, train_outputs);
        std::cout << "NN Warper Training Done!" << std::endl;
        this->net = net;
        return net;
    }

    std::pair<float, float> NNWarper::warp(float x, float y) const
    {
        
        matrix<float> input(2, 1);
        input(0) = x;
        input(1) = y;
        net_type net = this->net;
        //matrix<float,2,1> result(2,1); 
        std::vector<matrix<float>> input_vector = {input};
        matrix<float,2,1> result = net(input_vector)[0];

        return {result(0), result(1)};
    }
}


