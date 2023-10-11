// SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
// SPDX-License-Identifier: MIT License

#include <rclcpp/rclcpp.hpp>

#include <wili_msgs/msg/hmm.hpp>
#include <wili_msgs/srv/get_hmm.hpp>

#include "../include/wili_hmm/baum_welch.hpp"

void BaumWelch::set_hmm_parameters(wili_msgs::msg::HMM hmm){
    // rclcpp::Logger logger = this->get_logger();

    // number of motions
    motion_num = hmm.motion_num;
    // RCLCPP_INFO(logger, "motion_num=%u", motion_num);

    // transition probabilities
    tr_prob = new float*[motion_num];
    for(int i = 0; i < motion_num; ++i){
        tr_prob[i] = new float[motion_num];
        int d = i * motion_num;
        for(int j = 0; j < motion_num; ++j){
            tr_prob[i][j] = hmm.tr_prob[d + j];
        }
    }

    // averages & covariance matrixs
    avr = new float*[motion_num];
    var = new float*[motion_num];
    for(int i = 0; i < motion_num; ++i){
        avr[i] = new float[2];
        var[i] = new float[3];

        avr[i][0] = hmm.heatmaps[i].gaussian.avr_x;
        avr[i][1] = hmm.heatmaps[i].gaussian.avr_y;

        var[i][0] = hmm.heatmaps[i].gaussian.var_xx;
        var[i][1] = hmm.heatmaps[i].gaussian.var_xy;
        var[i][2] = hmm.heatmaps[i].gaussian.var_yy;
    }
}

BaumWelch::BaumWelch() : Node("baum_welch"){
}

BaumWelch::~BaumWelch(){
    for(int i = 0; i < motion_num; ++i){
        delete tr_prob[i];
        delete avr[i];
        delete var[i];
    }
    delete tr_prob;
    delete avr;
    delete var;
}

bool try_init_hmm_parameters(std::shared_ptr<BaumWelch> node_ptr){
    rclcpp::Client<wili_msgs::srv::GetHMM>::SharedPtr cli_hmm;
    cli_hmm = node_ptr->create_client<wili_msgs::srv::GetHMM>("get_hmm");

    if(!cli_hmm->wait_for_service())
        return false;

    if(!rclcpp::ok())
        return false;

    auto future = cli_hmm->async_send_request(
        std::make_shared<wili_msgs::srv::GetHMM::Request>()
    );

    if(
        rclcpp::spin_until_future_complete(node_ptr, future) == 
        rclcpp::FutureReturnCode::SUCCESS
    ){
        node_ptr->set_hmm_parameters(future.get()->hmm);
        RCLCPP_INFO(node_ptr->get_logger(), "initialized hmm parameters");
        return true;
    }
    else{
        return false;
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<BaumWelch>();
    if(try_init_hmm_parameters(node_ptr)){
        RCLCPP_INFO(node_ptr->get_logger(), "start");
        rclcpp::spin(node_ptr);
    }
    else{
        RCLCPP_INFO(node_ptr->get_logger(), "failed to initialize hmm parameters");
    }
    rclcpp::shutdown();
    return 0;
}