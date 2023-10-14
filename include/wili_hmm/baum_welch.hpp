// SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
// SPDX-License-Identifier: MIT License

#ifndef __BAUM_WELCH_HPP__
#define __BAUM_WELCH_HPP__

#include <rclcpp/rclcpp.hpp>

#include <wili_msgs/msg/hmm.hpp>
#include <wili_msgs/srv/get_hmm.hpp>

class BaumWelch : public rclcpp::Node
{
    private:
    uint8_t motion_num;
    float* init_prob;
    float** tr_prob;
    float** avrs;
    float** covars;

    float gaussian(uint8_t motion, float* where);
    float update_once(uint32_t observation_len, float** observation);

    public:
    std::shared_ptr<BaumWelch> node;

    BaumWelch();
    ~BaumWelch();

    void set_hmm_parameters(wili_msgs::msg::HMM hmm);
    void update(
        uint32_t observation_len,
        float** observation,
        float diff_liklihood_threshold
    );
};

bool try_init_hmm_parameters(std::shared_ptr<BaumWelch> node_ptr);

#endif