// SPDX-FileCopyrightText: 2023 ShinagwaKazemaru
// SPDX-License-Identifier: MIT License

#include <rclcpp/rclcpp.hpp>

#include <wili_msgs/msg/hmm.hpp>
#include <wili_msgs/msg/observation.hpp>
#include <wili_msgs/srv/get_hmm.hpp>

#include "../include/wili_hmm/baum_welch.hpp"

// calculate Gaussian of each motion
float BaumWelch::gaussian(uint8_t motion, float* where){
    float x_ = where[0] - avrs[motion][0];
    float y_ = where[1] - avrs[motion][1];

    float det_sigma = covars[motion][0] * covars[motion][2] - covars[motion][1] * covars[motion][1];

    float ind = covars[motion][2] * x_ * x_;
    ind += 2 * covars[motion][1] * x_ * y_;
    ind += covars[motion][0] * y_ * y_;
    ind /= det_sigma;
    ind *= -0.5;

    float e = expf(ind);
    e /= 2 * M_PI * sqrtf(det_sigma);

    return e;
}

// update hmm parameters
// return value is logP(obs)
float BaumWelch::update_once(uint32_t observation_len, float** observation){
    uint32_t t_max = observation_len - 1;

    float** b = new float*[motion_num]; // gaussian cache
    for(uint8_t i = 0; i < motion_num; ++i){
        b[i] = new float[observation_len];
        for(uint32_t t = 0; t <= t_max; ++t){
            b[i][t] = gaussian(i, observation[t]);
            RCLCPP_INFO(get_logger(), "b[%u][%u]=%f", i, t, b[i][t]);
        }
    }


    // ---------------------------------------------------
    // calculate forward probability & scaling coefficient
    // ---------------------------------------------------

    float** alpha = new float*[observation_len]; // forward prob
    float* c = new float[observation_len];  // scaling coef
    c[0] = 1;

    //init
    alpha[0] = new float[motion_num];
    for(uint8_t i = 0; i < motion_num; ++i)
        alpha[0][i] = init_prob[i] * b[i][0];

    // recurse
    for(uint32_t t = 1; t <= t_max; ++t){
        alpha[t] = new float[motion_num];
        for(uint8_t i = 0; i < motion_num; ++i){
            alpha[t][i] = 0;
            for(uint8_t j = 0; j < motion_num; ++j)
                alpha[t][i] += alpha[t - 1][j] * tr_prob[j][i] * b[i][t];
        }

        // calc c
        c[t] = 0;
        for(uint8_t i = 0; i < motion_num; ++i)
            c[t] += alpha[t][i];

        // normalize
        // sum_i alpha_ti = 1
        for(uint8_t i = 0; i < motion_num; ++i)
            alpha[t][i] /= c[t];
    }


    // ------------------------------
    // calculate backward probability
    // ------------------------------

    float** beta = new float*[observation_len]; // backward prob

    // init
    beta[t_max] = new float[motion_num];
    for(uint8_t i = 0; i < motion_num; ++i)
        beta[t_max][i] = 1;

    // recurse
    // * * * notion about t range * * *
    // type of t is unsigned
    // =>
    // t=0 --> --t --> underflow !
    // * * * * * * * * * * * * * * * * *
    for(uint32_t t = t_max - 1; t < t_max; --t){
        beta[t] = new float[motion_num];

        for(uint8_t i = 0; i < motion_num; ++i){
            beta[t][i] = 0;
            for(uint8_t j = 0; j < motion_num; ++j)
                beta[t][i] += tr_prob[i][j] * b[j][t] * beta[t + 1][j];
            beta[t][i] /= c[t];
        }
    }

    // ------------------------------------
    // calculate new transition probability
    // ------------------------------------

    float** new_tr_prob = new float*[motion_num];
    for(uint8_t i = 0; i < motion_num; ++i){
        new_tr_prob[i] = new float[motion_num];
        float sum = 0;

        // calc new transition prob
        for(uint8_t j = 0; j < motion_num; ++j){
            new_tr_prob[i][j] = 0;
            for(uint32_t t = 0; t <= t_max - 1; ++t){
                float temp = alpha[t][i] * tr_prob[i][j] * b[j][t + 1] * beta[t + 1][j];
                temp /= c[t];
                new_tr_prob[i][j] += temp;
            }
            sum += new_tr_prob[i][j];
        }

        RCLCPP_INFO(get_logger(), "sum_%u a=%f", i, sum);

        // normalize
        // sum_j a_ij = 1
        for(uint8_t j = 0; j < motion_num; ++j)
            new_tr_prob[i][j] /= sum;
    }

    // swap
    for(uint8_t i = 0; i < motion_num; ++i){
        for(uint8_t j = 0; j < motion_num; ++j){
            tr_prob[i][j] = new_tr_prob[i][j];
            RCLCPP_INFO(get_logger(), "new tr_prob[%u][%u]=%f", i, j, tr_prob[i][j]);
        }
        delete new_tr_prob[i];
    }
    delete new_tr_prob;


    // --------------------------------
    // calculate logP(o; old parameter)
    // this is the return value
    // --------------------------------

    float liklihood = 0; // logP(o) , return value
    // ommit t=0 because c[t]=1
    for(uint32_t t = 1; t <= t_max; ++t)
        liklihood += logf(c[t]);

    RCLCPP_INFO(get_logger(), "liklihood=%f", liklihood);

    // delete no use array
    delete c;
    for(uint8_t i = 0; i < motion_num; ++i)
        delete b[i];
    delete b;


    // --------------------------------
    // calculate new Gaussian parameter
    // --------------------------------

    float** gamma = new float*[observation_len]; // P(s_t= i| o; old parameters)

    // gamma = alpha * beta
    for(uint32_t t = 0; t <= t_max; ++t){
        gamma[t] = new float[motion_num];
        float sum = 0;
        for(uint8_t i = 0; i < motion_num; ++i){
            gamma[t][i] = alpha[t][i] * beta[t][i];
            sum += gamma[t][i];
        }

        // normalize
        // to reduce error
        for(uint8_t i = 0; i < motion_num; ++i)
            gamma[t][i] /= sum;
    }

    // delete no use array
    for(uint32_t t = 0; t <= t_max; ++t){
        delete alpha[t];
        delete beta[t];
    }
    delete alpha;
    delete beta;

    // calc
    for(uint8_t i = 0; i < motion_num; ++i){
        float sum_gamma = 0; // sum_t gamma
        for(uint32_t t = 0; t <= t_max; ++t)
            sum_gamma += gamma[t][i];

        //calc new average of Gaussian
        float* new_avr = new float[2]; // average
        for(uint8_t x = 0; x < 2; ++x){
            new_avr[x] = 0;
            for(uint32_t t = 0; t <= t_max; ++t)
                new_avr[x] += gamma[t][i] * observation[t][x];
            new_avr[x] /= sum_gamma;
        }

        RCLCPP_INFO(get_logger(), "new avrs[%u]=(%f,%f)", i, new_avr[0], new_avr[1]);

        // swap average
        for(uint8_t x = 0; x < 2; ++x)
            avrs[i][x] = new_avr[x];
        delete new_avr;

        // calc new covariance of Gaussian
        float new_cov_xx = 0; // covariance(x,x) (= var x)
        float new_cov_xy = 0; // covariance(x,y)
        float new_cov_yy = 0; // covariance(y,y) (= var y)
        // detが0になっちゃう-->次のループで破綻 要理論再確認
        for(uint32_t t = 0; t <= t_max; ++t){
            float x_ = observation[t][0] - avrs[i][0];
            float y_ = observation[t][1] - avrs[i][1];
            new_cov_xx += gamma[t][i] * x_ * x_;
            new_cov_xy += gamma[t][i] * x_ * y_;
            new_cov_yy += gamma[t][i] * y_ * y_;
        }

        RCLCPP_INFO(get_logger(), "new coavrs[%u]=(%f,%f,%f)", i, new_cov_xx, new_cov_xy, new_cov_yy);

        // set covariance
        covars[i][0] = new_cov_xx;
        covars[i][1] = new_cov_xy;
        covars[i][2] = new_cov_yy;
    }


    // ----------------------------------------
    // calculate new initial motion probability
    // ----------------------------------------

    for(uint8_t i = 0; i < motion_num; ++i)
        init_prob[i] = gamma[0][i];

    // delete gamma
    for(uint32_t t = 0; t <= t_max; ++t)
        delete gamma[t];
    delete gamma;


    return liklihood;
}


// create msg of /new_hmm topic with member variables
wili_msgs::msg::HMM BaumWelch::create_msg(){
    auto msg = wili_msgs::msg::HMM();

    msg.motion_num = motion_num;

    msg.init_prob = std::vector<float>(init_prob, init_prob + motion_num);

    msg.tr_prob = std::vector<float>(motion_num * motion_num);
    for(uint16_t i = 0; i < motion_num; ++i){
        uint16_t d = i * motion_num;
        for(uint16_t j = 0; j < motion_num; ++j)
            msg.tr_prob[d + j] = tr_prob[i][j];
    }

    msg.heatmaps = std::vector<wili_msgs::msg::Heatmap>(motion_num);
    for(uint8_t i = 0; i < motion_num; ++i){
        msg.heatmaps[i].gaussian.avr_x = avrs[i][0];
        msg.heatmaps[i].gaussian.avr_y = avrs[i][1];
        msg.heatmaps[i].gaussian.var_xx = covars[i][0];
        msg.heatmaps[i].gaussian.var_xy = covars[i][1];
        msg.heatmaps[i].gaussian.var_yy = covars[i][2];
    }

    return msg;
}


// callback function of /observation topic
void BaumWelch::cb_observation(const std::shared_ptr<wili_msgs::msg::Observation> msg){
    RCLCPP_INFO(get_logger(), "subscribed /observation");

    uint32_t observation_len = msg->data.size();
    float** observation = new float*[observation_len];
    for(uint32_t t = 0; t < observation_len; ++t){
        observation[t] = new float[2];
        observation[t][0] = msg->data[t].x;
        observation[t][1] = msg->data[t].y;
    }

    update(observation_len, observation, 0.05);

    for(uint32_t t = 0; t < observation_len; ++t)
        delete observation[t];
    delete observation;

    pub_newhmm->publish(create_msg());

    return;
}


BaumWelch::BaumWelch() : Node("baum_welch"){
    this-> sub_observation = this->create_subscription<wili_msgs::msg::Observation>(
        "observation",
        3,
        std::bind(&BaumWelch::cb_observation, this, std::placeholders::_1)
    );
    this->pub_newhmm = this->create_publisher<wili_msgs::msg::HMM>("new_hmm", 3);
}


BaumWelch::~BaumWelch(){
    for(int i = 0; i < motion_num; ++i){
        delete tr_prob[i];
        delete avrs[i];
        delete covars[i];
    }
    delete init_prob;
    delete tr_prob;
    delete avrs;
    delete covars;
}


void BaumWelch::set_hmm_parameters(wili_msgs::msg::HMM hmm){
    // rclcpp::Logger logger = this->get_logger();

    // number of motions
    motion_num = hmm.motion_num;
    // RCLCPP_INFO(logger, "motion_num=%u", motion_num);

    // initial motion probability
    init_prob = new float[motion_num];
    for(uint8_t i = 0; i < motion_num; ++i){
        init_prob[i] = hmm.init_prob[i];
    }

    // transition probabilities
    tr_prob = new float*[motion_num];
    for(uint16_t i = 0; i < motion_num; ++i){
        tr_prob[i] = new float[motion_num];
        uint16_t d = i * motion_num;
        for(uint16_t j = 0; j < motion_num; ++j)
            tr_prob[i][j] = hmm.tr_prob[d + j];
    }

    // averages & covariance matrixs o Gaussian
    avrs = new float*[motion_num];
    covars = new float*[motion_num];
    for(uint8_t i = 0; i < motion_num; ++i){
        avrs[i] = new float[2];
        covars[i] = new float[3];

        avrs[i][0] = hmm.heatmaps[i].gaussian.avr_x;
        avrs[i][1] = hmm.heatmaps[i].gaussian.avr_y;

        covars[i][0] = hmm.heatmaps[i].gaussian.var_xx;
        covars[i][1] = hmm.heatmaps[i].gaussian.var_xy;
        covars[i][2] = hmm.heatmaps[i].gaussian.var_yy;
    }
}


// update hmm parameters until they converge
void BaumWelch::update(
        uint32_t observation_len,
        float** observation,
        float diff_liklihood_threshold
){
    float likilihood; // logP(o)

    // init
    likilihood = update_once(observation_len, observation);

    // loop update untill logP(o) converge
    int update_cnt = 1;
    for(; update_cnt < 2; ++update_cnt){
        float new_likilihood = update_once(observation_len, observation);
        if(new_likilihood - likilihood <= diff_liklihood_threshold)
            break;
        likilihood = new_likilihood;
    }
    // ***** note ***********************************************************
    //
    // logP(o) gets larger at every update according to Baum-Welch algorhytm.
    // =>
    // once logP(o; new) - logP(o; old) is enogh small, logP(o) is converged.
    //
    // **********************************************************************

    RCLCPP_INFO(get_logger(), "finish update with %d loops", update_cnt);

    return;
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