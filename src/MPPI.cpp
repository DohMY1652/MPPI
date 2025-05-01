#include "mppi_car/MPPI.h"




MPPI::MPPI(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig) 
    : nh(nh), databaseconfig(databaseconfig), previous_control({0.00, 0.00}),control({0.00, 0.00}) {
        std::random_device rd;
        generator = std::default_random_engine(rd());
    
        simulation_car = std::make_unique<Dynamics>(databaseconfig);
        
        dt = databaseconfig->get_dt();
        N = databaseconfig->get_N();
        K = databaseconfig->get_K();

        lambda = databaseconfig->get_lambda();

        distance = 100000.0;

        std::vector<double> dist_params = databaseconfig->get_distribution_parameters();
        distribution = std::normal_distribution<double>(dist_params[0], dist_params[1]);

        goal = databaseconfig->get_goals();
        weights = databaseconfig->get_weights();

        controls.resize(K);
        for (int k = 0; k < K; ++k) {
            controls[k].resize(N);
            for (int n = 0; n < N; ++n) {
                controls[k][n].resize(2, 0.0);
            }
        }

        states.resize(K);
        for (int k = 0; k < K; ++k) {
            states[k].resize(N);
            for (int n = 0; n < N; ++n) {
                states[k][n].resize(4, 0.0);
            }
        }

        costs.resize(K, 0.0);
        control_weights.resize(K, 0.0);

        
    }


MPPI::~MPPI() {}

void MPPI::calculate_control(std::vector<double> previous_state) {


    ///나중에 제거해도 됨////
    std::fill(costs.begin(), costs.end(), 0.0);
    std::fill(control_weights.begin(), control_weights.end(), 0.0);
    /////////////////////

    for (int k = 0; k< K; ++k) {
        simulation_car->set_state(previous_state);
        costs[k] = 0.0;
        for (int n = 0; n < N; ++n) {
            controls[k][n][0] = previous_control[0] + distribution(generator);
            controls[k][n][1] = previous_control[1] + distribution(generator);
            simulation_car->update_state(controls[k][n]);
            states[k][n] = simulation_car->get_state();
            costs[k] += calculate_cost(states[k][n], controls[k][n], n);
        }
        
    }

    calculate_control();

}

std::vector<double> MPPI::get_control() const {return control;}


double MPPI::calculate_cost(std::vector<double> state, std::vector<double> control, double i) {
    
    double goal_cost = weights[0] *(pow(goal[0] - state[0],2) + pow(goal[1] - state[1],2));
    double tracking_cost = 0;
    double transverse_distance = abs(state[0]-state[1])/sqrt(2);
    if (transverse_distance <= 0.1) {
        tracking_cost = weights[1] * transverse_distance;
    } else {
        tracking_cost = 100000;
    }
    double control_cost = weights[2] * (pow(control[0] ,2) + pow(control[1] ,2));
    // std::cout <<tracking_cost + control_cost << std::endl;

    return goal_cost + tracking_cost + control_cost;
    
}


void MPPI::calculate_control() {
    double min_cost = *std::min_element(costs.begin(), costs.end());
    double weight_sum = 0;
    control = {0.0, 0.0};

    for (int k = 0; k < K; ++k) {
        control_weights[k] = std::exp(-(costs[k] - min_cost) / lambda);
        weight_sum += control_weights[k];
    }

    for (int k = 0; k < K; ++k) {
        double w = control_weights[k] / weight_sum;
        control[0] += w * controls[k][0][0];
        control[1] += w * controls[k][0][1];
    }
    previous_control[0] = control[0];
    previous_control[1] = control[1];

}

bool MPPI::check_arrival(std::vector<double> state) {

    distance = sqrt(pow(goal[0] - state[0],2) + pow(goal[1] - state[1],2));
    if (distance <= goal[3]) {
        return true;
    }
    else {
        return false;
    }

}


std::vector<std::vector<std::vector<double>>> MPPI::get_states(int sampling_number) {
    std::vector<std::vector<std::vector<double>>> sampled_states;
    sampled_states.resize(sampling_number);
    for (int k = 0; k < sampling_number; ++k) {
        sampled_states[k].resize(N);
        for (int n = 0; n < N; ++n) {
            sampled_states[k][n].resize(2, 0.0);
        }
    }

    for (int i = 0; i < sampling_number; ++i) {
        for (int n = 0; n < N; ++n) {
            sampled_states[i][n][0]= states[i * (int)(K/sampling_number)][n][0];
            sampled_states[i][n][1]= states[i * (int)(K/sampling_number)][n][1];
        }
    }
    return sampled_states;
}