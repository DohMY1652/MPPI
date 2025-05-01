#include "mppi_car/DatabaseConfig.h"

DatabaseConfig::DatabaseConfig(const YAML::Node& config)
     : config(config), distribution_parameters({0.00, 0.00}) {
    dt = config["system_parameters"]["dt"].as<double>();
    N = config["system_parameters"]["prediction_horizen"].as<int>();
    K = config["system_parameters"]["sampling_number"].as<int>(); 

    distribution_parameters[0] = config["distribution_parameters"]["mean"].as<double>();
    distribution_parameters[1] = config["distribution_parameters"]["variance"].as<double>();

    lambda = config["inverse_temeprature"].as<double>();

    L = config["car_parameters"]["length"].as<double>();

    simulation_frequency = config["simulation_parameters"]["frequency"].as<double>();

    for (const auto& data : config["car_initial_state"]) {
        car_initial_state.push_back(data.second.as<double>());
    }

    for (const auto& data : config["target_state"]) {
        goals.push_back(data.second.as<double>());
    }

    for (const auto& data : config["weights"]) {
        weights.push_back(data.second.as<double>());
    }
}

DatabaseConfig::~DatabaseConfig() {}

double DatabaseConfig::get_dt() const { return dt; }
int DatabaseConfig::get_N() const { return N; }
int DatabaseConfig::get_K() const { return K; }

double DatabaseConfig::get_lambda() const { return lambda; }

double DatabaseConfig::get_car_length() const {return L;}

double DatabaseConfig::get_simulation_frequency() const {return simulation_frequency;}

std::vector<double> DatabaseConfig::get_distribution_parameters() const { return distribution_parameters; }
std::vector<double> DatabaseConfig::get_car_initial_state() const { return car_initial_state; }
std::vector<double> DatabaseConfig::get_goals() const { return goals; }
std::vector<double> DatabaseConfig::get_weights() const { return weights; }

