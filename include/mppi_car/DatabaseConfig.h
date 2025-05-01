#ifndef DATABASECONFIG_H
#define DATABASECONFIG_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <vector>

class DatabaseConfig {
   public:
    DatabaseConfig(const YAML::Node& config);
    ~DatabaseConfig();

    double get_dt() const;
    int get_N() const;
    int get_K() const;

    double get_lambda() const;

    double get_car_length() const;
    double get_simulation_frequency() const;

    std::vector<double> get_distribution_parameters() const;
    std::vector<double> get_car_initial_state() const;
    std::vector<double> get_goals() const;
    std::vector<double> get_weights() const;

    

   private:
    YAML::Node config;
    
    double dt;
    int N;
    int K;

    double lambda;

    double L;

    std::vector<double> distribution_parameters;

    std::vector<double> car_initial_state;

    std::vector<double> goals;
    std::vector<double> weights;

    double simulation_frequency;

};

#endif  // DATABASECONFIG_H