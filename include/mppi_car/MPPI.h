#ifndef MPPI_H
#define MPPI_H

#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <random>

#include "mppi_car/DatabaseConfig.h"
#include "mppi_car/Dynamics.h"


class MPPI {
    public:
        MPPI(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig);
        ~MPPI();  

        void calculate_control(std::vector<double> previous_state);

        bool check_arrival(std::vector<double> state);

        std::vector<double> get_control() const;

        std::vector<std::vector<std::vector<double>>> get_states(int sampling_number);
        

    private:
        ros::NodeHandle& nh;
        std::shared_ptr<DatabaseConfig>& databaseconfig;

        std::default_random_engine generator;
        std::normal_distribution<double> distribution;


        std::unique_ptr<Dynamics> simulation_car;

        double dt;
        int N;
        int K;

        double lambda;

        double distance;

        std::vector<double> previous_control;

        std::vector<std::vector<std::vector<double>>> controls;
        std::vector<std::vector<std::vector<double>>> states;
        std::vector<double> costs;
        std::vector<double> control_weights;

        std::vector<double> control;

        std::vector<double> goal;
        std::vector<double> weights;

        double calculate_cost(std::vector<double> state, std::vector<double> control, double i);
        void calculate_control();

};

#endif //MPPI_H