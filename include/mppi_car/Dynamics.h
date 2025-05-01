#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "mppi_car/DatabaseConfig.h"

class Dynamics {
    public:
        Dynamics(std::shared_ptr<DatabaseConfig>& databaseconfig);
        ~Dynamics();

        void update_state(std::vector<double> control);
        void set_state(std::vector<double> _state);

        std::vector<double> get_state() const;
        double get_x() const;
        double get_y() const;
        double get_theta() const;
        double get_v() const;
        
    private:
        std::shared_ptr<DatabaseConfig>& databaseconfig;

        double L;
        double dt;


        std::vector<double> state;
};

#endif //DYNAMICS_H