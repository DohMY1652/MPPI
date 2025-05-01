#include "mppi_car/Dynamics.h"



Dynamics::Dynamics(std::shared_ptr<DatabaseConfig>& databaseconfig) 
    : databaseconfig(databaseconfig) {

  state  = databaseconfig->get_car_initial_state();
  dt = databaseconfig->get_dt();
  L = databaseconfig->get_car_length();
}

Dynamics::~Dynamics() {}


void Dynamics::update_state(std::vector<double> control) {
    double acceleration = control[0];
    double steering_angle = control[1];
    
    state[0] = state[0] + state[3] * cos(state[2]) * dt;
    state[1] = state[1] + state[3] * sin(state[2]) * dt;
    state[2] = state[2] + (state[3] / L) * tan(steering_angle) * dt;
    state[3] = state[3] + acceleration * dt;

}

std::vector<double> Dynamics::get_state() const {return state;}
double Dynamics::get_x() const {return state[0];}
double Dynamics::get_y() const {return state[1];}
double Dynamics::get_theta() const {return state[2];}
double Dynamics::get_v() const {return state[3];}


void Dynamics::set_state(std::vector<double> _state) {
    state = _state;
}