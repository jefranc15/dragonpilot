#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6415261615196083022);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_585391152892609151);
void pose_H_mod_fun(double *state, double *out_6122232392299276739);
void pose_f_fun(double *state, double dt, double *out_3229721450137080995);
void pose_F_fun(double *state, double dt, double *out_2983843676786729027);
void pose_h_4(double *state, double *unused, double *out_1486608676067422377);
void pose_H_4(double *state, double *unused, double *out_4154347088581999177);
void pose_h_10(double *state, double *unused, double *out_8153320187445631634);
void pose_H_10(double *state, double *unused, double *out_6855067043592370112);
void pose_h_13(double *state, double *unused, double *out_6653821394755983052);
void pose_H_13(double *state, double *unused, double *out_942073263249666376);
void pose_h_14(double *state, double *unused, double *out_7681979440912549899);
void pose_H_14(double *state, double *unused, double *out_7237135520877371473);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}