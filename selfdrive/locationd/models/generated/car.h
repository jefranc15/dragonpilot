#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6191759550078159819);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4095681203358372484);
void car_H_mod_fun(double *state, double *out_6524855641257922771);
void car_f_fun(double *state, double dt, double *out_42607301673440609);
void car_F_fun(double *state, double dt, double *out_8173937638867878329);
void car_h_25(double *state, double *unused, double *out_4335301968621380522);
void car_H_25(double *state, double *unused, double *out_8827870159659232337);
void car_h_24(double *state, double *unused, double *out_4777486045985710368);
void car_H_24(double *state, double *unused, double *out_400195026409962888);
void car_h_30(double *state, double *unused, double *out_4793503330779942184);
void car_H_30(double *state, double *unused, double *out_5091177583922711081);
void car_h_26(double *state, double *unused, double *out_5396819215456419701);
void car_H_26(double *state, double *unused, double *out_5877370595176263055);
void car_h_27(double *state, double *unused, double *out_5278811821523042957);
void car_H_27(double *state, double *unused, double *out_7314771655106654298);
void car_h_29(double *state, double *unused, double *out_4528054775467966775);
void car_H_29(double *state, double *unused, double *out_5601408928237103265);
void car_h_28(double *state, double *unused, double *out_277138553862197312);
void car_H_28(double *state, double *unused, double *out_7565039199802429516);
void car_h_31(double *state, double *unused, double *out_4178115300270251377);
void car_H_31(double *state, double *unused, double *out_5251162492942911579);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}