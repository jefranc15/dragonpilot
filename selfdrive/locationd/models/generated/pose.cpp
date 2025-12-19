#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6415261615196083022) {
   out_6415261615196083022[0] = delta_x[0] + nom_x[0];
   out_6415261615196083022[1] = delta_x[1] + nom_x[1];
   out_6415261615196083022[2] = delta_x[2] + nom_x[2];
   out_6415261615196083022[3] = delta_x[3] + nom_x[3];
   out_6415261615196083022[4] = delta_x[4] + nom_x[4];
   out_6415261615196083022[5] = delta_x[5] + nom_x[5];
   out_6415261615196083022[6] = delta_x[6] + nom_x[6];
   out_6415261615196083022[7] = delta_x[7] + nom_x[7];
   out_6415261615196083022[8] = delta_x[8] + nom_x[8];
   out_6415261615196083022[9] = delta_x[9] + nom_x[9];
   out_6415261615196083022[10] = delta_x[10] + nom_x[10];
   out_6415261615196083022[11] = delta_x[11] + nom_x[11];
   out_6415261615196083022[12] = delta_x[12] + nom_x[12];
   out_6415261615196083022[13] = delta_x[13] + nom_x[13];
   out_6415261615196083022[14] = delta_x[14] + nom_x[14];
   out_6415261615196083022[15] = delta_x[15] + nom_x[15];
   out_6415261615196083022[16] = delta_x[16] + nom_x[16];
   out_6415261615196083022[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_585391152892609151) {
   out_585391152892609151[0] = -nom_x[0] + true_x[0];
   out_585391152892609151[1] = -nom_x[1] + true_x[1];
   out_585391152892609151[2] = -nom_x[2] + true_x[2];
   out_585391152892609151[3] = -nom_x[3] + true_x[3];
   out_585391152892609151[4] = -nom_x[4] + true_x[4];
   out_585391152892609151[5] = -nom_x[5] + true_x[5];
   out_585391152892609151[6] = -nom_x[6] + true_x[6];
   out_585391152892609151[7] = -nom_x[7] + true_x[7];
   out_585391152892609151[8] = -nom_x[8] + true_x[8];
   out_585391152892609151[9] = -nom_x[9] + true_x[9];
   out_585391152892609151[10] = -nom_x[10] + true_x[10];
   out_585391152892609151[11] = -nom_x[11] + true_x[11];
   out_585391152892609151[12] = -nom_x[12] + true_x[12];
   out_585391152892609151[13] = -nom_x[13] + true_x[13];
   out_585391152892609151[14] = -nom_x[14] + true_x[14];
   out_585391152892609151[15] = -nom_x[15] + true_x[15];
   out_585391152892609151[16] = -nom_x[16] + true_x[16];
   out_585391152892609151[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6122232392299276739) {
   out_6122232392299276739[0] = 1.0;
   out_6122232392299276739[1] = 0.0;
   out_6122232392299276739[2] = 0.0;
   out_6122232392299276739[3] = 0.0;
   out_6122232392299276739[4] = 0.0;
   out_6122232392299276739[5] = 0.0;
   out_6122232392299276739[6] = 0.0;
   out_6122232392299276739[7] = 0.0;
   out_6122232392299276739[8] = 0.0;
   out_6122232392299276739[9] = 0.0;
   out_6122232392299276739[10] = 0.0;
   out_6122232392299276739[11] = 0.0;
   out_6122232392299276739[12] = 0.0;
   out_6122232392299276739[13] = 0.0;
   out_6122232392299276739[14] = 0.0;
   out_6122232392299276739[15] = 0.0;
   out_6122232392299276739[16] = 0.0;
   out_6122232392299276739[17] = 0.0;
   out_6122232392299276739[18] = 0.0;
   out_6122232392299276739[19] = 1.0;
   out_6122232392299276739[20] = 0.0;
   out_6122232392299276739[21] = 0.0;
   out_6122232392299276739[22] = 0.0;
   out_6122232392299276739[23] = 0.0;
   out_6122232392299276739[24] = 0.0;
   out_6122232392299276739[25] = 0.0;
   out_6122232392299276739[26] = 0.0;
   out_6122232392299276739[27] = 0.0;
   out_6122232392299276739[28] = 0.0;
   out_6122232392299276739[29] = 0.0;
   out_6122232392299276739[30] = 0.0;
   out_6122232392299276739[31] = 0.0;
   out_6122232392299276739[32] = 0.0;
   out_6122232392299276739[33] = 0.0;
   out_6122232392299276739[34] = 0.0;
   out_6122232392299276739[35] = 0.0;
   out_6122232392299276739[36] = 0.0;
   out_6122232392299276739[37] = 0.0;
   out_6122232392299276739[38] = 1.0;
   out_6122232392299276739[39] = 0.0;
   out_6122232392299276739[40] = 0.0;
   out_6122232392299276739[41] = 0.0;
   out_6122232392299276739[42] = 0.0;
   out_6122232392299276739[43] = 0.0;
   out_6122232392299276739[44] = 0.0;
   out_6122232392299276739[45] = 0.0;
   out_6122232392299276739[46] = 0.0;
   out_6122232392299276739[47] = 0.0;
   out_6122232392299276739[48] = 0.0;
   out_6122232392299276739[49] = 0.0;
   out_6122232392299276739[50] = 0.0;
   out_6122232392299276739[51] = 0.0;
   out_6122232392299276739[52] = 0.0;
   out_6122232392299276739[53] = 0.0;
   out_6122232392299276739[54] = 0.0;
   out_6122232392299276739[55] = 0.0;
   out_6122232392299276739[56] = 0.0;
   out_6122232392299276739[57] = 1.0;
   out_6122232392299276739[58] = 0.0;
   out_6122232392299276739[59] = 0.0;
   out_6122232392299276739[60] = 0.0;
   out_6122232392299276739[61] = 0.0;
   out_6122232392299276739[62] = 0.0;
   out_6122232392299276739[63] = 0.0;
   out_6122232392299276739[64] = 0.0;
   out_6122232392299276739[65] = 0.0;
   out_6122232392299276739[66] = 0.0;
   out_6122232392299276739[67] = 0.0;
   out_6122232392299276739[68] = 0.0;
   out_6122232392299276739[69] = 0.0;
   out_6122232392299276739[70] = 0.0;
   out_6122232392299276739[71] = 0.0;
   out_6122232392299276739[72] = 0.0;
   out_6122232392299276739[73] = 0.0;
   out_6122232392299276739[74] = 0.0;
   out_6122232392299276739[75] = 0.0;
   out_6122232392299276739[76] = 1.0;
   out_6122232392299276739[77] = 0.0;
   out_6122232392299276739[78] = 0.0;
   out_6122232392299276739[79] = 0.0;
   out_6122232392299276739[80] = 0.0;
   out_6122232392299276739[81] = 0.0;
   out_6122232392299276739[82] = 0.0;
   out_6122232392299276739[83] = 0.0;
   out_6122232392299276739[84] = 0.0;
   out_6122232392299276739[85] = 0.0;
   out_6122232392299276739[86] = 0.0;
   out_6122232392299276739[87] = 0.0;
   out_6122232392299276739[88] = 0.0;
   out_6122232392299276739[89] = 0.0;
   out_6122232392299276739[90] = 0.0;
   out_6122232392299276739[91] = 0.0;
   out_6122232392299276739[92] = 0.0;
   out_6122232392299276739[93] = 0.0;
   out_6122232392299276739[94] = 0.0;
   out_6122232392299276739[95] = 1.0;
   out_6122232392299276739[96] = 0.0;
   out_6122232392299276739[97] = 0.0;
   out_6122232392299276739[98] = 0.0;
   out_6122232392299276739[99] = 0.0;
   out_6122232392299276739[100] = 0.0;
   out_6122232392299276739[101] = 0.0;
   out_6122232392299276739[102] = 0.0;
   out_6122232392299276739[103] = 0.0;
   out_6122232392299276739[104] = 0.0;
   out_6122232392299276739[105] = 0.0;
   out_6122232392299276739[106] = 0.0;
   out_6122232392299276739[107] = 0.0;
   out_6122232392299276739[108] = 0.0;
   out_6122232392299276739[109] = 0.0;
   out_6122232392299276739[110] = 0.0;
   out_6122232392299276739[111] = 0.0;
   out_6122232392299276739[112] = 0.0;
   out_6122232392299276739[113] = 0.0;
   out_6122232392299276739[114] = 1.0;
   out_6122232392299276739[115] = 0.0;
   out_6122232392299276739[116] = 0.0;
   out_6122232392299276739[117] = 0.0;
   out_6122232392299276739[118] = 0.0;
   out_6122232392299276739[119] = 0.0;
   out_6122232392299276739[120] = 0.0;
   out_6122232392299276739[121] = 0.0;
   out_6122232392299276739[122] = 0.0;
   out_6122232392299276739[123] = 0.0;
   out_6122232392299276739[124] = 0.0;
   out_6122232392299276739[125] = 0.0;
   out_6122232392299276739[126] = 0.0;
   out_6122232392299276739[127] = 0.0;
   out_6122232392299276739[128] = 0.0;
   out_6122232392299276739[129] = 0.0;
   out_6122232392299276739[130] = 0.0;
   out_6122232392299276739[131] = 0.0;
   out_6122232392299276739[132] = 0.0;
   out_6122232392299276739[133] = 1.0;
   out_6122232392299276739[134] = 0.0;
   out_6122232392299276739[135] = 0.0;
   out_6122232392299276739[136] = 0.0;
   out_6122232392299276739[137] = 0.0;
   out_6122232392299276739[138] = 0.0;
   out_6122232392299276739[139] = 0.0;
   out_6122232392299276739[140] = 0.0;
   out_6122232392299276739[141] = 0.0;
   out_6122232392299276739[142] = 0.0;
   out_6122232392299276739[143] = 0.0;
   out_6122232392299276739[144] = 0.0;
   out_6122232392299276739[145] = 0.0;
   out_6122232392299276739[146] = 0.0;
   out_6122232392299276739[147] = 0.0;
   out_6122232392299276739[148] = 0.0;
   out_6122232392299276739[149] = 0.0;
   out_6122232392299276739[150] = 0.0;
   out_6122232392299276739[151] = 0.0;
   out_6122232392299276739[152] = 1.0;
   out_6122232392299276739[153] = 0.0;
   out_6122232392299276739[154] = 0.0;
   out_6122232392299276739[155] = 0.0;
   out_6122232392299276739[156] = 0.0;
   out_6122232392299276739[157] = 0.0;
   out_6122232392299276739[158] = 0.0;
   out_6122232392299276739[159] = 0.0;
   out_6122232392299276739[160] = 0.0;
   out_6122232392299276739[161] = 0.0;
   out_6122232392299276739[162] = 0.0;
   out_6122232392299276739[163] = 0.0;
   out_6122232392299276739[164] = 0.0;
   out_6122232392299276739[165] = 0.0;
   out_6122232392299276739[166] = 0.0;
   out_6122232392299276739[167] = 0.0;
   out_6122232392299276739[168] = 0.0;
   out_6122232392299276739[169] = 0.0;
   out_6122232392299276739[170] = 0.0;
   out_6122232392299276739[171] = 1.0;
   out_6122232392299276739[172] = 0.0;
   out_6122232392299276739[173] = 0.0;
   out_6122232392299276739[174] = 0.0;
   out_6122232392299276739[175] = 0.0;
   out_6122232392299276739[176] = 0.0;
   out_6122232392299276739[177] = 0.0;
   out_6122232392299276739[178] = 0.0;
   out_6122232392299276739[179] = 0.0;
   out_6122232392299276739[180] = 0.0;
   out_6122232392299276739[181] = 0.0;
   out_6122232392299276739[182] = 0.0;
   out_6122232392299276739[183] = 0.0;
   out_6122232392299276739[184] = 0.0;
   out_6122232392299276739[185] = 0.0;
   out_6122232392299276739[186] = 0.0;
   out_6122232392299276739[187] = 0.0;
   out_6122232392299276739[188] = 0.0;
   out_6122232392299276739[189] = 0.0;
   out_6122232392299276739[190] = 1.0;
   out_6122232392299276739[191] = 0.0;
   out_6122232392299276739[192] = 0.0;
   out_6122232392299276739[193] = 0.0;
   out_6122232392299276739[194] = 0.0;
   out_6122232392299276739[195] = 0.0;
   out_6122232392299276739[196] = 0.0;
   out_6122232392299276739[197] = 0.0;
   out_6122232392299276739[198] = 0.0;
   out_6122232392299276739[199] = 0.0;
   out_6122232392299276739[200] = 0.0;
   out_6122232392299276739[201] = 0.0;
   out_6122232392299276739[202] = 0.0;
   out_6122232392299276739[203] = 0.0;
   out_6122232392299276739[204] = 0.0;
   out_6122232392299276739[205] = 0.0;
   out_6122232392299276739[206] = 0.0;
   out_6122232392299276739[207] = 0.0;
   out_6122232392299276739[208] = 0.0;
   out_6122232392299276739[209] = 1.0;
   out_6122232392299276739[210] = 0.0;
   out_6122232392299276739[211] = 0.0;
   out_6122232392299276739[212] = 0.0;
   out_6122232392299276739[213] = 0.0;
   out_6122232392299276739[214] = 0.0;
   out_6122232392299276739[215] = 0.0;
   out_6122232392299276739[216] = 0.0;
   out_6122232392299276739[217] = 0.0;
   out_6122232392299276739[218] = 0.0;
   out_6122232392299276739[219] = 0.0;
   out_6122232392299276739[220] = 0.0;
   out_6122232392299276739[221] = 0.0;
   out_6122232392299276739[222] = 0.0;
   out_6122232392299276739[223] = 0.0;
   out_6122232392299276739[224] = 0.0;
   out_6122232392299276739[225] = 0.0;
   out_6122232392299276739[226] = 0.0;
   out_6122232392299276739[227] = 0.0;
   out_6122232392299276739[228] = 1.0;
   out_6122232392299276739[229] = 0.0;
   out_6122232392299276739[230] = 0.0;
   out_6122232392299276739[231] = 0.0;
   out_6122232392299276739[232] = 0.0;
   out_6122232392299276739[233] = 0.0;
   out_6122232392299276739[234] = 0.0;
   out_6122232392299276739[235] = 0.0;
   out_6122232392299276739[236] = 0.0;
   out_6122232392299276739[237] = 0.0;
   out_6122232392299276739[238] = 0.0;
   out_6122232392299276739[239] = 0.0;
   out_6122232392299276739[240] = 0.0;
   out_6122232392299276739[241] = 0.0;
   out_6122232392299276739[242] = 0.0;
   out_6122232392299276739[243] = 0.0;
   out_6122232392299276739[244] = 0.0;
   out_6122232392299276739[245] = 0.0;
   out_6122232392299276739[246] = 0.0;
   out_6122232392299276739[247] = 1.0;
   out_6122232392299276739[248] = 0.0;
   out_6122232392299276739[249] = 0.0;
   out_6122232392299276739[250] = 0.0;
   out_6122232392299276739[251] = 0.0;
   out_6122232392299276739[252] = 0.0;
   out_6122232392299276739[253] = 0.0;
   out_6122232392299276739[254] = 0.0;
   out_6122232392299276739[255] = 0.0;
   out_6122232392299276739[256] = 0.0;
   out_6122232392299276739[257] = 0.0;
   out_6122232392299276739[258] = 0.0;
   out_6122232392299276739[259] = 0.0;
   out_6122232392299276739[260] = 0.0;
   out_6122232392299276739[261] = 0.0;
   out_6122232392299276739[262] = 0.0;
   out_6122232392299276739[263] = 0.0;
   out_6122232392299276739[264] = 0.0;
   out_6122232392299276739[265] = 0.0;
   out_6122232392299276739[266] = 1.0;
   out_6122232392299276739[267] = 0.0;
   out_6122232392299276739[268] = 0.0;
   out_6122232392299276739[269] = 0.0;
   out_6122232392299276739[270] = 0.0;
   out_6122232392299276739[271] = 0.0;
   out_6122232392299276739[272] = 0.0;
   out_6122232392299276739[273] = 0.0;
   out_6122232392299276739[274] = 0.0;
   out_6122232392299276739[275] = 0.0;
   out_6122232392299276739[276] = 0.0;
   out_6122232392299276739[277] = 0.0;
   out_6122232392299276739[278] = 0.0;
   out_6122232392299276739[279] = 0.0;
   out_6122232392299276739[280] = 0.0;
   out_6122232392299276739[281] = 0.0;
   out_6122232392299276739[282] = 0.0;
   out_6122232392299276739[283] = 0.0;
   out_6122232392299276739[284] = 0.0;
   out_6122232392299276739[285] = 1.0;
   out_6122232392299276739[286] = 0.0;
   out_6122232392299276739[287] = 0.0;
   out_6122232392299276739[288] = 0.0;
   out_6122232392299276739[289] = 0.0;
   out_6122232392299276739[290] = 0.0;
   out_6122232392299276739[291] = 0.0;
   out_6122232392299276739[292] = 0.0;
   out_6122232392299276739[293] = 0.0;
   out_6122232392299276739[294] = 0.0;
   out_6122232392299276739[295] = 0.0;
   out_6122232392299276739[296] = 0.0;
   out_6122232392299276739[297] = 0.0;
   out_6122232392299276739[298] = 0.0;
   out_6122232392299276739[299] = 0.0;
   out_6122232392299276739[300] = 0.0;
   out_6122232392299276739[301] = 0.0;
   out_6122232392299276739[302] = 0.0;
   out_6122232392299276739[303] = 0.0;
   out_6122232392299276739[304] = 1.0;
   out_6122232392299276739[305] = 0.0;
   out_6122232392299276739[306] = 0.0;
   out_6122232392299276739[307] = 0.0;
   out_6122232392299276739[308] = 0.0;
   out_6122232392299276739[309] = 0.0;
   out_6122232392299276739[310] = 0.0;
   out_6122232392299276739[311] = 0.0;
   out_6122232392299276739[312] = 0.0;
   out_6122232392299276739[313] = 0.0;
   out_6122232392299276739[314] = 0.0;
   out_6122232392299276739[315] = 0.0;
   out_6122232392299276739[316] = 0.0;
   out_6122232392299276739[317] = 0.0;
   out_6122232392299276739[318] = 0.0;
   out_6122232392299276739[319] = 0.0;
   out_6122232392299276739[320] = 0.0;
   out_6122232392299276739[321] = 0.0;
   out_6122232392299276739[322] = 0.0;
   out_6122232392299276739[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3229721450137080995) {
   out_3229721450137080995[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3229721450137080995[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3229721450137080995[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3229721450137080995[3] = dt*state[12] + state[3];
   out_3229721450137080995[4] = dt*state[13] + state[4];
   out_3229721450137080995[5] = dt*state[14] + state[5];
   out_3229721450137080995[6] = state[6];
   out_3229721450137080995[7] = state[7];
   out_3229721450137080995[8] = state[8];
   out_3229721450137080995[9] = state[9];
   out_3229721450137080995[10] = state[10];
   out_3229721450137080995[11] = state[11];
   out_3229721450137080995[12] = state[12];
   out_3229721450137080995[13] = state[13];
   out_3229721450137080995[14] = state[14];
   out_3229721450137080995[15] = state[15];
   out_3229721450137080995[16] = state[16];
   out_3229721450137080995[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2983843676786729027) {
   out_2983843676786729027[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2983843676786729027[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2983843676786729027[2] = 0;
   out_2983843676786729027[3] = 0;
   out_2983843676786729027[4] = 0;
   out_2983843676786729027[5] = 0;
   out_2983843676786729027[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2983843676786729027[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2983843676786729027[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2983843676786729027[9] = 0;
   out_2983843676786729027[10] = 0;
   out_2983843676786729027[11] = 0;
   out_2983843676786729027[12] = 0;
   out_2983843676786729027[13] = 0;
   out_2983843676786729027[14] = 0;
   out_2983843676786729027[15] = 0;
   out_2983843676786729027[16] = 0;
   out_2983843676786729027[17] = 0;
   out_2983843676786729027[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2983843676786729027[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2983843676786729027[20] = 0;
   out_2983843676786729027[21] = 0;
   out_2983843676786729027[22] = 0;
   out_2983843676786729027[23] = 0;
   out_2983843676786729027[24] = 0;
   out_2983843676786729027[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2983843676786729027[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2983843676786729027[27] = 0;
   out_2983843676786729027[28] = 0;
   out_2983843676786729027[29] = 0;
   out_2983843676786729027[30] = 0;
   out_2983843676786729027[31] = 0;
   out_2983843676786729027[32] = 0;
   out_2983843676786729027[33] = 0;
   out_2983843676786729027[34] = 0;
   out_2983843676786729027[35] = 0;
   out_2983843676786729027[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2983843676786729027[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2983843676786729027[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2983843676786729027[39] = 0;
   out_2983843676786729027[40] = 0;
   out_2983843676786729027[41] = 0;
   out_2983843676786729027[42] = 0;
   out_2983843676786729027[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2983843676786729027[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2983843676786729027[45] = 0;
   out_2983843676786729027[46] = 0;
   out_2983843676786729027[47] = 0;
   out_2983843676786729027[48] = 0;
   out_2983843676786729027[49] = 0;
   out_2983843676786729027[50] = 0;
   out_2983843676786729027[51] = 0;
   out_2983843676786729027[52] = 0;
   out_2983843676786729027[53] = 0;
   out_2983843676786729027[54] = 0;
   out_2983843676786729027[55] = 0;
   out_2983843676786729027[56] = 0;
   out_2983843676786729027[57] = 1;
   out_2983843676786729027[58] = 0;
   out_2983843676786729027[59] = 0;
   out_2983843676786729027[60] = 0;
   out_2983843676786729027[61] = 0;
   out_2983843676786729027[62] = 0;
   out_2983843676786729027[63] = 0;
   out_2983843676786729027[64] = 0;
   out_2983843676786729027[65] = 0;
   out_2983843676786729027[66] = dt;
   out_2983843676786729027[67] = 0;
   out_2983843676786729027[68] = 0;
   out_2983843676786729027[69] = 0;
   out_2983843676786729027[70] = 0;
   out_2983843676786729027[71] = 0;
   out_2983843676786729027[72] = 0;
   out_2983843676786729027[73] = 0;
   out_2983843676786729027[74] = 0;
   out_2983843676786729027[75] = 0;
   out_2983843676786729027[76] = 1;
   out_2983843676786729027[77] = 0;
   out_2983843676786729027[78] = 0;
   out_2983843676786729027[79] = 0;
   out_2983843676786729027[80] = 0;
   out_2983843676786729027[81] = 0;
   out_2983843676786729027[82] = 0;
   out_2983843676786729027[83] = 0;
   out_2983843676786729027[84] = 0;
   out_2983843676786729027[85] = dt;
   out_2983843676786729027[86] = 0;
   out_2983843676786729027[87] = 0;
   out_2983843676786729027[88] = 0;
   out_2983843676786729027[89] = 0;
   out_2983843676786729027[90] = 0;
   out_2983843676786729027[91] = 0;
   out_2983843676786729027[92] = 0;
   out_2983843676786729027[93] = 0;
   out_2983843676786729027[94] = 0;
   out_2983843676786729027[95] = 1;
   out_2983843676786729027[96] = 0;
   out_2983843676786729027[97] = 0;
   out_2983843676786729027[98] = 0;
   out_2983843676786729027[99] = 0;
   out_2983843676786729027[100] = 0;
   out_2983843676786729027[101] = 0;
   out_2983843676786729027[102] = 0;
   out_2983843676786729027[103] = 0;
   out_2983843676786729027[104] = dt;
   out_2983843676786729027[105] = 0;
   out_2983843676786729027[106] = 0;
   out_2983843676786729027[107] = 0;
   out_2983843676786729027[108] = 0;
   out_2983843676786729027[109] = 0;
   out_2983843676786729027[110] = 0;
   out_2983843676786729027[111] = 0;
   out_2983843676786729027[112] = 0;
   out_2983843676786729027[113] = 0;
   out_2983843676786729027[114] = 1;
   out_2983843676786729027[115] = 0;
   out_2983843676786729027[116] = 0;
   out_2983843676786729027[117] = 0;
   out_2983843676786729027[118] = 0;
   out_2983843676786729027[119] = 0;
   out_2983843676786729027[120] = 0;
   out_2983843676786729027[121] = 0;
   out_2983843676786729027[122] = 0;
   out_2983843676786729027[123] = 0;
   out_2983843676786729027[124] = 0;
   out_2983843676786729027[125] = 0;
   out_2983843676786729027[126] = 0;
   out_2983843676786729027[127] = 0;
   out_2983843676786729027[128] = 0;
   out_2983843676786729027[129] = 0;
   out_2983843676786729027[130] = 0;
   out_2983843676786729027[131] = 0;
   out_2983843676786729027[132] = 0;
   out_2983843676786729027[133] = 1;
   out_2983843676786729027[134] = 0;
   out_2983843676786729027[135] = 0;
   out_2983843676786729027[136] = 0;
   out_2983843676786729027[137] = 0;
   out_2983843676786729027[138] = 0;
   out_2983843676786729027[139] = 0;
   out_2983843676786729027[140] = 0;
   out_2983843676786729027[141] = 0;
   out_2983843676786729027[142] = 0;
   out_2983843676786729027[143] = 0;
   out_2983843676786729027[144] = 0;
   out_2983843676786729027[145] = 0;
   out_2983843676786729027[146] = 0;
   out_2983843676786729027[147] = 0;
   out_2983843676786729027[148] = 0;
   out_2983843676786729027[149] = 0;
   out_2983843676786729027[150] = 0;
   out_2983843676786729027[151] = 0;
   out_2983843676786729027[152] = 1;
   out_2983843676786729027[153] = 0;
   out_2983843676786729027[154] = 0;
   out_2983843676786729027[155] = 0;
   out_2983843676786729027[156] = 0;
   out_2983843676786729027[157] = 0;
   out_2983843676786729027[158] = 0;
   out_2983843676786729027[159] = 0;
   out_2983843676786729027[160] = 0;
   out_2983843676786729027[161] = 0;
   out_2983843676786729027[162] = 0;
   out_2983843676786729027[163] = 0;
   out_2983843676786729027[164] = 0;
   out_2983843676786729027[165] = 0;
   out_2983843676786729027[166] = 0;
   out_2983843676786729027[167] = 0;
   out_2983843676786729027[168] = 0;
   out_2983843676786729027[169] = 0;
   out_2983843676786729027[170] = 0;
   out_2983843676786729027[171] = 1;
   out_2983843676786729027[172] = 0;
   out_2983843676786729027[173] = 0;
   out_2983843676786729027[174] = 0;
   out_2983843676786729027[175] = 0;
   out_2983843676786729027[176] = 0;
   out_2983843676786729027[177] = 0;
   out_2983843676786729027[178] = 0;
   out_2983843676786729027[179] = 0;
   out_2983843676786729027[180] = 0;
   out_2983843676786729027[181] = 0;
   out_2983843676786729027[182] = 0;
   out_2983843676786729027[183] = 0;
   out_2983843676786729027[184] = 0;
   out_2983843676786729027[185] = 0;
   out_2983843676786729027[186] = 0;
   out_2983843676786729027[187] = 0;
   out_2983843676786729027[188] = 0;
   out_2983843676786729027[189] = 0;
   out_2983843676786729027[190] = 1;
   out_2983843676786729027[191] = 0;
   out_2983843676786729027[192] = 0;
   out_2983843676786729027[193] = 0;
   out_2983843676786729027[194] = 0;
   out_2983843676786729027[195] = 0;
   out_2983843676786729027[196] = 0;
   out_2983843676786729027[197] = 0;
   out_2983843676786729027[198] = 0;
   out_2983843676786729027[199] = 0;
   out_2983843676786729027[200] = 0;
   out_2983843676786729027[201] = 0;
   out_2983843676786729027[202] = 0;
   out_2983843676786729027[203] = 0;
   out_2983843676786729027[204] = 0;
   out_2983843676786729027[205] = 0;
   out_2983843676786729027[206] = 0;
   out_2983843676786729027[207] = 0;
   out_2983843676786729027[208] = 0;
   out_2983843676786729027[209] = 1;
   out_2983843676786729027[210] = 0;
   out_2983843676786729027[211] = 0;
   out_2983843676786729027[212] = 0;
   out_2983843676786729027[213] = 0;
   out_2983843676786729027[214] = 0;
   out_2983843676786729027[215] = 0;
   out_2983843676786729027[216] = 0;
   out_2983843676786729027[217] = 0;
   out_2983843676786729027[218] = 0;
   out_2983843676786729027[219] = 0;
   out_2983843676786729027[220] = 0;
   out_2983843676786729027[221] = 0;
   out_2983843676786729027[222] = 0;
   out_2983843676786729027[223] = 0;
   out_2983843676786729027[224] = 0;
   out_2983843676786729027[225] = 0;
   out_2983843676786729027[226] = 0;
   out_2983843676786729027[227] = 0;
   out_2983843676786729027[228] = 1;
   out_2983843676786729027[229] = 0;
   out_2983843676786729027[230] = 0;
   out_2983843676786729027[231] = 0;
   out_2983843676786729027[232] = 0;
   out_2983843676786729027[233] = 0;
   out_2983843676786729027[234] = 0;
   out_2983843676786729027[235] = 0;
   out_2983843676786729027[236] = 0;
   out_2983843676786729027[237] = 0;
   out_2983843676786729027[238] = 0;
   out_2983843676786729027[239] = 0;
   out_2983843676786729027[240] = 0;
   out_2983843676786729027[241] = 0;
   out_2983843676786729027[242] = 0;
   out_2983843676786729027[243] = 0;
   out_2983843676786729027[244] = 0;
   out_2983843676786729027[245] = 0;
   out_2983843676786729027[246] = 0;
   out_2983843676786729027[247] = 1;
   out_2983843676786729027[248] = 0;
   out_2983843676786729027[249] = 0;
   out_2983843676786729027[250] = 0;
   out_2983843676786729027[251] = 0;
   out_2983843676786729027[252] = 0;
   out_2983843676786729027[253] = 0;
   out_2983843676786729027[254] = 0;
   out_2983843676786729027[255] = 0;
   out_2983843676786729027[256] = 0;
   out_2983843676786729027[257] = 0;
   out_2983843676786729027[258] = 0;
   out_2983843676786729027[259] = 0;
   out_2983843676786729027[260] = 0;
   out_2983843676786729027[261] = 0;
   out_2983843676786729027[262] = 0;
   out_2983843676786729027[263] = 0;
   out_2983843676786729027[264] = 0;
   out_2983843676786729027[265] = 0;
   out_2983843676786729027[266] = 1;
   out_2983843676786729027[267] = 0;
   out_2983843676786729027[268] = 0;
   out_2983843676786729027[269] = 0;
   out_2983843676786729027[270] = 0;
   out_2983843676786729027[271] = 0;
   out_2983843676786729027[272] = 0;
   out_2983843676786729027[273] = 0;
   out_2983843676786729027[274] = 0;
   out_2983843676786729027[275] = 0;
   out_2983843676786729027[276] = 0;
   out_2983843676786729027[277] = 0;
   out_2983843676786729027[278] = 0;
   out_2983843676786729027[279] = 0;
   out_2983843676786729027[280] = 0;
   out_2983843676786729027[281] = 0;
   out_2983843676786729027[282] = 0;
   out_2983843676786729027[283] = 0;
   out_2983843676786729027[284] = 0;
   out_2983843676786729027[285] = 1;
   out_2983843676786729027[286] = 0;
   out_2983843676786729027[287] = 0;
   out_2983843676786729027[288] = 0;
   out_2983843676786729027[289] = 0;
   out_2983843676786729027[290] = 0;
   out_2983843676786729027[291] = 0;
   out_2983843676786729027[292] = 0;
   out_2983843676786729027[293] = 0;
   out_2983843676786729027[294] = 0;
   out_2983843676786729027[295] = 0;
   out_2983843676786729027[296] = 0;
   out_2983843676786729027[297] = 0;
   out_2983843676786729027[298] = 0;
   out_2983843676786729027[299] = 0;
   out_2983843676786729027[300] = 0;
   out_2983843676786729027[301] = 0;
   out_2983843676786729027[302] = 0;
   out_2983843676786729027[303] = 0;
   out_2983843676786729027[304] = 1;
   out_2983843676786729027[305] = 0;
   out_2983843676786729027[306] = 0;
   out_2983843676786729027[307] = 0;
   out_2983843676786729027[308] = 0;
   out_2983843676786729027[309] = 0;
   out_2983843676786729027[310] = 0;
   out_2983843676786729027[311] = 0;
   out_2983843676786729027[312] = 0;
   out_2983843676786729027[313] = 0;
   out_2983843676786729027[314] = 0;
   out_2983843676786729027[315] = 0;
   out_2983843676786729027[316] = 0;
   out_2983843676786729027[317] = 0;
   out_2983843676786729027[318] = 0;
   out_2983843676786729027[319] = 0;
   out_2983843676786729027[320] = 0;
   out_2983843676786729027[321] = 0;
   out_2983843676786729027[322] = 0;
   out_2983843676786729027[323] = 1;
}
void h_4(double *state, double *unused, double *out_1486608676067422377) {
   out_1486608676067422377[0] = state[6] + state[9];
   out_1486608676067422377[1] = state[7] + state[10];
   out_1486608676067422377[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4154347088581999177) {
   out_4154347088581999177[0] = 0;
   out_4154347088581999177[1] = 0;
   out_4154347088581999177[2] = 0;
   out_4154347088581999177[3] = 0;
   out_4154347088581999177[4] = 0;
   out_4154347088581999177[5] = 0;
   out_4154347088581999177[6] = 1;
   out_4154347088581999177[7] = 0;
   out_4154347088581999177[8] = 0;
   out_4154347088581999177[9] = 1;
   out_4154347088581999177[10] = 0;
   out_4154347088581999177[11] = 0;
   out_4154347088581999177[12] = 0;
   out_4154347088581999177[13] = 0;
   out_4154347088581999177[14] = 0;
   out_4154347088581999177[15] = 0;
   out_4154347088581999177[16] = 0;
   out_4154347088581999177[17] = 0;
   out_4154347088581999177[18] = 0;
   out_4154347088581999177[19] = 0;
   out_4154347088581999177[20] = 0;
   out_4154347088581999177[21] = 0;
   out_4154347088581999177[22] = 0;
   out_4154347088581999177[23] = 0;
   out_4154347088581999177[24] = 0;
   out_4154347088581999177[25] = 1;
   out_4154347088581999177[26] = 0;
   out_4154347088581999177[27] = 0;
   out_4154347088581999177[28] = 1;
   out_4154347088581999177[29] = 0;
   out_4154347088581999177[30] = 0;
   out_4154347088581999177[31] = 0;
   out_4154347088581999177[32] = 0;
   out_4154347088581999177[33] = 0;
   out_4154347088581999177[34] = 0;
   out_4154347088581999177[35] = 0;
   out_4154347088581999177[36] = 0;
   out_4154347088581999177[37] = 0;
   out_4154347088581999177[38] = 0;
   out_4154347088581999177[39] = 0;
   out_4154347088581999177[40] = 0;
   out_4154347088581999177[41] = 0;
   out_4154347088581999177[42] = 0;
   out_4154347088581999177[43] = 0;
   out_4154347088581999177[44] = 1;
   out_4154347088581999177[45] = 0;
   out_4154347088581999177[46] = 0;
   out_4154347088581999177[47] = 1;
   out_4154347088581999177[48] = 0;
   out_4154347088581999177[49] = 0;
   out_4154347088581999177[50] = 0;
   out_4154347088581999177[51] = 0;
   out_4154347088581999177[52] = 0;
   out_4154347088581999177[53] = 0;
}
void h_10(double *state, double *unused, double *out_8153320187445631634) {
   out_8153320187445631634[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8153320187445631634[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8153320187445631634[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6855067043592370112) {
   out_6855067043592370112[0] = 0;
   out_6855067043592370112[1] = 9.8100000000000005*cos(state[1]);
   out_6855067043592370112[2] = 0;
   out_6855067043592370112[3] = 0;
   out_6855067043592370112[4] = -state[8];
   out_6855067043592370112[5] = state[7];
   out_6855067043592370112[6] = 0;
   out_6855067043592370112[7] = state[5];
   out_6855067043592370112[8] = -state[4];
   out_6855067043592370112[9] = 0;
   out_6855067043592370112[10] = 0;
   out_6855067043592370112[11] = 0;
   out_6855067043592370112[12] = 1;
   out_6855067043592370112[13] = 0;
   out_6855067043592370112[14] = 0;
   out_6855067043592370112[15] = 1;
   out_6855067043592370112[16] = 0;
   out_6855067043592370112[17] = 0;
   out_6855067043592370112[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6855067043592370112[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6855067043592370112[20] = 0;
   out_6855067043592370112[21] = state[8];
   out_6855067043592370112[22] = 0;
   out_6855067043592370112[23] = -state[6];
   out_6855067043592370112[24] = -state[5];
   out_6855067043592370112[25] = 0;
   out_6855067043592370112[26] = state[3];
   out_6855067043592370112[27] = 0;
   out_6855067043592370112[28] = 0;
   out_6855067043592370112[29] = 0;
   out_6855067043592370112[30] = 0;
   out_6855067043592370112[31] = 1;
   out_6855067043592370112[32] = 0;
   out_6855067043592370112[33] = 0;
   out_6855067043592370112[34] = 1;
   out_6855067043592370112[35] = 0;
   out_6855067043592370112[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6855067043592370112[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6855067043592370112[38] = 0;
   out_6855067043592370112[39] = -state[7];
   out_6855067043592370112[40] = state[6];
   out_6855067043592370112[41] = 0;
   out_6855067043592370112[42] = state[4];
   out_6855067043592370112[43] = -state[3];
   out_6855067043592370112[44] = 0;
   out_6855067043592370112[45] = 0;
   out_6855067043592370112[46] = 0;
   out_6855067043592370112[47] = 0;
   out_6855067043592370112[48] = 0;
   out_6855067043592370112[49] = 0;
   out_6855067043592370112[50] = 1;
   out_6855067043592370112[51] = 0;
   out_6855067043592370112[52] = 0;
   out_6855067043592370112[53] = 1;
}
void h_13(double *state, double *unused, double *out_6653821394755983052) {
   out_6653821394755983052[0] = state[3];
   out_6653821394755983052[1] = state[4];
   out_6653821394755983052[2] = state[5];
}
void H_13(double *state, double *unused, double *out_942073263249666376) {
   out_942073263249666376[0] = 0;
   out_942073263249666376[1] = 0;
   out_942073263249666376[2] = 0;
   out_942073263249666376[3] = 1;
   out_942073263249666376[4] = 0;
   out_942073263249666376[5] = 0;
   out_942073263249666376[6] = 0;
   out_942073263249666376[7] = 0;
   out_942073263249666376[8] = 0;
   out_942073263249666376[9] = 0;
   out_942073263249666376[10] = 0;
   out_942073263249666376[11] = 0;
   out_942073263249666376[12] = 0;
   out_942073263249666376[13] = 0;
   out_942073263249666376[14] = 0;
   out_942073263249666376[15] = 0;
   out_942073263249666376[16] = 0;
   out_942073263249666376[17] = 0;
   out_942073263249666376[18] = 0;
   out_942073263249666376[19] = 0;
   out_942073263249666376[20] = 0;
   out_942073263249666376[21] = 0;
   out_942073263249666376[22] = 1;
   out_942073263249666376[23] = 0;
   out_942073263249666376[24] = 0;
   out_942073263249666376[25] = 0;
   out_942073263249666376[26] = 0;
   out_942073263249666376[27] = 0;
   out_942073263249666376[28] = 0;
   out_942073263249666376[29] = 0;
   out_942073263249666376[30] = 0;
   out_942073263249666376[31] = 0;
   out_942073263249666376[32] = 0;
   out_942073263249666376[33] = 0;
   out_942073263249666376[34] = 0;
   out_942073263249666376[35] = 0;
   out_942073263249666376[36] = 0;
   out_942073263249666376[37] = 0;
   out_942073263249666376[38] = 0;
   out_942073263249666376[39] = 0;
   out_942073263249666376[40] = 0;
   out_942073263249666376[41] = 1;
   out_942073263249666376[42] = 0;
   out_942073263249666376[43] = 0;
   out_942073263249666376[44] = 0;
   out_942073263249666376[45] = 0;
   out_942073263249666376[46] = 0;
   out_942073263249666376[47] = 0;
   out_942073263249666376[48] = 0;
   out_942073263249666376[49] = 0;
   out_942073263249666376[50] = 0;
   out_942073263249666376[51] = 0;
   out_942073263249666376[52] = 0;
   out_942073263249666376[53] = 0;
}
void h_14(double *state, double *unused, double *out_7681979440912549899) {
   out_7681979440912549899[0] = state[6];
   out_7681979440912549899[1] = state[7];
   out_7681979440912549899[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7237135520877371473) {
   out_7237135520877371473[0] = 0;
   out_7237135520877371473[1] = 0;
   out_7237135520877371473[2] = 0;
   out_7237135520877371473[3] = 0;
   out_7237135520877371473[4] = 0;
   out_7237135520877371473[5] = 0;
   out_7237135520877371473[6] = 1;
   out_7237135520877371473[7] = 0;
   out_7237135520877371473[8] = 0;
   out_7237135520877371473[9] = 0;
   out_7237135520877371473[10] = 0;
   out_7237135520877371473[11] = 0;
   out_7237135520877371473[12] = 0;
   out_7237135520877371473[13] = 0;
   out_7237135520877371473[14] = 0;
   out_7237135520877371473[15] = 0;
   out_7237135520877371473[16] = 0;
   out_7237135520877371473[17] = 0;
   out_7237135520877371473[18] = 0;
   out_7237135520877371473[19] = 0;
   out_7237135520877371473[20] = 0;
   out_7237135520877371473[21] = 0;
   out_7237135520877371473[22] = 0;
   out_7237135520877371473[23] = 0;
   out_7237135520877371473[24] = 0;
   out_7237135520877371473[25] = 1;
   out_7237135520877371473[26] = 0;
   out_7237135520877371473[27] = 0;
   out_7237135520877371473[28] = 0;
   out_7237135520877371473[29] = 0;
   out_7237135520877371473[30] = 0;
   out_7237135520877371473[31] = 0;
   out_7237135520877371473[32] = 0;
   out_7237135520877371473[33] = 0;
   out_7237135520877371473[34] = 0;
   out_7237135520877371473[35] = 0;
   out_7237135520877371473[36] = 0;
   out_7237135520877371473[37] = 0;
   out_7237135520877371473[38] = 0;
   out_7237135520877371473[39] = 0;
   out_7237135520877371473[40] = 0;
   out_7237135520877371473[41] = 0;
   out_7237135520877371473[42] = 0;
   out_7237135520877371473[43] = 0;
   out_7237135520877371473[44] = 1;
   out_7237135520877371473[45] = 0;
   out_7237135520877371473[46] = 0;
   out_7237135520877371473[47] = 0;
   out_7237135520877371473[48] = 0;
   out_7237135520877371473[49] = 0;
   out_7237135520877371473[50] = 0;
   out_7237135520877371473[51] = 0;
   out_7237135520877371473[52] = 0;
   out_7237135520877371473[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_6415261615196083022) {
  err_fun(nom_x, delta_x, out_6415261615196083022);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_585391152892609151) {
  inv_err_fun(nom_x, true_x, out_585391152892609151);
}
void pose_H_mod_fun(double *state, double *out_6122232392299276739) {
  H_mod_fun(state, out_6122232392299276739);
}
void pose_f_fun(double *state, double dt, double *out_3229721450137080995) {
  f_fun(state,  dt, out_3229721450137080995);
}
void pose_F_fun(double *state, double dt, double *out_2983843676786729027) {
  F_fun(state,  dt, out_2983843676786729027);
}
void pose_h_4(double *state, double *unused, double *out_1486608676067422377) {
  h_4(state, unused, out_1486608676067422377);
}
void pose_H_4(double *state, double *unused, double *out_4154347088581999177) {
  H_4(state, unused, out_4154347088581999177);
}
void pose_h_10(double *state, double *unused, double *out_8153320187445631634) {
  h_10(state, unused, out_8153320187445631634);
}
void pose_H_10(double *state, double *unused, double *out_6855067043592370112) {
  H_10(state, unused, out_6855067043592370112);
}
void pose_h_13(double *state, double *unused, double *out_6653821394755983052) {
  h_13(state, unused, out_6653821394755983052);
}
void pose_H_13(double *state, double *unused, double *out_942073263249666376) {
  H_13(state, unused, out_942073263249666376);
}
void pose_h_14(double *state, double *unused, double *out_7681979440912549899) {
  h_14(state, unused, out_7681979440912549899);
}
void pose_H_14(double *state, double *unused, double *out_7237135520877371473) {
  H_14(state, unused, out_7237135520877371473);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
