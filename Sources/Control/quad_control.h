#ifndef _QUAD_CONTROL_H_INCLUDED_
#define _QUAD_CONTROL_H_INCLUDED_

#include "arith.h"
#include "motors.h"

typedef struct
{
	quat QEst;
	vec3 bff_angle_rate;
	vec3 torque;
	frac thrust;
}controlData_T;

typedef struct{

	quat attitude;
	frac thrust;

}setpoint_T;


vec3 adv_att_control(quat setpoint, quat att, vec3 angle_rate);
frac h_control(frac setpoint, frac h);
void control_mixer(frac thrust, vec3 torque, struct motorData* output);

/*
********** Control de actitud *************
 */

/* Integral */
#define int_gain_divide 8000
#define integral_error_limit 250

/* Realimentación de posicion */
#define prop_gain_frac_xy 6500
#define prop_gain_frac_z 6500

extern vec3 PosGain;

/* Realimentacion de velocidad */

//#define der_gain_int_xy (1)
//#define der_gain_int_z (1)
//const ivec3 VelGain = {der_gain_int_xy, der_gain_int_xy, der_gain_int_z};

#define der_gain_int_xy 256
#define der_gain_int_z 280
extern vec3_q8_8 VelGain;

/* * */


#endif // _QUAD_CONTROL_H_INCLUDED_