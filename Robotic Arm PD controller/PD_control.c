#include <stdio.h>
#include <math.h>

double theta_dot_prev = 0.0;
double total_force=50001.0;

double PD_control(theta, theta_dot, theta_ref, theta_dot_ref)
double theta, theta_dot, theta_ref, theta_dot_ref;
{
	
//Calculation for G
double G = 0.882;

//Calculation for B
double const_ang_velo_force = 50000.0; 
double B = const_ang_velo_force/theta_dot;
B = 0.079988;


//Calculation for I
double inertia_const_force = 1.0;
double theta_double_dot = (theta_dot - theta_dot_prev) * 500;
theta_dot_prev = theta_dot;
double theta_double_dot_final = 19.0471645; //Took average of range it varies
double I = inertia_const_force / theta_double_dot;
I = inertia_const_force / theta_double_dot_final;
I=0.052051; 

double torque = G+const_ang_velo_force+inertia_const_force;


//PD Controller

theta_ref=1.57;
theta_dot_ref = 0.5;
double acce = (theta_dot-theta_dot_ref) * 500;
total_force = (-1) * ((I * acce) + (B * theta_dot));

double theta_dot_diff = 2 * (theta_dot-theta_dot_ref);
double theta_diff = (theta-theta_ref); 

double Kd1 = ((-1 * theta_dot_diff) + sqrt(fabs((theta_dot_diff*theta_dot_diff)-(4*theta_diff*total_force))))/(2*theta_diff);
double Kp1 = pow(Kd1,2) / 4;

total_force = ((total_force)/fabs(((Kp1*theta_diff) + (Kd1*theta_dot_diff))));


return(total_force);
}
