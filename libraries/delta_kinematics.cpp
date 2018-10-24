/*
Credit to original author: mzavatsky
Images and diagrams highlighint constants available at: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/

Mzavatsky's software was wrapped into a class by Matthew Wildoer
*/


#include "delta_kinematics.h"

const double DeltaKinematics::sqrt3  = 1.732050807;
const double DeltaKinematics::pi     = 3.141592653;    // PI
const double DeltaKinematics::sin120 = sqrt3/2.0;
const double DeltaKinematics::cos120 = -0.5;
const double DeltaKinematics::tan60  = sqrt3;
const double DeltaKinematics::sin30  = 0.5;
const double DeltaKinematics::tan30  = 1/sqrt3;

int DeltaKinematics::jointToCart(double theta1, double theta2, double theta3, double *x0, double *y0, double *z0) {
    double t = end_effector_radius - base_radius;//double t = (f-e)*tan30/2;. For edge length format
    double dtr = pi/(double)180.0;

    theta1 *= dtr;
    theta2 *= dtr;
    theta3 *= dtr;

    double y1 = -(t + horn_length*cos(theta1));
    double z1 = -horn_length*sin(theta1);

    double y2 = (t + horn_length*cos(theta2))*sin30;
    double x2 = y2*tan60;
    double z2 = -horn_length*sin(theta2);

    double y3 = (t + horn_length*cos(theta3))*sin30;
    double x3 = -y3*tan60;
    double z3 = -horn_length*sin(theta3);

    double dnm = (y2-y1)*x3-(y3-y1)*x2;

    double w1 = y1*y1 + z1*z1;
    double w2 = x2*x2 + y2*y2 + z2*z2;
    double w3 = x3*x3 + y3*y3 + z3*z3;

    // x = (a1*z + b1)/dnm
    double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
    double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

    // y = (a2*z + b2)/dnm;
    double a2 = -(z2-z1)*x3+(z3-z1)*x2;
    double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

    // a*z^2 + b*z + c = 0
    double a = a1*a1 + a2*a2 + dnm*dnm;
    double b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
    double c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - link_length*link_length);

    // discriminant
    double d = b*b - (double)4.0*a*c;
    if (d < 0) return -1; // non-existing point

    *z0 = -(double)0.5*(b+sqrt(d))/a;
    *x0 = (a1*(*z0) + b1)/dnm;
    *y0 = (a2*(*z0) + b2)/dnm;
    return 0;
}

int DeltaKinematics::delta_calcAngleYZ(double x0, double y0, double z0, double *theta_ptr) {
    double y1 = -base_radius; //double y1 = -0.5 * 0.57735 * f; // f/2 * tg 30. For edge length format
    y0 -= end_effector_radius; //y0 -= 0.5 * 0.57735    * e;    // shift center to edge. For edge length format
    // z = a + b*y
    double a = (x0*x0 + y0*y0 + z0*z0 +horn_length*horn_length - link_length*link_length - y1*y1)/(2*z0);
    double b = (y1-y0)/z0;
    // discriminant
    double d = -(a+b*y1)*(a+b*y1)+horn_length*(b*b*horn_length+horn_length);
    if (d < 0) return -1; // non-existing point
    double yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
    double zj = a + b*yj;
    double theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
    *theta_ptr = theta;
    return 0;
}

int DeltaKinematics::cartToJoint(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3) {
    *theta1 = *theta2 = *theta3 = 0;
    int status = delta_calcAngleYZ(x0, y0, z0, theta1);
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
    if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
    return status;
}
