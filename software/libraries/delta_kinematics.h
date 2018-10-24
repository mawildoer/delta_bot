/*
Credit to original author: mzavatsky
Images and diagrams highlighint constants available at: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/

Mzavatsky's software was wrapped into a class by Matthew Wildoer
*/


#ifndef DELTA_KINE_h
#define DELTA_KINE_h

#include "Arduino.h"
#include "math.h"

class DeltaKinematics {
private:
  // trigonometric constants
  static const double sqrt3;
  static const double pi;    // PI
  static const double sin120;
  static const double cos120;
  static const double tan60;
  static const double sin30;
  static const double tan30;

  // inverse kinematics
  // helper functions, calculates angle theta1 (for YZ-pane)
  int delta_calcAngleYZ(double x0, double y0, double z0, double *theta_ptr);

public:

  // robot geometry
  // 'side lengths' are reffering to the effectics side length if the base/end effector are triangles
  float end_effector_radius; // radius from the center of the end effector to the effective link point
  float base_radius; // radius from the center of the base to the point of rotation of horns
  float link_length; // linkage arm length
  float horn_length; // servo horn length

  DeltaKinematics(float end_effector_radius, float base_radius, float link_length, float horn_length): end_effector_radius(end_effector_radius), base_radius(base_radius), link_length(link_length), horn_length(horn_length) {}

  // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
  // returned status: 0=OK, -1=non-existing position
  // theta1 is in negative y direction. Named counter-clockwise looking from the top
  int jointToCart(double theta1, double theta2, double theta3, double *x0, double *y0, double *z0);

  // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
  // returned status: 0=OK, -1=non-existing position
  // theta1 is in negative y direction. Named counter-clockwise looking from the top
  int cartToJoint(double x0, double y0, double z0, double *theta1, double *theta2, double *theta3);
};

#endif
