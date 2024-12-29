#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor lift(-7);
inline pros::Motor intake(5);

inline pros::Rotation rotation(17);
inline pros::Optical optical(4);
inline pros::adi::DigitalIn increase('G');


inline ez::Piston mogo('H');
inline ez::Piston arm('F');
inline ez::Piston intakelift('E');


