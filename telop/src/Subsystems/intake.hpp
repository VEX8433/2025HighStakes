#pragma once

#include "api.h"

class Intake{
  public:
    Intake(pros::Motor bottom, pros::Motor top, pros::v5::Optical optical);

    void telOP(bool R1, bool R2);

  private:
    pros::Motor bottom;
    pros::Motor top;
    pros::v5::Optical optical;
};