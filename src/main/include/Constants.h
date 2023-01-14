// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/voltage.h>
#include <units/length.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/SimpleMotorFeedForward.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants
{
    constexpr units::volt_t kS = 0.22_V;
    constexpr auto kV = 1.98 * 1_V / 1_mps;
    constexpr auto kA = 0.2 * 1_V / 1_mps_sq;
    constexpr units::meter_t trackWidth = 0.69_m;
    constexpr auto kVAngular = 1.5 * 1_V/1_mps;
    constexpr auto kAAngular = 0.3 * 1_V / 1_mps_sq;

}
    
