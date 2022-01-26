// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Package
package frc.robot.subsystems;

// Libraries

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Conveyor extends SubsystemBase {

    private final WPI_VictorSPX flywheel = new WPI_VictorSPX(ConveyorConstants.mainPort);
    
    public void conveyorStart (double speed) {
        flywheel.set(ControlMode.PercentOutput, speed);
    }

    public void conveyorStop () {
        flywheel.stopMotor();
    }








}
