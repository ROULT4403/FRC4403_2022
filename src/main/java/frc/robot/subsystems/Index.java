// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class Index extends SubsystemBase {

    // Motor controllers
    private final VictorSPX mainMotor = new VictorSPX(IndexConstants.portConveyor);

    // Sensors
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private final AnalogInput ultrasonic = new AnalogInput(IndexConstants.ultrasonicPort);

    public void ConveyorControl (double speed) {
        mainMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}