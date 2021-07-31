// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.DriveDefault;

public class SwerveModule extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    public SwerveModule(int moduleNumber, CANSparkMax angledMotor, CANSparkMax forwardBackMotor) {
        angleMotor = angledMotor;
        driveMotor = forwardBackMotor;
        driveMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setAngleDrivePercent(double percent) {
        angleMotor.set(percent);
    }

    public void setDriveMotorPercent(double percent) {
        driveMotor.set(percent);
    }

    public void setDriveEncoderZero(DutyCycleEncoder angleEncoder) {
        angleEncoder.reset();
    }

    public double getCurrentAngle(DutyCycleEncoder encoder) {
        double currentAngle = encoder.getDistance() * 360;
        return currentAngle;
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
