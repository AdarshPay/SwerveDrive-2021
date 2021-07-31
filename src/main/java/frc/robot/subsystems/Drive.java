// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.DriveDefault;

public class Drive extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final CANSparkMax leftForwardMotor = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax leftForwardAngleMotor = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax leftBackMotor = new CANSparkMax(2, MotorType.kBrushless);
    private final CANSparkMax leftBackAngleMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax rightForwardMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax rightForwardAngleMotor = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax rightBackMotor = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax rightBackAngleMotor = new CANSparkMax(7, MotorType.kBrushless);
    
    private CANEncoder leftForwardEncoder;
    private CANEncoder leftBackEncoder;
    private CANEncoder rightForwardEncoder;
    private CANEncoder rightBackEncoder;

    private DutyCycleEncoder backRightAbsoluteEncoder = new DutyCycleEncoder(1);
    private DutyCycleEncoder backLeftAbsoluteEncoder = new DutyCycleEncoder(0);
    private DutyCycleEncoder frontRightAbsoluteEncoder = new DutyCycleEncoder(2);
    private DutyCycleEncoder frontLeftAbsoluteEncoder = new DutyCycleEncoder(3);

    private CANPIDController leftForwardPIDController;
    private CANPIDController leftBackPIDController;
    private CANPIDController rightForwardPIDController;
    private CANPIDController rightBackPIDController;

    private double kF = 0;
    private double kP = 0.3;
    private double kI = 0;
    private double kD = 0;

    private double initialBackRightAbsVal;
    private double initialFrontRightAbsVal;
    private double initialBackLeftAbsVal;
    private double initialFrontLeftAbsVal;

    private SwerveModule[] swerveModules = new SwerveModule[] {
        new SwerveModule(0, leftForwardAngleMotor, leftForwardMotor),
        new SwerveModule(1, leftBackAngleMotor, leftBackMotor),
        new SwerveModule(2, rightForwardAngleMotor, rightForwardMotor),
        new SwerveModule(4, rightBackAngleMotor, rightBackMotor),
    };

    public Drive() {
    }

    public void setDriveMotorPercents(double percent) {
        leftForwardMotor.set(percent);
        leftBackMotor.set(percent);
        rightForwardMotor.set(percent);
        rightBackMotor.set(percent);
    }

    public void getAngleMotorAngle(double navxOffset) {
        
    }

    public void setAnglePid(double targetAngle, double navxOffset) {
        targetAngle = targetAngle + navxOffset;

        SmartDashboard.putNumber("Navx Offset", navxOffset);

        SmartDashboard.putNumber("Original", initialBackLeftAbsVal);
        SmartDashboard.putNumber("Absolute Encoder", backLeftAbsoluteEncoder.get());
        SmartDashboard.putNumber("Integrated Encoder", leftBackEncoder.getPosition());

        double leftForwardEncoderTics = (targetAngle) * 18/360;
        leftForwardEncoderTics = leftForwardEncoderTics + leftForwardEncoder.getPosition() + (initialFrontLeftAbsVal) + (frontLeftAbsoluteEncoder.get() * 18);

        // System.out.println(leftForwardEncoderTics);

        // SmartDashboard.putNumber("LeftForward Encoder Tics", leftForwardEncoderTics);

        double rightForwardEncoderTics = (targetAngle) * 18/360;
        rightForwardEncoderTics = rightForwardEncoderTics + rightForwardEncoder.getPosition() + (initialFrontRightAbsVal) + (frontRightAbsoluteEncoder.get() * 18);

        // SmartDashboard.putNumber("RightForward Encoder Tics", rightForwardEncoderTics);

        double leftBackEncoderTics = (targetAngle) * 18/360;
        leftBackEncoderTics = leftBackEncoderTics + leftBackEncoder.getPosition() + (initialBackLeftAbsVal) + (backLeftAbsoluteEncoder.get() * 18);

        // System.out.println(leftBackEncoderTics);

        SmartDashboard.putNumber("LeftBack Encoder Tics", leftBackEncoderTics);
        
        double rightBackEncoderTics = (targetAngle) * 18/360;
        rightBackEncoderTics = rightBackEncoderTics + rightBackEncoder.getPosition() + (initialBackRightAbsVal) + (backRightAbsoluteEncoder.get() * 18);

        // SmartDashboard.putNumber("RightBack Encoder Tics", rightBackEncoderTics);

        leftForwardPIDController.setReference(leftForwardEncoderTics, ControlType.kPosition);
        leftBackPIDController.setReference(leftBackEncoderTics, ControlType.kPosition);
        rightForwardPIDController.setReference(rightForwardEncoderTics, ControlType.kPosition);
        rightBackPIDController.setReference(rightBackEncoderTics, ControlType.kPosition);
    }

    public double getJoystickAngle(double joystickUp, double joystickSide) {
        if(Math.abs(joystickUp) < 0.05 && Math.abs(joystickSide) < 0.005) {
            return 0.0;
        }
        double joystickAngle = Math.atan2(-joystickUp, joystickSide);
        joystickAngle = (joystickAngle * 180/Math.PI) + 180;
        return joystickAngle;
    }

    public double getCurrentRotation() {
        double currentAngle = (leftForwardEncoder.getPosition()) * 360/18;
        return currentAngle;
    }

    public void postAbsoluteEncoder() {
        SmartDashboard.putNumber("Angle", (rightBackEncoder.getPosition() - backRightAbsoluteEncoder.get() * 18 - initialBackRightAbsVal) * 180/18);
        SmartDashboard.putNumber("Absolute Encoder", backLeftAbsoluteEncoder.get());
        SmartDashboard.putNumber("Integrated Encoder", leftBackEncoder.getPosition());
        SmartDashboard.putNumber("Original Value", initialBackLeftAbsVal);
        // SmartDashboard.putNumber("Back Right", backRightAbsoluteEncoder.get() * 18);
        // SmartDashboard.putNumber("Front Left", frontLeftAbsoluteEncoder.get() * 18);
        // SmartDashboard.putNumber("Front Right", frontRightAbsoluteEncoder.get() * 18);
    }

    public double getDriveMotorPercent(double joystickUp, double joystickSide) {
        if(Math.abs(joystickUp) < 0.1) {
            joystickUp = 0;
        }
        if(Math.abs(joystickSide) < 0.1) {
            joystickSide = 0;
        }
        double upSquared = Math.pow(joystickUp, 2);
        double sideSquared = Math.pow(joystickSide, 2);
        double currentMotorPercent = Math.sqrt(upSquared + sideSquared);
        // double currentMotorPercent = (Math.sqrt(Math.abs(Math.pow(joystickUp, 2)) + Math.abs(Math.pow(joystickSide, 2))))/Math.sqrt(2);
        return currentMotorPercent;
    }

    public void init() {
        setDefaultCommand(new DriveDefault(this));

        leftForwardEncoder = leftForwardAngleMotor.getEncoder();
        leftForwardEncoder.setPosition(0);

        leftBackEncoder = leftBackAngleMotor.getEncoder();
        leftBackEncoder.setPosition(0);

        rightForwardEncoder = rightForwardAngleMotor.getEncoder();
        rightForwardEncoder.setPosition(0);

        rightBackEncoder = rightBackAngleMotor.getEncoder();
        rightBackEncoder.setPosition(0);

        leftForwardPIDController = new CANPIDController(leftForwardAngleMotor);
        leftForwardPIDController.setFF(kF);
        leftForwardPIDController.setP(kP);
        leftForwardPIDController.setI(kI);
        leftForwardPIDController.setD(kD);
        // leftForwardPIDController.setSmartMotionAllowedClosedLoopError(.1, 0);

        leftBackPIDController = new CANPIDController(leftBackAngleMotor);
        leftBackPIDController.setFF(kF);
        leftBackPIDController.setP(kP);
        leftBackPIDController.setI(kI);
        leftBackPIDController.setD(kD);
        // leftBackPIDController.setSmartMotionAllowedClosedLoopError(.1, 0);

        rightForwardPIDController = new CANPIDController(rightForwardAngleMotor);
        rightForwardPIDController.setFF(kF);
        rightForwardPIDController.setP(kP);
        rightForwardPIDController.setI(kI);
        rightForwardPIDController.setD(kD);
        // rightForwardPIDController.setSmartMotionAllowedClosedLoopError(.1, 0);

        rightBackPIDController = new CANPIDController(rightBackAngleMotor);
        rightBackPIDController.setFF(kF);
        rightBackPIDController.setP(kP);
        rightBackPIDController.setI(kI);
        rightBackPIDController.setD(kD);

        // rightBackPIDController.setSmartMotionAllowedClosedLoopError(.1, 0);

        initialBackLeftAbsVal = backLeftAbsoluteEncoder.get() * 18;
        initialBackRightAbsVal = backRightAbsoluteEncoder.get() * 18;
        initialFrontLeftAbsVal = frontLeftAbsoluteEncoder.get() * 18;
        initialFrontRightAbsVal = frontRightAbsoluteEncoder.get() * 18;
    }

    private Command DriveDefault() {
        return null;
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
