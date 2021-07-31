package frc.robot.commands.defaults;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.tools.Sensors.Navx;
import frc.robot.subsystems.Drive;

public class DriveDefault extends CommandBase {
  /** Creates a new DriveDefault. */
  private static Drive drive;

  private static AHRS ahrs = new AHRS(Port.kMXP);

  private static Navx navx = new Navx(ahrs);
  

  public DriveDefault(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
    // navx.softResetAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navx.softResetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.getAngleMotorAngle(navx.currentAngle());
    // drive.postAbsoluteEncoder();
    if(OI.driverController.getAButton()) {
      drive.setAnglePid(90, navx.currentAngle());
    }
    else if(OI.driverController.getBButton()) {
      drive.setAnglePid(180, navx.currentAngle());
    }
    else if(OI.driverController.getYButton()) {
      drive.setAnglePid(270, navx.currentAngle());
    }
    else if(OI.getDriverLeftY() != 0 && OI.getDriverLeftX() != 0){
      drive.setAnglePid(drive.getJoystickAngle(OI.getDriverLeftY(), OI.getDriverLeftX()), navx.currentAngle());
      drive.setDriveMotorPercents(drive.getDriveMotorPercent(OI.getDriverLeftY(), OI.getDriverLeftX()));
    }
    else {
      drive.setDriveMotorPercents(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
