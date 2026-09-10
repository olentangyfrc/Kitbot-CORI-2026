package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  public static final double DEADBAND = 0.1;

  private DriveCommands() {}

  public static Command joystickDrive(
      Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    System.out.println("ran joystickdrive");
    return Commands.run(
        () -> {
          double xspeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND) * 3;
          double yspeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND) * 3;
          double omegaspeed = MathUtil.applyDeadband(omega.getAsDouble(), DEADBAND) * 6;

          Math.copySign(xspeed * xspeed, xspeed);
          Math.copySign(yspeed * yspeed, yspeed);
          Math.copySign(omegaspeed * omegaspeed, omegaspeed);

          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, omegaspeed);
          switch (Constants.currentMode) {
            case REAL:
              drive.drive(chassisSpeeds);
              break;

            case SIM:
              drive.driveSim(chassisSpeeds);
              break;

            default:
              break;
          }
        },
        drive);
  }
}
