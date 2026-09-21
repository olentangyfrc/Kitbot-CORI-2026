package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  public static final double DEADBAND = 0.25;

  private DriveCommands() {}

  public static Command joystickDrive(
      Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () -> {
          double xspeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
          double yspeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
          double omegaspeed = MathUtil.applyDeadband(omega.getAsDouble(), DEADBAND);

          xspeed = Math.copySign(xspeed * xspeed, xspeed) * 3;
          yspeed = Math.copySign(yspeed * yspeed, yspeed) * 3;
          omegaspeed = Math.copySign(omegaspeed * omegaspeed, omegaspeed) * 6;

          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, omegaspeed);
          drive.drive(chassisSpeeds);
        },
        drive);
  }

  public static Command joystickDriveSnake(
      Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () -> {
          double xspeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
          double yspeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
          double omegaspeed = MathUtil.applyDeadband(omega.getAsDouble(), DEADBAND);

          xspeed = Math.copySign(xspeed * xspeed, xspeed) * 3;
          yspeed = Math.copySign(yspeed * yspeed, yspeed) * 3;
          omegaspeed = Math.copySign(omegaspeed * omegaspeed, omegaspeed) * 6;

          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, omegaspeed);
          drive.driveSnake(chassisSpeeds);
        },
        drive);
  }
}
