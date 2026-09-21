package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  public static final double DEADBAND = 0.1;
  public static double snakeAngle = 0;

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

  public static Command joystickDriveSnake(Drive drive, DoubleSupplier x, DoubleSupplier y) {
    PIDController angleController = new PIDController(6, 0, 0);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {
              double xspeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
              double yspeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);

              if (Math.hypot(xspeed, yspeed) > DEADBAND) {
                snakeAngle = Math.atan2(yspeed, xspeed);
              }

              xspeed = Math.copySign(xspeed * xspeed, xspeed) * 3;
              yspeed = Math.copySign(yspeed * yspeed, yspeed) * 3;

              double omegaspeed =
                  angleController.calculate(drive.getRotation().getRadians(), snakeAngle);

              ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, omegaspeed);
              drive.drive(chassisSpeeds);
            },
            drive)
        .beforeStarting(
            () -> {
              snakeAngle = drive.getRotation().getRadians();
              angleController.reset();
            });
  }

  public static Command joystickDriveWithAngle(
      Drive drive, DoubleSupplier x, DoubleSupplier y, Supplier<Rotation2d> rotation) {

    PIDController angleController = new PIDController(6, 0, 0);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {
              double xspeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
              double yspeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
              double omegaspeed =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotation.get().getRadians());

              xspeed = Math.copySign(xspeed * xspeed, xspeed) * 3;
              yspeed = Math.copySign(yspeed * yspeed, yspeed) * 3;

              ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, omegaspeed);
              drive.drive(chassisSpeeds);
            },
            drive)
        .beforeStarting(() -> angleController.reset());
  }

  public static Command pointToHub(Drive drive, DoubleSupplier x, DoubleSupplier y) {
    return joystickDriveWithAngle(drive, x, y, () -> drive.getRotationToHub());
  }

  public static Command shootOnTheMove(Drive drive, DoubleSupplier x, DoubleSupplier y) {
    return joystickDriveWithAngle(drive, x, y, () -> drive.getRotationToVirtualHub());
  }

  public static Command resetGyro(Drive drive) {
    return Commands.run(
        () -> {
          drive.resetGyro();
        },
        drive);
  }
}
