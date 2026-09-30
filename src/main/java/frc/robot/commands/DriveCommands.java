package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  public static final double DEADBAND = 0.1;
  public static final double maxLinearSpeed = Constants.maxLinearSpeed; // mps
  public static final double maxRotationalSpeed = Constants.maxRotationalSpeed; // rad per sec
  public static double snakeAngle = 0;

  private DriveCommands() {}

  public static Command joystickDrive(
      Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return Commands.run(
        () -> {
          double xSpeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
          double ySpeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
          double omegaSpeed = MathUtil.applyDeadband(omega.getAsDouble(), DEADBAND);

          xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed) * maxLinearSpeed;
          ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed) * maxLinearSpeed;
          omegaSpeed = Math.copySign(omegaSpeed * omegaSpeed, omegaSpeed) * maxRotationalSpeed;

          ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
          drive.drive(chassisSpeeds);
        },
        drive);
  }

  public static Command joystickDriveSnake(
      Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    PIDController angleController = new PIDController(8, 0, 0);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {
              double xSpeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
              double ySpeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
              double omegaSpeed = MathUtil.applyDeadband(omega.getAsDouble(), DEADBAND);

              if (Math.hypot(xSpeed, ySpeed) > DEADBAND) {
                snakeAngle = Math.atan2(ySpeed, xSpeed);
              }

              xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed) * maxLinearSpeed;
              ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed) * maxLinearSpeed;
              omegaSpeed = Math.copySign(omegaSpeed * omegaSpeed, omegaSpeed) * maxRotationalSpeed;

              double snakeOmegaSpeed =
                  angleController.calculate(drive.getRotation().getRadians(), snakeAngle);

              if (omegaSpeed != 0) {
                snakeOmegaSpeed = omegaSpeed;
              }

              ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, snakeOmegaSpeed);
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

    PIDController angleController = new PIDController(8, 0, 0);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {
              double xSpeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
              double ySpeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
              double omegaSpeed =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotation.get().getRadians());

              xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed) * maxLinearSpeed;
              ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed) * maxLinearSpeed;

              ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
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
