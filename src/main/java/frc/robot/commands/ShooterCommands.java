package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {
  static double shooterShootingSpeed = 200; // change later
  static double shooterIntakingSpeed = 100; // change later

  static double indexerShootingSpeed = -60; // change later
  static double indexerIntakingSpeed = 60; // change later

  Shooter shooter;

  public static Command setShooterVelocity(Shooter shooter, double velocity) {
    return Commands.run(
        () -> {
          shooter.setShooterSpeed(velocity);
        },
        shooter);
  }

  public static Command setIntakeVelocity(Shooter shooter, double velocity) {
    return Commands.run(
        () -> {
          shooter.setIntakeSpeed(velocity);
        },
        shooter);
  }

  public static Command setIndexerVelocity(Shooter shooter, double velocity) {
    return Commands.run(
        () -> {
          shooter.setIndexerSpeed(velocity);
        },
        shooter);
  }

  public static Command stop(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.stop();
        },
        shooter);
  }

  public static Command spinUp(Shooter shooter) {
    return Commands.run(
            () -> {
              shooter.setShooterSpeed(shooterShootingSpeed);
            },
            shooter)
        .until(() -> shooter.isShooterAtSpeed())
        .andThen(
            Commands.run(
                () -> {
                  shooter.setShooterSpeed(shooterShootingSpeed);
                  shooter.setIndexerSpeed(indexerShootingSpeed);
                },
                shooter));
  }

  public static Command intake(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.setIntakeSpeed(shooterIntakingSpeed);
          shooter.setIndexerSpeed(indexerIntakingSpeed);
        },
        shooter);
  }

  public static Command eject(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.setIntakeSpeed(-shooterIntakingSpeed);
          shooter.setIndexerSpeed(-indexerIntakingSpeed);
        },
        shooter);
  }
}
