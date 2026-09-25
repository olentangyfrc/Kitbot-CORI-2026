package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

  Shooter shooter;
  Drive drive;

  public static Command setShooterVelocity(Shooter shooter, double velocity) {
    return Commands.run(
        () -> {
          shooter.setShooterSpeed(velocity);
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

  public static Command shoot(Shooter shooter, Drive drive) {
    return Commands.run(
        () -> {
          double distance = drive.getDistanceFromVirtualHub();
          shooter.shootForHub(distance);

          if (shooter.isShooterAtSpeed()
              && drive.isRobotFacingVirtualHub()
              && drive.canShootAtVirtualHub()) {
            shooter.indexerShoot();
          } else {
            shooter.holdIndexer();
          }
        },
        shooter);
  }

  public static Command spinUp(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.spinUp();
        },
        shooter);
  }

  public static Command intake(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.intake();
        },
        shooter);
  }

  public static Command eject(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.eject();
        },
        shooter);
  }
}
