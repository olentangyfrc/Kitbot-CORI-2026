package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShooterCommands {

  Shooter shooter;
  private static final double DEADBAND = 0.1;

  // public static Command setShooterVelocity(Shooter shooter, double velocity) {
  //   return Commands.run(
  //       () -> {
  //         shooter.setShooterSpeed(velocity);
  //       },
  //       shooter);
  // }

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

  public static Command shoot(
      Shooter shooter, Drive drive, DoubleSupplier negativeRaw, DoubleSupplier positiveRaw) {
    return Commands.run(
        () -> {
          double positive = MathUtil.applyDeadband(positiveRaw.getAsDouble(), DEADBAND) * 2;
          double negative = MathUtil.applyDeadband(negativeRaw.getAsDouble(), DEADBAND) * 2;

          // double distance = drive.getDistanceFromVirtualHub();
          // shooterManualVelocity = MathUtil.clamp(shooterManualVelocity - negative.getAsDouble() +
          // positive.getAsDouble(), 500, 4000);
          shooter.setShooterManualSpeed(
              MathUtil.clamp(shooter.getShooterManualSpeed() + positive - negative, 500, 4000));

          shooter.shoot(shooter.getShooterManualSpeed());
          SmartDashboard.putNumber("shooterManualVelocity", shooter.getShooterManualSpeed());

          if (shooter.isShooterAtSpeed()) { // && drive.isRobotFacingVirtualHub() &&
            // drive.canShootAtVirtualHub()
            shooter.indexerShoot();
          } else {
            shooter.stopIndexer();
          }
        },
        shooter);
  }
  // /**
  //  * sets manual shoot velocity, takes rpm change per second.
  //  *
  //  * @param shooter
  //  * @param velocity
  //  * @return
  //  */
  // public static Command changeManualVelocity(
  //     Shooter shooter, DoubleSupplier delta) {
  //   return Commands.run(
  //       () -> {
  //         shooter.setShooterManualSpeed(MathUtil.clamp(
  //                 shooter.getShooterManualSpeed() + delta.getAsDouble(),
  //                 500,
  //                 4000));
  //         shooter.shoot(shooter.getShooterManualSpeed());

  //         SmartDashboard.putNumber("shooterManualVelocity", shooter.getShooterManualSpeed());
  //       },
  //       shooter);
  // }
  // public static Command spinUp(Shooter shooter) {
  //   return Commands.run(
  //       () -> {
  //         shooter.spinUp();
  //       },
  //       shooter);
  // }

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
