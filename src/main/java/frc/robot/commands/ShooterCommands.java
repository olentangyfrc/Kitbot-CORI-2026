package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShooterCommands {

  Shooter shooter;
  private static final double DEADBAND = Constants.DEADBAND;

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
          SmartDashboard.putString("Shooter State", "STOP");
        },
        shooter);
  }

  public static Command shoot(Shooter shooter, Drive drive) {
    return Commands.run(
        () -> {
          shooter.shoot(shooter.getShooterManualSpeed());

          if (shooter.isShooterAtSpeed()) { // && drive.isRobotFacingVirtualHub() &&
            // drive.canShootAtVirtualHub()
            shooter.indexerShoot();
            SmartDashboard.putString("Shooter State", "SPIN_UP");
          } else {
            shooter.stopIndexer();
            SmartDashboard.putString("Shooter State", "SHOOT");
          }
        },
        shooter);
  }

  public static Command changeManualVelocity(Shooter shooter, DoubleSupplier deltaRaw) {
    return Commands.run(
        () -> {
          double delta = MathUtil.applyDeadband(deltaRaw.getAsDouble(), DEADBAND) * 4;

          shooter.setShooterManualSpeed(
              Math.round(MathUtil.clamp(shooter.getShooterManualSpeed() + delta, 500, 4000)));
        });
  }

  public static Command setManualVelocity(Shooter shooter, Double rpm) {
    return Commands.run(
        () -> {
          shooter.setShooterManualSpeed(Math.round(MathUtil.clamp(rpm, 500, 4000)));
        });
  }

  public static Command spinUp(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.spinUp();
          SmartDashboard.putString("Shooter State", "SPIN_UP");
        },
        shooter);
  }

  public static Command intake(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.intake();
          SmartDashboard.putString("Shooter State", "INTAKE");
        },
        shooter);
  }

  public static Command eject(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.eject();
          SmartDashboard.putString("Shooter State", "EJECT");
        },
        shooter);
  }
}
