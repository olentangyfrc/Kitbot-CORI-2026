package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

  Shooter shooter;
  Indexer indexer;

  public static Command start(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.start();
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

  public static Command warmUp(Shooter shooter, Indexer indexer) {
    return Commands.run(
        () -> {
          shooter.warmUp(100); // change this later, this is a placeholder
          if (shooter.warmUp(100)) { // change this later, this is a placeholder
            indexer.start(-1); // NOT REAL VALUE change later
          }
          ;
        },
        shooter);
  }
}
