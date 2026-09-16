package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommands {

  Indexer indexer;

  public static Command startIntaking(Indexer indexer) {
    return Commands.run(
        () -> {
          indexer.start(1); // NOT REAL VALUE change later
        },
        indexer);
  }

  public static Command startShooting(Indexer indexer) {
    return Commands.run(
        () -> {
          indexer.start(-1); // NOT REAL VALUE change later
        },
        indexer);
  }

  public static Command stop(Indexer indexer) {
    return Commands.run(
        () -> {
          indexer.stop();
        },
        indexer);
  }
}
