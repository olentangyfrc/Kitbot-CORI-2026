// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive();
        shooter = new Shooter();
        vision = new Vision(drive, "limelight");
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive();
        shooter = new Shooter();
        vision = new Vision(drive, "limelight");
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive();
        shooter = new Shooter();
        vision = new Vision(drive, "limelight");
        break;
    }

    shooter.init();
    vision.setup();
    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // autoChooser = null;
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            // temporary fix for some random bug
            () -> -controller.getRawAxis(2)
            // () -> -controller.getRightX()
            ));

    shooter.setDefaultCommand(ShooterCommands.stop(shooter));
    // Snake command, front is always forwards
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveSnake(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    // unused point to hub code, shoot on move is better
    // controller
    //     .x()
    //     .whileTrue(
    //         DriveCommands.pointToHub(
    //             drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    controller
        .rightTrigger(0.35)
        .whileTrue(
            DriveCommands.shootOnTheMove(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // point to hub
    // spin up shooter
    // wait for shooter to get up to speed
    // run indexers
    controller.rightTrigger(0.35).whileTrue(ShooterCommands.spinUp(shooter));
    controller.rightTrigger(0.35).whileFalse(ShooterCommands.stop(shooter));

    controller.leftTrigger().whileTrue(ShooterCommands.intake(shooter));

    controller.b().whileTrue(ShooterCommands.eject(shooter));

    controller.start().whileTrue(DriveCommands.resetGyro(drive));
  }
  ;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return null;
  }
}
