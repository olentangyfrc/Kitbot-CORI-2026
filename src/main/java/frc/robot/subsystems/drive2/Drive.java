package frc.robot.subsystems.drive2;

import frc.robot.subsystems.drive2.ModuleIOSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.ModuleIO;

public class Drive extends SubsystemBase {
  // placeholder dimensions
  private double robotWidth = 2;
  private double robotLength = 2;

  public Drive() {
    final Translation2d frontLeftLocation = new Translation2d();
    final Translation2d frontRightLocation = new Translation2d();
    final Translation2d backLeftLocation = new Translation2d();
    final Translation2d backRightLocation = new Translation2d();

    // change can id
    ModuleIOSparkMax frontLeftModule = new ModuleIOSparkMax(20, 21, 0, 0.0);
    ModuleIOSparkMax frontRightModule = new ModuleIOSparkMax(22, 23, 1, 0.0);
    ModuleIOSparkMax backLeftModule = new ModuleIOSparkMax(24, 25, 2, 0.0);
    ModuleIOSparkMax backRightModule = new ModuleIOSparkMax(26, 27, 3, 0.0);

    
  }
}
