package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  // placeholder dimensions
  private double robotWidth = 2;
  private double robotLength = 2;

  private final Pigeon2 gyro;

  private SwerveDriveKinematics kinematics;

  private ModuleIOSparkMax frontLeftModule;
  private ModuleIOSparkMax frontRightModule;
  private ModuleIOSparkMax backLeftModule;
  private ModuleIOSparkMax backRightModule;

  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  // odometry stuff
  private Pose2d robotPose;
  // private Pose2d recentPose;
  private Field2d field;
  private SwerveDrivePoseEstimator poseEstimator;

  private PIDController snakePIDController;
  private double snakeAngleVector = 0;

  public Drive() {
    frontLeftLocation = new Translation2d(robotWidth / 2, robotLength / 2);
    frontRightLocation = new Translation2d(robotWidth / 2, -robotLength / 2);
    backLeftLocation = new Translation2d(-robotWidth / 2, robotLength / 2);
    backRightLocation = new Translation2d(-robotWidth / 2, -robotLength / 2);

    // change can id
    frontLeftModule = new ModuleIOSparkMax(10, 11, 0, 0.0);
    frontRightModule = new ModuleIOSparkMax(12, 13, 1, 0.0);
    backLeftModule = new ModuleIOSparkMax(14, 15, 2, 0.0);
    backRightModule = new ModuleIOSparkMax(16, 17, 3, 0.0);

    kinematics = new SwerveDriveKinematics(getModuleTranslations());

    snakePIDController = new PIDController(3, 0, 0);
    snakePIDController.enableContinuousInput(-Math.PI, Math.PI);

    // change for type of gyro and id
    gyro = new Pigeon2(0);
    gyro.setYaw(0);

    field = new Field2d();
    robotPose = new Pose2d();
  }

  public double getGyroRadians() {
    return Math.toRadians(gyro.getYaw().getValueAsDouble());
  }

  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromRadians(getGyroRadians());
  }

  public void drive(ChassisSpeeds fieldSpeeds) {
    System.out.println(fieldSpeeds.omegaRadiansPerSecond);
    switch (Constants.currentMode) {
      case REAL:
        ChassisSpeeds chassisSpeeds =
            ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getGyroRotation2d()), 0.02);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
        break;

      case SIM:
        ChassisSpeeds simSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, robotPose.getRotation());
        // divide by 50 is to account for 20ms loop. 50 loops in a second.
        double modX = simSpeeds.vxMetersPerSecond / 50;
        double modY = simSpeeds.vyMetersPerSecond / 50;
        Rotation2d modOmega = Rotation2d.fromRadians(simSpeeds.omegaRadiansPerSecond / 50);

        Transform2d modTransform = new Transform2d(modX, modY, modOmega);

        robotPose = robotPose.plus(modTransform);
        break;

      default:
        break;
    }
  }

  public void driveSnake(ChassisSpeeds fieldSpeeds) {
    double vecX = fieldSpeeds.vxMetersPerSecond;
    double vecY = fieldSpeeds.vyMetersPerSecond;

    if (Math.hypot(vecX, vecY) > 0) {
      snakeAngleVector = Math.atan2(vecY, vecX);
    }

    switch (Constants.currentMode) {
      case REAL:
        ChassisSpeeds chassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getGyroRotation2d());

        double x = chassisSpeeds.vxMetersPerSecond;
        double y = chassisSpeeds.vyMetersPerSecond;
        double omega = snakePIDController.calculate(getGyroRadians(), snakeAngleVector);

        ChassisSpeeds snakeSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(x, y, omega), 0.02);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(snakeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
        break;

      case SIM:
        ChassisSpeeds simSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, robotPose.getRotation());

        Rotation2d modOmega =
            Rotation2d.fromRadians(
                snakePIDController.calculate(robotPose.getRotation().getRadians(), snakeAngleVector)
                    / 50);

        // divide by 50 is to account for 20ms loop. 50 loops in a second.
        double modX = simSpeeds.vxMetersPerSecond / 50;
        double modY = simSpeeds.vyMetersPerSecond / 50;

        Transform2d modTransform = new Transform2d(modX, modY, modOmega);

        robotPose = robotPose.plus(modTransform);
        break;

      default:
        break;
    }
  }

  public void updateRobotPose() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = frontLeftModule.getPosition();
    positions[1] = frontRightModule.getPosition();
    positions[2] = backLeftModule.getPosition();
    positions[3] = backRightModule.getPosition();

    robotPose = poseEstimator.update(getGyroRotation2d(), positions);
  }

  public void updatePoseEstimator() {
    // vision pseudo code
    // recentPose = vision.get_pose
    // if recentPose is not none
    //  poseEstimator.addVisionMeasurement(recentPose, vision.timestamp)
    // updateRobotPose()

  }

  public void stop() {
    drive(new ChassisSpeeds());
  }

  public void stopWithX() {
    frontLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    stop();
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    };
  }

  public void periodic() {
    field.setRobotPose(robotPose);
    SmartDashboard.putData(field);
  }
}
