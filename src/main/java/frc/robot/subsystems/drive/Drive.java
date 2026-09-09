package frc.robot.subsystems.drive;

import java.lang.reflect.Array;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  // odometry stuff
  private Pose2d robotPose;
  private Pose2d recentPose;
  private Field2d field;
  private SwerveDrivePoseEstimator poseEstimator;

  public Drive() {
    final Translation2d frontLeftLocation = new Translation2d(robotWidth / 2, robotLength / 2);
    final Translation2d frontRightLocation = new Translation2d(robotWidth / 2, -robotLength / 2);
    final Translation2d backLeftLocation = new Translation2d(-robotWidth / 2, robotLength / 2);
    final Translation2d backRightLocation = new Translation2d(-robotWidth / 2, -robotLength / 2);

    // change can id
    frontLeftModule = new ModuleIOSparkMax(20, 21, 0, 0.0);
    frontRightModule = new ModuleIOSparkMax(22, 23, 1, 0.0);
    backLeftModule = new ModuleIOSparkMax(24, 25, 2, 0.0);
    backRightModule = new ModuleIOSparkMax(26, 27, 3, 0.0);

    kinematics =
        new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // change for type of gyro and id
    gyro = new Pigeon2(0);
    gyro.setYaw(0);
  }

  public double getGyroRadians() {
    return Math.toRadians(gyro.getYaw().getValueAsDouble());
  }

  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromRadians(getGyroRadians());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroRotation2d());
    fieldSpeeds = ChassisSpeeds.discretize(fieldSpeeds, 0.02);
    
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(fieldSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

    frontLeftModule.setState(moduleStates[0]);
    frontRightModule.setState(moduleStates[1]);
    backLeftModule.setState(moduleStates[2]);
    backRightModule.setState(moduleStates[3]);

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
    //recentPose = vision.get_pose
    //if recentPose is not none
    //  poseEstimator.addVisionMeasurement(recentPose, vision.timestamp)
    //updateRobotPose()
  }



}
