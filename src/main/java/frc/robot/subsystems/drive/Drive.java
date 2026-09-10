package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
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
    frontLeftModule = new ModuleIOSparkMax(90, 91, 0, 0.0);
    frontRightModule = new ModuleIOSparkMax(92, 93, 1, 0.0);
    backLeftModule = new ModuleIOSparkMax(94, 95, 2, 0.0);
    backRightModule = new ModuleIOSparkMax(96, 97, 3, 0.0);

    kinematics =
        new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

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

  public void drive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroRotation2d());
    fieldSpeeds = ChassisSpeeds.discretize(fieldSpeeds, 0.02);

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(fieldSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

    frontLeftModule.setState(moduleStates[0]);
    frontRightModule.setState(moduleStates[1]);
    backLeftModule.setState(moduleStates[2]);
    backRightModule.setState(moduleStates[3]);
  }

  public void driveSim(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, robotPose.getRotation());

    double modX = fieldSpeeds.vxMetersPerSecond / 50;
    double modY = fieldSpeeds.vyMetersPerSecond / 50;
    Rotation2d modOmega = Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond / 50);

    Transform2d modTransform = new Transform2d(modX, modY, modOmega);

    robotPose = robotPose.plus(modTransform);

    // fieldSpeeds = ChassisSpeeds.discretize(fieldSpeeds, 0.02);

    // double newX = robotPose.getX() + fieldSpeeds.vxMetersPerSecond / 50;
    // double newY = robotPose.getY() + fieldSpeeds.vyMetersPerSecond / 50;
    // Rotation2d newOmega =
    //     Rotation2d.fromRadians(
    //         robotPose.getRotation().getRadians() + fieldSpeeds.omegaRadiansPerSecond / 50);
    // System.out.println(newOmega.getDegrees());

    // Transform2d newTransform = new Transform2d(newX, newY, newOmega);

    // robotPose = robotPose.plus(newTransform);
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

  public void periodic() {
    field.setRobotPose(robotPose);
    SmartDashboard.putData(field);
  }
}
