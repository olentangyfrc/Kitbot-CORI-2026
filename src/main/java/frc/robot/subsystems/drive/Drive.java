package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  private SwerveModulePosition[] modulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private ModuleIO frontLeftModule;
  private ModuleIO frontRightModule;
  private ModuleIO backLeftModule;
  private ModuleIO backRightModule;

  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  // odometry stuff
  private Pose2d robotPose = new Pose2d();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  // private Pose2d recentPose;
  private Field2d field = new Field2d();
  private SwerveDrivePoseEstimator poseEstimator;

  public Drive() {
    frontLeftLocation = new Translation2d(robotWidth / 2, robotLength / 2);
    frontRightLocation = new Translation2d(robotWidth / 2, -robotLength / 2);
    backLeftLocation = new Translation2d(-robotWidth / 2, robotLength / 2);
    backRightLocation = new Translation2d(-robotWidth / 2, -robotLength / 2);

    // change can id
    frontLeftModule = new ModuleIO(10, 11, 0, 0.0);
    frontRightModule = new ModuleIO(12, 13, 1, 0.0);
    backLeftModule = new ModuleIO(14, 15, 2, 0.0);
    backRightModule = new ModuleIO(16, 17, 3, 0.0);

    kinematics = new SwerveDriveKinematics(getModuleTranslations());

    // change can id
    gyro = new Pigeon2(0);
    gyro.setYaw(0);

    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, getRotation(), modulePositions, robotPose);
  }

  public Rotation2d getRotation() {
    switch (Constants.currentMode) {
      case REAL:
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
      case SIM:
        return robotPose.getRotation();
      default:
        return new Rotation2d();
    }
  }

  public void resetGyro() {
    gyro.reset();
    robotPose = new Pose2d(robotPose.getX(), robotPose.getY(), Rotation2d.kZero);
  }

  public void drive(ChassisSpeeds fieldSpeeds) {
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getRotation()), 0.02);
    this.chassisSpeeds = chassisSpeeds;

    switch (Constants.currentMode) {
      case REAL:
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
        break;

      case SIM:
        // divide by 50 is to account for 20ms loop. 50 loops in a second.
        double modX = chassisSpeeds.vxMetersPerSecond / 50;
        double modY = chassisSpeeds.vyMetersPerSecond / 50;
        Rotation2d modOmega = Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond / 50);

        Transform2d modTransform = new Transform2d(modX, modY, modOmega);

        robotPose = robotPose.plus(modTransform);
        break;

      default:
        break;
    }
  }

  public Pose2d getPose() {
    switch (Constants.currentMode) {
      case REAL:
        return poseEstimator.getEstimatedPosition();
      case SIM:
        return robotPose;
      default:
        return new Pose2d();
    }
  }

  public void updateRobotPose() {
    switch (Constants.currentMode) {
      case REAL:
        modulePositions[0] = frontLeftModule.getPosition();
        modulePositions[1] = frontRightModule.getPosition();
        modulePositions[2] = backLeftModule.getPosition();
        modulePositions[3] = backRightModule.getPosition();

        robotPose = poseEstimator.update(getRotation(), modulePositions);
        break;
      case SIM:
        break;
      default:
        break;
    }
  }

  public void addVisionMeasurment(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    switch (Constants.currentMode) {
      case REAL:
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        break;
      case SIM:
        break;
      default:
        break;
    }
  }

  public void stop() {
    drive(new ChassisSpeeds());
  }

  public void stopWithX() {
    switch (Constants.currentMode) {
      case REAL:
        frontLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        break;
      case SIM:
        break;
      default:
        break;
    }
    stop();
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    };
  }

  public ChassisSpeeds getChassisSpeeds() {
    switch (Constants.currentMode) {
      case REAL:
        return kinematics.toChassisSpeeds(getModuleStates());
      case SIM:
        return chassisSpeeds;
      default:
        return new ChassisSpeeds();
    }
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    };
  }

  public Translation2d getHubPosition() {
    return new Translation2d(4.6, 4);
  }

  // call empty, uses default params, but with input gets specified distance or angle
  public double getDistanceFromHub() {
    return getDistanceFromHub(getHubPosition());
  }

  public double getDistanceFromHub(Translation2d position) {
    return getPose().getTranslation().getDistance(position);
  }

  public double getDistanceFromVirtualHub() {
    return getDistanceFromHub(getVirtualHubPosition());
  }

  public Rotation2d getRotationToHub() {
    return getRotationToHub(getHubPosition());
  }

  public Rotation2d getRotationToHub(Translation2d position) {
    Translation2d currentTranslation = getPose().getTranslation();
    return position.minus(currentTranslation).getAngle();
  }

  public Rotation2d getRotationToVirtualHub() {
    return getRotationToHub(getVirtualHubPosition());
  }

  // shoot on the move stuff
  public Translation2d getVirtualHubPosition() {
    Translation2d robotTranslation = getPose().getTranslation();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
    Translation2d hubPosition = getHubPosition();

    double defaultFuelSpeed =
        10; // change later, estimate of speed of fuel coming out of shooter in m/s
    Translation2d virtualTarget = hubPosition;

    // runs 3 iterations. need distance to calculate time of flight, need time of flight to
    // calculate distance
    // start with distance from actual hub, adjust from there.
    for (int i = 0; i < 3; i++) {
      double distance = robotTranslation.getDistance(virtualTarget);
      double timeOfFlight = distance / defaultFuelSpeed;
      virtualTarget =
          new Translation2d(
              hubPosition.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight,
              hubPosition.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight);
    }
    return virtualTarget;
  }

  public void periodic() {
    field.setRobotPose(robotPose);
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("virtual hub x", getVirtualHubPosition().getX());
    SmartDashboard.putNumber("virtual hub y", getVirtualHubPosition().getY());

    updateRobotPose();
  }
}
