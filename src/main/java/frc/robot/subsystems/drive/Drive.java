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
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  // meters
  private final double robotWidth = 0.8128;
  private final double robotLength = 0.5842;

  private Pigeon2 gyro;

  private SwerveDriveKinematics kinematics;

  private final ModuleIO frontLeftModule;
  private final ModuleIO frontRightModule;
  private final ModuleIO backLeftModule;
  private final ModuleIO backRightModule;

  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  private SwerveModulePosition[] modulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // odometry stuff
  private Pose2d robotPose = new Pose2d();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  // private Pose2d recentPose;
  private final Field2d field = new Field2d();
  private final FieldObject2d virtualHub = field.getObject("virtualHub");
  private SwerveDrivePoseEstimator poseEstimator;

  private final double angleTolerance = 3;

  public Drive() {
    frontLeftLocation = new Translation2d(robotWidth / 2, robotLength / 2);
    frontRightLocation = new Translation2d(robotWidth / 2, -robotLength / 2);
    backLeftLocation = new Translation2d(-robotWidth / 2, robotLength / 2);
    backRightLocation = new Translation2d(-robotWidth / 2, -robotLength / 2);

    // change encoder id, calculate offsets
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
  /**
   * Gets current rotation of the robot.
   *
   * @return Robot Gyro or pose rotation
   */
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
  /** Resets robot gyro and pose rotation to 0. */
  public void resetGyro() {
    gyro.reset();
    robotPose = new Pose2d(robotPose.getX(), robotPose.getY(), Rotation2d.kZero);
  }
  /**
   * Drives the robot by translating chassis speeds into swerve module signals.
   *
   * @param fieldSpeeds
   */
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
  /** Gets current pose estimator pose or sim pose. */
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
  /** Updates pose estimator with swerve module positions. */
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
  /**
   * Adds data from limelight tracking to the pose estimator.
   *
   * @param visionRobotPoseMeters
   * @param timestampSeconds
   * @param visionMeasurementStdDevs
   */
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
  /** Cuts power to drivetrain. */
  public void stop() {
    drive(new ChassisSpeeds());
  }
  /** Cuts power to drivetrain, but with swerve modules angled in an x to prevent movement. */
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
  /**
   * Gets positions of swerve modules in relation to the center of the robot.
   *
   * @return
   */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    };
  }
  /**
   * Gets current chassis speeds using kinematic or sent chassis speeds in sim.
   *
   * @return A chassisspeeds object
   */
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
  /**
   * Gets swerve module states.
   *
   * @return A list of swerve module states
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getState(),
      frontRightModule.getState(),
      backLeftModule.getState(),
      backRightModule.getState()
    };
  }

  /** Returns the Pose2d of the Hub. */
  public Translation2d getHubPosition() {
    return new Translation2d(4.6, 4);
  }
  /** Gets distance in meters from current pose to the Hub. */
  public double getDistanceFromHub() {
    return getDistanceFromHub(getHubPosition());
  }
  /** Gets distance in meters from current pose to a Pose2d. */
  public double getDistanceFromHub(Translation2d position) {
    return getPose().getTranslation().getDistance(position);
  }
  /** Gets distance in meters from current pose to the Virtual Hub. */
  public double getDistanceFromVirtualHub() {
    return getDistanceFromHub(getVirtualHubPosition());
  }
  /** Gets angle from current pose to the Hub. */
  public Rotation2d getRotationToHub() {
    return getRotationToHub(getHubPosition());
  }
  /** Gets angle from current pose to a Pose2d. */
  public Rotation2d getRotationToHub(Translation2d position) {
    Translation2d currentTranslation = getPose().getTranslation();
    return position.minus(currentTranslation).getAngle();
  }
  /** Gets angle from current pose to the Virtual Hub. */
  public Rotation2d getRotationToVirtualHub() {
    return getRotationToHub(getVirtualHubPosition());
  }

  /**
   * Gets the position of the Virtual Hub, which is where the robot needs to shoot in order to shoot
   * on the move.
   */
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
  /**
   * Sees if the robot is facing the Hub, with a tolerance.
   *
   * @return True if angle is within tolerance
   */
  public boolean isRobotFacingHub() {
    return isRobotFacingHub(getHubPosition());
  }
  /**
   * Sees if the robot is facing a Pose2d, with a tolerance.
   *
   * @return True if angle is within tolerance
   */
  public boolean isRobotFacingHub(Translation2d position) {

    double currentAngle = getPose().getRotation().getRadians() + Math.PI;
    double targetAngle = getRotationToHub(position).getRadians();
    double delta = targetAngle - currentAngle;
    delta = ((delta + Math.PI) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI) - Math.PI;

    return (Math.abs(delta) > angleTolerance);
  }
  /**
   * Sees if the robot is facing the Virtual Hub, with a tolerance.
   *
   * @return True if angle is within tolerance
   */
  public boolean isRobotFacingVirtualHub() {
    return isRobotFacingHub(getVirtualHubPosition());
  }
  /** Sees if the robot is within distance to shoot at the Hub. */
  public boolean canShootAtHub() {
    return (getDistanceFromHub() < 2 && getDistanceFromHub() > 4);
  }
  /** Sees if the robot is within distance to shoot at a Pose2d. */
  public boolean canShootAtHub(Translation2d position, double min, double max) {
    return (getDistanceFromHub(position) > min && getDistanceFromHub(position) < max);
  }
  /** Sees if the robot is within distance to shoot at the Virtual Hub. */
  public boolean canShootAtVirtualHub() {
    return (getDistanceFromVirtualHub() > 2 && getDistanceFromVirtualHub() < 4);
  }

  public void periodic() {
    field.setRobotPose(robotPose);
    virtualHub.setPose(
        getVirtualHubPosition().getX(), getVirtualHubPosition().getY(), new Rotation2d());

    SmartDashboard.putData("poseField", field);
    SmartDashboard.putNumber("virtualHubX", getVirtualHubPosition().getX());
    SmartDashboard.putNumber("virtualHubY", getVirtualHubPosition().getY());

    SmartDashboard.putBoolean("IsRobotFacingVirtualHub", isRobotFacingVirtualHub());
    SmartDashboard.putBoolean("canShootAtVirtualHub", canShootAtVirtualHub());

    updateRobotPose();
  }
}
