package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  // need to change!!! measure from center of swerveto center, not frame perimeter!
  private final double robotLength = Units.inchesToMeters(32);
  private final double robotWidth = Units.inchesToMeters(23);

  private final double maxSpeed = Constants.maxLinearSpeed; // mps

  private final Pigeon2 gyro;
  private final Pigeon2Configuration gyroConfig = new Pigeon2Configuration();

  private final ModuleIO frontLeftModule;
  private final ModuleIO frontRightModule;
  private final ModuleIO backLeftModule;
  private final ModuleIO backRightModule;

  private final Translation2d frontLeftLocation =
      new Translation2d(robotWidth / 2, robotLength / 2);
  private final Translation2d frontRightLocation =
      new Translation2d(robotWidth / 2, -robotLength / 2);
  private final Translation2d backLeftLocation =
      new Translation2d(-robotWidth / 2, robotLength / 2);
  private final Translation2d backRightLocation =
      new Translation2d(-robotWidth / 2, -robotLength / 2);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());

  private SwerveModulePosition[] modulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private Pose2d simPose = new Pose2d();
  // private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private final Field2d field = new Field2d();
  // private final FieldObject2d virtualHub = field.getObject("virtualHub");
  private final SwerveDrivePoseEstimator poseEstimator;

  private final double angleTolerance = 7; // degrees

  public Drive() {
    // change encoder id, calculate offsets
    frontLeftModule = new ModuleIO(31, 11, 0, -2.5730803146122083 + Math.PI);
    frontRightModule = new ModuleIO(30, 13, 1, -0.6588291787123399);
    backLeftModule = new ModuleIO(33, 15, 2, -1.5260830950994393);
    backRightModule = new ModuleIO(32, 17, 3, -0.8061077985790659);

    // change can id
    gyro = new Pigeon2(5);
    gyroConfig.MountPose.withMountPoseYaw(0.0);
    gyroConfig.MountPose.withMountPosePitch(0.0);
    gyroConfig.MountPose.withMountPoseRoll(0.0);
    gyro.getConfigurator().apply(gyroConfig);
    gyro.reset();

    poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, getRotation(), modulePositions, new Pose2d());

    SmartDashboard.putData(
        "Swerve",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> frontLeftModule.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> frontLeftModule.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> frontRightModule.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () -> frontRightModule.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> backLeftModule.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> backLeftModule.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> backRightModule.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> backRightModule.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
        });
    SmartDashboard.putData("Field", field);
  }
  /** Gets current rotation of the robot. */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }
  /** Resets robot gyro and pose rotation to 0. */
  public void resetGyro() {
    gyro.reset();
    poseEstimator.resetRotation(new Rotation2d());
  }
  /** Drives the robot by translating fieldSpeeds into swerve module signals. */
  public void drive(ChassisSpeeds fieldSpeeds) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getRotation());

    if (Constants.currentMode.toString() == "SIM") {
      gyro.setYaw(
          gyro.getYaw().getValueAsDouble()
              + (Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond) / 50));
    }

    double modX = chassisSpeeds.vxMetersPerSecond / 50;
    double modY = chassisSpeeds.vyMetersPerSecond / 50;
    Rotation2d modOmega = Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond / 50);

    Transform2d simDelta = new Transform2d(modX, modY, modOmega);

    simPose = simPose.plus(simDelta);

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

    frontLeftModule.setState(moduleStates[0]);
    frontRightModule.setState(moduleStates[1]);
    backLeftModule.setState(moduleStates[2]);
    backRightModule.setState(moduleStates[3]);
  }
  /** Gets current pose estimator pose or sim pose. */
  public Pose2d getPose() {
    switch (Constants.currentMode) {
      case REAL:
        return poseEstimator.getEstimatedPosition();
      case SIM:
        return simPose;
      default:
        return new Pose2d();
    }
  }
  /** Updates pose estimator with swerve module positions. */
  public void updateRobotPose() {
    modulePositions[0] = frontLeftModule.getPosition();
    modulePositions[1] = frontRightModule.getPosition();
    modulePositions[2] = backLeftModule.getPosition();
    modulePositions[3] = backRightModule.getPosition();

    poseEstimator.update(getRotation(), modulePositions);
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
    frontLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightModule.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  /** Gets positions of swerve modules in relation to the center of the robot. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    };
  }
  /** Gets current chassis speeds using kinematic or sent chassis speeds in sim. */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  /** Gets swerve module states. */
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
  /** Sees if the robot is facing the Hub, with a tolerance. */
  public boolean isRobotFacingHub() {
    return isRobotFacingHub(getHubPosition());
  }
  /** Sees if the robot is facing a Translation2d, with a tolerance. */
  public boolean isRobotFacingHub(Translation2d position) {

    double currentAngle = getPose().getRotation().getRadians();
    double targetAngle = getRotationToHub(position).getRadians();
    double delta = targetAngle - currentAngle;
    delta = ((delta + Math.PI) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI) - Math.PI;

    return (Math.abs(delta) < Math.toRadians(angleTolerance));
  }
  /** Sees if the robot is facing the Virtual Hub, with a tolerance. */
  public boolean isRobotFacingVirtualHub() {
    return isRobotFacingHub(getVirtualHubPosition());
  }
  /** Sees if the robot is within distance to shoot at the Hub. */
  public boolean canShootAtHub() {
    return canShootAtHub(getHubPosition(), 2, 5);
  }
  /** Sees if the robot is within distance to shoot at a Pose2d. */
  public boolean canShootAtHub(Translation2d position, double min, double max) {
    return (Math.abs(getDistanceFromHub(position)) > min
        && Math.abs(getDistanceFromHub(position)) < max);
  }
  /** Sees if the robot is within distance to shoot at the Virtual Hub. */
  public boolean canShootAtVirtualHub() {
    return canShootAtHub(getVirtualHubPosition(), 2, 5);
  }

  public void periodic() {
    field.setRobotPose(getPose());
    // virtualHub.setPose(
    //     getVirtualHubPosition().getX(), getVirtualHubPosition().getY(), new Rotation2d());

    // SmartDashboard.putBoolean("IsRobotFacingVirtualHub", isRobotFacingVirtualHub());
    // SmartDashboard.putBoolean("canShootAtVirtualHub", canShootAtVirtualHub());

    updateRobotPose();
  }
}
