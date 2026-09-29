package frc.robot.subsystems.shooter;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// shooter and intake are controlled by the same motor. indexer in same subsystem.
public class Shooter extends SubsystemBase {
  // tune
  // all in rpm
  private final double spinUpVelocity = 1500;
  private final double shootMaxVelocity = 3000;
  private final double intakeMaxVelocity = -2000;
  private final double indexerMaxVelocity = 500;
  private final double shooterVelocityTolerance = 25;

  private ShooterUtil.ShooterParameters shooterParams = new ShooterUtil.ShooterParameters(0);

  private TalonFXConfiguration shooterConfig;
  private SparkMaxConfig indexerConfig;

  private final int shooterCanId = 40; // change later
  private TalonFX shooterMotor;

  private final int indexerCanId = 21; // change later
  private SparkMax indexerMotor;
  private SparkClosedLoopController indexerMotorController;

  public Shooter() {
    shooterMotor = new TalonFX(shooterCanId, "rio");
    indexerMotor = new SparkMax(indexerCanId, MotorType.kBrushless);
    init();
  }

  public void init() {
    shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // tune
    shooterConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    shooterConfig.Slot0.kP = 0.1;
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kD = 0;
    shooterConfig.Slot0.kS = 0;
    shooterConfig.Slot0.kV = 0;
    shooterConfig.Slot0.kA = 0;
    shooterMotor.getConfigurator().apply(shooterConfig, 0.25);

    indexerConfig = new SparkMaxConfig();
    indexerConfig.idleMode(IdleMode.kBrake);
    indexerConfig.inverted(false); // test this
    // tune
    indexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0.0005).i(0).d(0);
    indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexerMotorController = indexerMotor.getClosedLoopController();
  }
  /**
   * Sets shooter motor speed in RPM.
   *
   * @param speed
   */
  public void setShooterSpeed(double speed) {
    shooterMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(speed / 60));
  }
  /**
   * Sets indexer motor speed in RPM.
   *
   * @param speed
   */
  public void setIndexerSpeed(double speed) {
    indexerMotorController.setSetpoint(speed, ControlType.kVelocity);
  }
  /** Holds indexer in place. */
  public void holdIndexer() {
    setIndexerSpeed(0);
  }
  /** Cuts power to indexer motor and lets it freely rotate. */
  public void stopIndexer() {
    indexerMotor.setVoltage(0);
  }
  /** Cuts power to shooter motor and lets it freely rotate. */
  public void stopShooter() {
    shooterMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }
  /** Cuts power to shooter and indexer motor. */
  public void stop() {
    stopShooter();
    stopIndexer();
  }
  /** Runs shooter and indexer motor so fuel is ejected out of intake. */
  public void eject() {
    setShooterSpeed(-intakeMaxVelocity);
    setIndexerSpeed(-indexerMaxVelocity);
  }

  // public void shoot() {
  //   setShooterSpeed(shootMaxVelocity);
  // }

  /**
   * Sets shooter speed to correct value based off distance. Uses an interpolation table to
   * calculate speed.
   *
   * @param distanceMeters
   */
  public void shootForHub(double distanceMeters) {
    shooterParams = ShooterUtil.getInterpolatedValues(distanceMeters);

    // min to not go over max
    setShooterSpeed(
        Math.min(shooterParams.shooterRpm(), shootMaxVelocity));
  }
  /** Runs shooter motor at a slower speed. */
  public void spinUp() {
    setShooterSpeed(spinUpVelocity);
  }
  /** Runs the shooter and the indexer to intake. */
  public void intake() {
    setShooterSpeed(intakeMaxVelocity);
    setIndexerSpeed(indexerMaxVelocity);
  }
  /** Runs the indexer to shoot */
  public void indexerShoot() {
    setIndexerSpeed(-indexerMaxVelocity);
  }
  /**
   * Checks if the shooter's current speed is within tolerance of the target speed.
   *
   * @return True if shooter speed is within tolerance.
   */
  public boolean isShooterAtSpeed() {
    return (Math.abs(getShooterTargetSpeed() - getShooterSpeed()) < shooterVelocityTolerance);
  }
  /**
   * Gets the current shooter speed.
   *
   * @return Current shooter speed in RPM.
   */
  public double getShooterSpeed() {
    return shooterMotor.getVelocity().getValueAsDouble() * 60;
  }
  /**
   * Gets the shooters target speed.
   *
   * @return Shooter target speed in RPM.
   */
  public double getShooterTargetSpeed() {
    return shooterParams.shooterRpm();
  }

  public void periodic() {
    SmartDashboard.putNumber("shooterRps", getShooterSpeed());
    SmartDashboard.putNumber("shooterTargetRps", getShooterTargetSpeed());
  }
}
