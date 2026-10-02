package frc.robot.subsystems.shooter;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
  private final double intakeMaxVelocity = 2000;
  private final double indexerMaxVelocity = 6;
  private final double shooterVelocityTolerance = 120;
  private double targetVelocity = 0;
  private double shooterManualVelocity = 3000;

  private ShooterUtil.ShooterParameters shooterParams = new ShooterUtil.ShooterParameters(0);

  private TalonFXConfiguration shooterConfig;
  private SparkMaxConfig indexerConfig;

  private final int shooterCanId = 40; // change later
  private TalonFX shooterMotor;

  private final int indexerCanId = 21; // change later
  private SparkMax indexerMotor;

  public Shooter() {
    shooterMotor = new TalonFX(shooterCanId, "rio");
    indexerMotor = new SparkMax(indexerCanId, MotorType.kBrushless);
    init();
  }

  public void init() {
    shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // tune
    shooterConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    shooterConfig.Slot0.kP = 0.15711;
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kD = 0;
    shooterConfig.Slot0.kS = 0.12386;
    shooterConfig.Slot0.kV = 0.11439;
    shooterConfig.Slot0.kA = 0.025192;
    shooterMotor.getConfigurator().apply(shooterConfig, 0.25);

    indexerConfig = new SparkMaxConfig();
    indexerConfig.idleMode(IdleMode.kBrake);
    indexerConfig.inverted(false); // test this
    // tune
    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  /**
   * Sets shooter motor speed in RPM.
   *
   * @param speed
   */
  public void setShooterSpeed(double speed) {
    targetVelocity = speed;
  }
  /**
   * Sets indexer motor speed in RPM.
   *
   * @param speed
   */
  public void setIndexerSpeed(double speed) {
    indexerMotor.setVoltage(speed);
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
    targetVelocity = 0;
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

  public void shoot(Double speed) {
    setShooterSpeed(speed);
  }

  /**
   * Sets shooter speed to correct value based off distance. Uses an interpolation table to
   * calculate speed.
   *
   * @param distanceMeters
   */
  public void shootForHub(double distanceMeters) {
    shooterParams = ShooterUtil.getInterpolatedValues(distanceMeters);

    // min to not go over max
    setShooterSpeed(Math.min(shooterParams.shooterRpm(), shootMaxVelocity));
  }
  /** Runs shooter motor at a slower speed. */
  public void spinUp() {
    setShooterSpeed(spinUpVelocity);
  }
  /** Runs the shooter and the indexer to intake. */
  public void intake() {
    setShooterSpeed(intakeMaxVelocity);
    setIndexerSpeed(-indexerMaxVelocity);
  }
  /** Runs the indexer to shoot */
  public void indexerShoot() {
    setIndexerSpeed(indexerMaxVelocity);
  }
  /**
   * Checks if the shooter's current speed is within tolerance of the target speed.
   *
   * @return True if shooter speed is within tolerance.
   */
  public boolean isShooterAtSpeed() {
    return shooterMotor.getClosedLoopError().getValueAsDouble() * 60 <= shooterVelocityTolerance;
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
    return targetVelocity;
  }

  public void setShooterManualSpeed(double speed) {
    shooterManualVelocity = speed;
  }

  public double getShooterManualSpeed() {
    return shooterManualVelocity;
  }

  public void periodic() {
    SmartDashboard.putNumber("shooterRps", getShooterSpeed());
    SmartDashboard.putNumber("shooterTargetRps", getShooterTargetSpeed());
    if (targetVelocity > 0) {
      shooterMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(targetVelocity / 60));
    }
  }
}
