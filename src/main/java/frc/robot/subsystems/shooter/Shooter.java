package frc.robot.subsystems.shooter;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// shooter and intake are controlled by the same motor. indexer in same subsystem.
public class Shooter extends SubsystemBase {
  // tune
  private final double spinUpVelocity = 1500;
  private final double shootMaxVelocity = 3000;
  private final double intakeMaxVelocity = -2000;
  private final double indexerMaxVelocity = 500;

  private final double shooterVelocityTolerance = 25;

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
    shooterMotor.getConfigurator().apply(shooterConfig, 0.25);

    indexerConfig = new SparkMaxConfig();
    indexerConfig.idleMode(IdleMode.kBrake);
    indexerConfig.inverted(false); // test this
  }
  /**
   * Sets shooter motor speed in RPS.
   *
   * @param speed
   */
  public void setShooterSpeed(double speed) { // input rps
    shooterMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(speed));
  }
  /**
   * Sets indexer motor speed in RPS.
   *
   * @param speed
   */
  public void setIndexerSpeed(double speed) { // input rps
    indexerMotor.setVoltage(1);
  }
  /** Holds indexer motor in place, without rotation. */
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
    ShooterUtil.ShooterParameters params = ShooterUtil.getInterpolatedValues(distanceMeters);
    // min to not go over max
    setShooterSpeed(
        Math.min(params.shooterRpm() * 60, shootMaxVelocity)); // setShooterSpeed takes RPS
  }
  /** Runs shooter motor at a slower speed. */
  public void spinUp() {
    setShooterSpeed(spinUpVelocity);
  }
  /** Runs the shooter and the indexer to intake. */
  public void intake() {
    setShooterSpeed(1);
    setIndexerSpeed(1);
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
    return shooterMotor.getClosedLoopError().getValueAsDouble() < shooterVelocityTolerance;
  }
  /**
   * Gets the current shooter speed.
   *
   * @return Current shooter speed in RPS.
   */
  public double getShooterSpeed() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }
  /**
   * Gets the shooter target speed.
   *
   * @return Shooter target speed in RPS.
   */
  public double getShooterTargetSpeed() {
    return shooterMotor.getClosedLoopReference().getValueAsDouble();
  }

  public void periodic() {
    SmartDashboard.putNumber("shooter current rps", getShooterSpeed());
    SmartDashboard.putNumber("shooter target rps", getShooterTargetSpeed());
  }
}
