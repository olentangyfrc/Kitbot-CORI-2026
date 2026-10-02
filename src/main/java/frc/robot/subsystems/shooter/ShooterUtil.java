package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterUtil {
  public record ShooterParameters(double shooterRpm) {}

  public static InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

  static {
    // need to tune
    // distance (meters), rpm
    double[][] highCeilingData = {
      {0, 0},
      {1, 500},
      {2, 750},
      {3, 1250},
      {4, 2000},
      {5, 3000},
    };

    for (double[] point : highCeilingData) {
      double distanceMeters = point[0];
      double rpm = point[1];

      rpmMap.put(distanceMeters, rpm);
    }
  }

  private ShooterUtil() {}
  /**
   * Gets shooter RPM based on distance to the hub using an interpolation table.
   *
   * @param distanceMeters
   * @return Shooter speed in RPM.
   */
  public static ShooterParameters getInterpolatedValues(double distanceMeters) {
    double rpm = Math.floor(rpmMap.get(distanceMeters));
    return new ShooterParameters(rpm);
  }
}
