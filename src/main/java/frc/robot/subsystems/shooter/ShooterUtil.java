package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterUtil {
  public record ShooterParameters(double shooterRpm) {}

  public static InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

  static {
    // need to tune
    // distance (meters), rpm
    double[][] highCeilingData = {
      {0, 500},
      {1, 1000},
      {2, 1500}
    };

    for (double[] point : highCeilingData) {
      double distanceMeters = point[0];
      double rpm = point[1];

      rpmMap.put(distanceMeters, rpm);
    }
  }

  private ShooterUtil() {}

  public static ShooterParameters getInterpolatedValues(double distanceMeters) {
    double rpm = Math.floor(rpmMap.get(distanceMeters));
    return new ShooterParameters(rpm);
  }
}
