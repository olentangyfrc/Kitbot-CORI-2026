package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
    public static final double DEADBAND = 0.1;

    private DriveCommands() {}
    
    public static Command joystickDrive(Drive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        return Commands.run(
            () -> {
                double xspeed = MathUtil.applyDeadband(x.getAsDouble(), DEADBAND);
                double yspeed = MathUtil.applyDeadband(y.getAsDouble(), DEADBAND);
                double omegaspeed = MathUtil.applyDeadband(omega.getAsDouble(), DEADBAND);

                Math.copySign(xspeed * xspeed, xspeed);
                Math.copySign(yspeed * yspeed, yspeed);
                Math.copySign(omegaspeed * omegaspeed, omegaspeed);

                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xspeed, yspeed, omegaspeed);

                drive.drive(chassisSpeeds);
            },
        drive);
    }
}
