package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class SwerveCommands {
    private static final Swerve SWERVE = RobotContainer.SWERVE;


    /**
     * Creates a command that locks the swerve and brakes it.
     * This should be used when you want to make it hard to move the swerve.
     *
     * @return the command
     */
    public static StartEndCommand getLockSwerveCommand() {
        return new StartEndCommand(SWERVE::lockSwerve, SWERVE::stop, SWERVE);
    }

    public static Command getResetHeadingCommand() {
        return new InstantCommand(()->SWERVE.setHeading(new Rotation2d())).ignoringDisable(true);
    }
    public static Command setHeadingCommand(Rotation2d offset) {
        return new InstantCommand(() -> SWERVE.setHeading(offset)).ignoringDisable(true);
    }

    /**
     * Creates a command that sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     * @return the command
     */
    public static InstantCommand getSetSwerveBrakeCommand(boolean brake) {
        return new InstantCommand(() -> SWERVE.setBrake(brake), SWERVE);
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the field's frame of reference, in closed loop mode.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static FunctionalCommand getFieldRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                () -> initializeDrive(),
                () -> fieldRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the robot's frame of reference, in closed loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static FunctionalCommand getSelfRelativeClosedLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                () -> initializeDrive(),
                () -> selfRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }


    /**
     * Creates a command that drives the swerve with the given velocities, relative to the robot's frame of reference, in open loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static FunctionalCommand getSelfRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                () -> initializeDrive(),
                () -> selfRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given velocities, relative to the field's frame of reference, in open loop mode.
     * All velocities are in percent output from -1 to 1.
     *
     * @param x     the target forwards velocity
     * @param y     the target leftwards velocity
     * @param theta the target theta velocity, CCW+
     * @return the command
     */
    public static FunctionalCommand getFieldRelativeOpenLoopSupplierDriveCommand(
            DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return new FunctionalCommand(
                SwerveCommands::initializeDrive,
                () -> fieldRelativeDrive(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()),
                (interrupted) -> stopDrive(),
                () -> false,
                SWERVE
        );
    }



    private static void initializePosePIDControllers(PIDController xPIDController, PIDController yPIDController, PIDController thetaPIDController, Pose2d targetPose) {
        setPosePIDControllersSetpoint(xPIDController, yPIDController, thetaPIDController, targetPose);
        xPIDController.reset();
        yPIDController.reset();
        thetaPIDController.reset();
    }

    private static void setPosePIDControllersSetpoint(PIDController xPIDController, PIDController yPIDController, PIDController thetaPIDController, Pose2d targetPose) {
        xPIDController.setSetpoint(targetPose.getTranslation().getX());
        yPIDController.setSetpoint(targetPose.getTranslation().getY());
        thetaPIDController.setSetpoint(targetPose.getRotation().getDegrees());
    }



    private static boolean isSwerveStill() {
        return SWERVE.getCurrentVelocity().vxMetersPerSecond < SWERVE.getTranslationVelocityTolerance() &&
                SWERVE.getCurrentVelocity().vyMetersPerSecond < SWERVE.getTranslationVelocityTolerance() &&
                SWERVE.getCurrentVelocity().omegaRadiansPerSecond < SWERVE.getRotationVelocityTolerance();
    }

    private static PIDController pidConstantsToController(PIDConstants pidConstants) {
        return new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
    }

    private static void initializeDrive() {
        SWERVE.setBrake(true);
    }



    private static void fieldRelativeDrive(double x, double y, double theta) {
        SWERVE.fieldRelativeDrive(
                getDriveTranslation(x, y),
                getDriveRotation(theta)
        );
    }

    private static void selfRelativeDrive(double x, double y, double theta) {
        SWERVE.selfRelativeDrive(
                getDriveTranslation(x, y),
                getDriveRotation(theta)
        );
    }

    private static Rotation2d getDriveRotation(double rotPower) {
        return new Rotation2d(rotPower * SWERVE.getMaxRotationalSpeedRadiansPerSecond());
    }

    private static Translation2d getDriveTranslation(double x, double y) {
        return new Translation2d(
                x * SWERVE.getMaxSpeedMetersPerSecond(),
                y * SWERVE.getMaxSpeedMetersPerSecond()
        );
    }

    private static void stopDrive() {
        SWERVE.stop();
        SWERVE.setBrake(false);
    }
}
