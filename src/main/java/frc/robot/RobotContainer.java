package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.output.OutputCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator.ElevatorCommands;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.output.OutputCommands;
import frc.robot.subsystems.output.OutputConstants.OutputState;
import frc.robot.subsystems.output.Output;
import frc.robot.subsystems.output.OutputConstants.OutputState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

@Logged
public class RobotContainer {
    public static final Swerve SWERVE = Swerve.getInstance();
    public static final Output OUTPUT = new Output();
    public static final Elevator ELEVATOR = new Elevator();

    private final CommandXboxController driverController1 = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController driverController2 = new CommandXboxController(OIConstants.kOperatorControllerPort);


    public RobotContainer() {
        SWERVE.setDefaultCommand(SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                () -> -driverController1.getLeftY()/2.5,
                () -> -driverController1.getLeftX()/2.5 ,
                () -> -driverController1.getRightX() /2
        ));

        configureButtonBindings();
        ELEVATOR.setDefaultCommand(ElevatorCommands.moveToHeight(ElevatorState.L1));
        OUTPUT.setDefaultCommand(OutputCommands.output(OutputState.STOP));
    }

    private void configureButtonBindings() {
        driverController2.a().and(driverController2.povDown()).whileTrue(OutputCommands.output(OutputState.L4));
        driverController2.a().and(driverController2.povUp().or(driverController2.povRight())).whileTrue(OutputCommands.output(OutputState.L2L3));
        driverController2.a().and(driverController2.povDown()).whileTrue(OutputCommands.output(OutputState.L4));
        driverController2.a().and(driverController2.povCenter()).whileTrue(OutputCommands.output(OutputState.L1));
        driverController2.y().whileTrue(OutputCommands.intake());
        driverController2.b().whileTrue(OutputCommands.output(OutputState.TAKEOUT));


        driverController2.povUp().and(driverController2.povDown().negate()).whileTrue(ElevatorCommands.moveToHeight(ElevatorState.L2));
        driverController2.povRight().and(driverController2.povDown().negate()).whileTrue(ElevatorCommands.moveToHeight(ElevatorState.L3));
        driverController2.povDown().whileTrue(ElevatorCommands.moveToHeight(ElevatorState.L4));

        driverController1.x().onTrue(SwerveCommands.getResetHeadingCommand());
    }

    @Logged(name = "Swerve")
    public Swerve logSwerve() {
        return SWERVE;
    }

    @Logged(name = "Output")
    public Output logOutput() {
        return OUTPUT;
    }

    @Logged(name = "Elevator")
    public Elevator logElevator() {
        return ELEVATOR;
    }

    public Command getAutonomousCommand() {
        return(SwerveCommands.getResetHeadingCommand().andThen(SwerveCommands.getFieldRelativeClosedLoopSupplierDriveCommand(
            ()->0.3,
            ()->0,
            ()->0
    )).withTimeout(3)).andThen(ElevatorCommands.moveToHeight(ElevatorState.L4).
        raceWith(new WaitCommand (2.6).andThen(OutputCommands.output(OutputState.L4).withTimeout(2).
        andThen(SwerveCommands.setHeadingCommand(Rotation2d.fromDegrees(180))))));
        
    }
}
