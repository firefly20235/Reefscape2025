package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator.ElevatorCommands;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.output.OutputCommands;
import frc.robot.subsystems.output.OutputConstants.OutputState;
import frc.robot.subsystems.output.Output;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

@Logged
public class RobotContainer {
    public static final Swerve SWERVE = Swerve.getInstance();
    public static final Output OUTPUT = new Output();
    public static final Elevator ELEVATOR = new Elevator();

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        SWERVE.setDefaultCommand(SwerveCommands.getFieldRelativeOpenLoopSupplierDriveCommand(
                () -> -driverController.getLeftY() / 4,
                () -> -driverController.getLeftX() / 4,
                () -> -driverController.getRightX() / 8
        ));

        configureButtonBindings();
        SmartDashboard.putData(ELEVATOR);
        
        ELEVATOR.setDefaultCommand(ElevatorCommands.moveToHeight(ElevatorState.L1));
    }

    private void configureButtonBindings() {
        driverController.y().whileTrue(OutputCommands.output(OutputState.L1));
        driverController.x().whileTrue(OutputCommands.output(OutputState.L2L3));
        driverController.a().whileTrue(OutputCommands.output(OutputState.L4));
        driverController.b().whileTrue(OutputCommands.output(OutputState.STOP));

        driverController.povUp().whileTrue(ElevatorCommands.moveToHeight(ElevatorState.L2));
        driverController.povRight().whileTrue(ElevatorCommands.moveToHeight(ElevatorState.L3));
        driverController.povDown().whileTrue(ElevatorCommands.moveToHeight(ElevatorState.L4));
                               
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
}
