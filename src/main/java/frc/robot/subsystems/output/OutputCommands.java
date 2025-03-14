package frc.robot.subsystems.output;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.output.OutputConstants.OutputState;

public class OutputCommands {
    public static Command output(OutputState state) {
        return new RunCommand(
                () -> {
                  
                        RobotContainer.OUTPUT.setMotorOutput(state);
                   
                },
                RobotContainer.OUTPUT
        );
    }
    public static Command intake(){
        return new RunCommand(
            () -> {
              
                    RobotContainer.OUTPUT.setMotorOutput(OutputConstants.OutputState.INTAKE);
               
            },
            RobotContainer.OUTPUT
    ).until(RobotContainer.OUTPUT::hasCoral).andThen(output(OutputState.INTAKE).withTimeout(0.16));
    }
}
