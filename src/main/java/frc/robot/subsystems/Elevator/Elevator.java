package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorState;

@Logged
public class Elevator extends SubsystemBase {

    public Elevator() {
        SparkMaxConfig motorconfig = new SparkMaxConfig();
        motorconfig.encoder.positionConversionFactor(ElevatorConstants.ENCODER_TO_METERS);
        motorconfig.encoder.velocityConversionFactor(ElevatorConstants.ENCODER_TO_METERS / 60);
        ElevatorConstants.elevatorMotor.configure(motorconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        new Trigger(this::isAtBottom).onTrue(new InstantCommand(this::resetEncoder));
    }

    public void initController(ElevatorState state) {
        ElevatorConstants.controller.setGoal(state.height);
        ElevatorConstants.controller.reset(getCurrentPosition(), getCurrentVelocity());
    }

    public void runMotor() {
        if (isAtBottom()) {
            stop();
            resetEncoder(); 
        } else {
            double feedback = ElevatorConstants.controller.calculate(getCurrentPosition());
            var setpoint = ElevatorConstants.controller.getSetpoint();
            double feedforward = ElevatorConstants.FEEDFORWARD.calculate(setpoint.velocity);
            ElevatorConstants.elevatorMotor.setVoltage(feedback + feedforward);
        }
 }
    public void setTargetHeight(ElevatorState state) {
        if (state != ElevatorState.L1) {
            initController(state);
            runMotor();
        } else {
            stop(); // Move to L1 when button is released
        }
    }
    

    public void stop() {
        ElevatorConstants.elevatorMotor.set(0);
    }

    public double getCurrentPosition() {
        return ElevatorConstants.elevatorMotor.getEncoder().getPosition();
    }

    public double getCurrentVelocity() {
        return ElevatorConstants.elevatorMotor.getEncoder().getVelocity();
    }

    public boolean isAtBottom() {
        return ElevatorConstants.limitSwitch.get(); 
    }

    public void resetEncoder() {
        ElevatorConstants.elevatorMotor.getEncoder().setPosition(0);
    }
}
