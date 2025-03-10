package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class OutputConstants {
     public static final int BEAM_BREAKER_PORT = 1; 
    public static final DigitalInput beamBreaker = new DigitalInput(BEAM_BREAKER_PORT);
     public enum OutputState {
        L4(OUT_POWER,L4_TOP_POWER),
        L2L3 (-ANGLE_POWER,-OUT_POWER),
        STOP (0, 0),
        L1 (-L1_TOP_MOTOR_POWER, -OUT_POWER),
        INTAKE(-INTAKE_DOWN,-INTAKE_UP),
        TAKEOUT(ANGLE_POWER,OUT_POWER);
        
        double upPower;
        double downPower;
        OutputState(double upPower, double downPower) {
            this.upPower = upPower;
            this.downPower = downPower;
        }
    }  



     static final SparkMax up = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
    static final SparkMax down = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);
    static final double ANGLE_POWER = 0.08
     ,OUT_POWER= -0.3
     ,L1_TOP_MOTOR_POWER = 0.12
     , L4_TOP_POWER=0.1
     ,INTAKE_UP=-0.4
     ,INTAKE_DOWN=0.1;


}

