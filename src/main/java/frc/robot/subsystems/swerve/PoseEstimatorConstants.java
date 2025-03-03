package frc.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimatorConstants {
    static final double GYRO_UPDATE_TIME_SECONDS = 0.020;
    static final Vector<N3>
            STATES_AMBIGUITY = VecBuilder.fill(0.005, 0.005, 0.00005),
            VISION_CALCULATIONS_AMBIGUITY = VecBuilder.fill(0.1, 0.05, Math.toRadians(720));
}

