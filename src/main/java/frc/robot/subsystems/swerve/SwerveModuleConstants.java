package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;

public class SwerveModuleConstants {
    static final double DRIVE_GEAR_RATIO = 6.75;
    static final double WHEEL_DIAMETER_METERS = 0.1016;
    static final double MAX_THEORETICAL_SPEED_METERS_PER_SECOND = 4;
    private static final double VOLTAGE_COMP_SATURATION = 12;


    public static final int
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            REAR_LEFT_ID = 2,
            REAR_RIGHT_ID = 3;
    private static final int
            FRONT_LEFT_DRIVE_MOTOR_ID = 8,
            FRONT_RIGHT_DRIVE_MOTOR_ID = 6,
            REAR_LEFT_DRIVE_MOTOR_ID = 2,
            REAR_RIGHT_DRIVE_MOTOR_ID = 4;
    private static final boolean DRIVE_MOTOR_INVERTED = true;
    private static final double
            DRIVE_OPEN_LOOP_RAMP_RATE = 0.2,
            DRIVE_CLOSED_LOOP_RAMP_RATE = 0.4;
    static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(0.0001, 0.0001, 0.0001);
    private static final TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new TalonFX(
                    FRONT_LEFT_DRIVE_MOTOR_ID
            ),
            FRONT_RIGHT_DRIVE_MOTOR = new TalonFX(
                    FRONT_RIGHT_DRIVE_MOTOR_ID
            ),
            REAR_LEFT_DRIVE_MOTOR = new TalonFX(
                    REAR_LEFT_DRIVE_MOTOR_ID
            ),
            REAR_RIGHT_DRIVE_MOTOR = new TalonFX(
                    REAR_RIGHT_DRIVE_MOTOR_ID
            );

    private static final int
            FRONT_LEFT_STEER_MOTOR_ID = 7,
            FRONT_RIGHT_STEER_MOTOR_ID = 5,
            REAR_LEFT_STEER_MOTOR_ID = 9,
            REAR_RIGHT_STEER_MOTOR_ID = 3;
    private static final boolean STEER_MOTOR_INVERTED = true;
    private static final int
        FRONT_LEFT_STEER_ENCODER_ID = 10,
        FRONT_RIGHT_STEER_ENCODER_ID = 11,
        REAR_LEFT_STEER_ENCODER_ID = 12,
        REAR_RIGHT_STEER_ENCODER_ID = 13;
    private static final double
            STEER_MOTOR_P = 0.024,//3/300d,  
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;

            
    private static final double
        FRONT_LEFT_STEER_ENCODER_OFFSET = 0.989990,
        FRONT_RIGHT_STEER_ENCODER_OFFSET = 0.141602,
        REAR_LEFT_STEER_ENCODER_OFFSET = 0.321777,
        REAR_RIGHT_STEER_ENCODER_OFFSET = 0.251709;
    private static final SparkMax
            FRONT_LEFT_STEER_MOTOR = new SparkMax(
                    FRONT_LEFT_STEER_MOTOR_ID,
                    SparkMax.MotorType.kBrushless
            ),
            FRONT_RIGHT_STEER_MOTOR = new SparkMax(
                    FRONT_RIGHT_STEER_MOTOR_ID,
                    SparkMax.MotorType.kBrushless
            ),
            REAR_LEFT_STEER_MOTOR = new SparkMax(
                    REAR_LEFT_STEER_MOTOR_ID,
                    SparkMax.MotorType.kBrushless
            ),
            REAR_RIGHT_STEER_MOTOR = new SparkMax(
                    REAR_RIGHT_STEER_MOTOR_ID,
                    SparkMax.MotorType.kBrushless
            );
    private static final CANcoder
            FRONT_LEFT_STEER_ENCODER = new CANcoder(
                    FRONT_LEFT_STEER_ENCODER_ID
            ),
            FRONT_RIGHT_STEER_ENCODER = new CANcoder(
                    FRONT_RIGHT_STEER_ENCODER_ID
            ),
            REAR_LEFT_STEER_ENCODER = new CANcoder(
                    REAR_LEFT_STEER_ENCODER_ID
            ),
            REAR_RIGHT_STEER_ENCODER = new CANcoder(
                    REAR_RIGHT_STEER_ENCODER_ID
            );
    private static final SwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_STEER_MOTOR,
                    FRONT_LEFT_STEER_ENCODER,
                    FRONT_LEFT_STEER_ENCODER_OFFSET
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR,
                    FRONT_RIGHT_STEER_ENCODER,
                    FRONT_RIGHT_STEER_ENCODER_OFFSET
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_STEER_MOTOR,
                    REAR_LEFT_STEER_ENCODER,
                    REAR_LEFT_STEER_ENCODER_OFFSET
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_STEER_MOTOR,
                    REAR_RIGHT_STEER_ENCODER,
                    REAR_RIGHT_STEER_ENCODER_OFFSET
            );

    private static final Translation2d
            FRONT_LEFT_MODULE_LOCATION = new Translation2d(
                    SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            FRONT_RIGHT_MODULE_LOCATION = new Translation2d(
                    SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_LEFT_MODULE_LOCATION = new Translation2d(
                    -SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_RIGHT_MODULE_LOCATION = new Translation2d(
                    -SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE,
                    -SwerveConstants.DISTANCE_FROM_CENTER_OF_BASE
            );

    final TalonFX driveMotor;
    final SparkMax steerMotor;
    final CANcoder steerEncoder;
    final double encoderOffset;

    public SwerveModuleConstants(TalonFX driveMotor, SparkMax steerMotor, CANcoder steerEncoder, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;
        this.encoderOffset = encoderOffset;

        initialConfig();
    }

    private void initialConfig() {
        var driveConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = DRIVE_OPEN_LOOP_RAMP_RATE;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DRIVE_CLOSED_LOOP_RAMP_RATE;

        driveMotor.getConfigurator().apply(driveConfig);

        var steerConfig = new SparkMaxConfig();

        steerConfig.inverted(STEER_MOTOR_INVERTED);
        steerConfig.closedLoop.pid(STEER_MOTOR_P, STEER_MOTOR_I, STEER_MOTOR_D);
        steerConfig.smartCurrentLimit(10);
        steerConfig.voltageCompensation(VOLTAGE_COMP_SATURATION);
        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.closedLoop.positionWrappingMinInput(0);
        steerConfig.closedLoop.positionWrappingMaxInput(Units.Rotation.of(1).in(Units.Degrees));
        steerConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        steerConfig.encoder.positionConversionFactor(7/150f*360);


        steerMotor.configure(steerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        steerMotor.getEncoder().setPosition(steerEncoder.getAbsolutePosition().getValue().minus(Units.Rotations.of(encoderOffset)).in(edu.wpi.first.units.Units.Degrees));
    }

    enum TestingSwerveModules {
        FRONT_LEFT(0, FRONT_LEFT_SWERVE_MODULE_CONSTANTS, FRONT_LEFT_MODULE_LOCATION),
        FRONT_RIGHT(1, FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_LOCATION),
        REAR_LEFT(2, REAR_LEFT_SWERVE_MODULE_CONSTANTS, REAR_LEFT_MODULE_LOCATION),
        REAR_RIGHT(3, REAR_RIGHT_SWERVE_MODULE_CONSTANTS, REAR_RIGHT_MODULE_LOCATION);

        final int id;
        final SwerveModuleConstants swerveModuleConstants;
        final Translation2d location;

        TestingSwerveModules(int id, SwerveModuleConstants swerveModuleConstants, Translation2d location) {
            this.id = id;
            this.swerveModuleConstants = swerveModuleConstants;
            this.location = location;
        }

        static TestingSwerveModules fromId(int id) {
            for (TestingSwerveModules module : values()) {
                if (module.id == id) {
                    return module;
                }
            }

            throw new IndexOutOfBoundsException("No module with id " + id + " exists");
        }
    }
}

