package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        

        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                        / ModuleConstants.kDrivingMotorReduction;
                double turningFactor = 2 * Math.PI;
                double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                drivingConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);
                drivingConfig.encoder
                        .positionConversionFactor(drivingFactor) // meters
                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                drivingConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.04, 0, 0)
                        .velocityFF(drivingVelocityFeedForward)
                        .outputRange(-1, 1);

                turningConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);
                turningConfig.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                        .inverted(true)
                        .positionConversionFactor(turningFactor) // radians
                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                turningConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);      
        }
    }
    public static final class ElevatorConfigs{
        public static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        static{
                elevatorConfig.Slot0.kP = 0;
                elevatorConfig.Slot0.kI = 0;
                elevatorConfig.Slot0.kD = 0; 
                elevatorConfig.Slot0.GravityType  = GravityTypeValue.Elevator_Static;
                elevatorConfig.Slot0.kA = 0;
                elevatorConfig.Slot0.kV = 0;
                elevatorConfig.Slot0.kG = 0;
                elevatorConfig.Slot0.kS = 0;
                elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 100; // Units in RPS
                elevatorConfig.MotionMagic.MotionMagicAcceleration = 100; // Units in RPS/s
                elevatorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                
        }

    }
}
