package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.NeoMotorConstants;

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
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
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

    public static final class DefaultNeo {
            public static final SparkMaxConfig neoConfig = new SparkMaxConfig();
            public static final SparkMaxConfig neoArmConfig = new SparkMaxConfig();
            public static final SparkMaxConfig neoHarpoonConfig = new SparkMaxConfig();
            public static final SparkMaxConfig neo550Config = new SparkMaxConfig();
            public static final SparkMaxConfig liftLeftConfig = new SparkMaxConfig();
            public static final SparkMaxConfig liftRightConfig = new SparkMaxConfig();
            
            static {
                double neoVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                // Standard NEO Config
                neoConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);  
                
                // Arm NEO Configs
                neoArmConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50);    
                
                neoArmConfig.absoluteEncoder
                        .inverted(false)
                        .positionConversionFactor(10); // Degrees
                
                neoArmConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(0.3, 0, 0)
                        .velocityFF(0)
                        .outputRange(-0.25, 0.25)
                        .positionWrappingEnabled(false);

                // neoArmConfig.closedLoop.maxMotion
                //         .maxAcceleration(1500)
                //         .maxVelocity(neoVelocityFeedForward)
                //         .allowedClosedLoopError(0.05);
                // Harpoon config (mostly for the PID garbo)
                neoHarpoonConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);    
                
                neoHarpoonConfig.absoluteEncoder
                        .positionConversionFactor(360); // Degrees
                
                neoHarpoonConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(0.04, 0, 0)
                        .velocityFF(neoVelocityFeedForward)
                        .outputRange(-.2, .2);

                neoHarpoonConfig.closedLoop.maxMotion
                        .maxAcceleration(1500)
                        .maxVelocity(NeoMotorConstants.kFreeSpeedRpm)
                        .allowedClosedLoopError(0.5);
                        
                neo550Config
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(25);
                        
                liftLeftConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(50)
                        .inverted(true);

                liftLeftConfig.absoluteEncoder
                        .positionConversionFactor(10); // Degrees
                        
                liftLeftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(.5, 0, 0.019)
                        .velocityFF(0.002)
                        .outputRange(-.6, .8);
                        
                liftLeftConfig.closedLoop.maxMotion
                        .maxAcceleration(3000)
                        .maxVelocity(5000);
                        
                liftRightConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80)
                        .follow(Constants.SystemConstants.kLeftLiftCanId, true);

        }
    }
}
