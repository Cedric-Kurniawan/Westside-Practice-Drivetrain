package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Setpoints.kLiftPosition;

public class Lift extends SubsystemBase {
    private final SparkMax m_liftLeftSpark;
    private final SparkMax m_liftRightSpark;

    public Lift(int LeftLiftCanId, int RightLiftCanId) {
        m_liftLeftSpark = new SparkMax(LeftLiftCanId, MotorType.kBrushless);
        m_liftRightSpark = new SparkMax(RightLiftCanId, MotorType.kBrushless);

        m_liftLeftSpark.configure(
            Configs.DefaultNeo.liftLeftConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
        m_liftRightSpark.configure(
            Configs.DefaultNeo.liftRightConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public kLiftPosition getLiftPosition() {
            return null;
    }
}
