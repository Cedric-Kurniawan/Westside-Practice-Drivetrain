package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Harpoon {
    private final SparkMax m_reelSpark;
    private final SparkMax m_deploySpark;

    public Harpoon(int reelCANId, int deployCANId) {
        m_reelSpark = new SparkMax(reelCANId, MotorType.kBrushless);
        m_deploySpark = new SparkMax(deployCANId, MotorType.kBrushless);

        // m_reelSpark.configure(IdleMode.kBrake, ResetMode.kNoResetSafeParameters);
        // m_deploySpark.configure(IdleMode.kBrake);
    }


}