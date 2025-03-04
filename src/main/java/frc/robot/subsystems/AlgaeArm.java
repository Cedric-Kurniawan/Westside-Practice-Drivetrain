package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.Setpoints.kLiftPosition;

public class AlgaeArm extends SubsystemBase {
  private final SparkMax m_armSpark;
    private final VictorSP m_intakeVictor;
    private final SparkClosedLoopController m_armClosedLoopController;

    public AlgaeArm(int intakeCANId, int armCANId) {
        m_intakeVictor = new VictorSP(intakeCANId);
        m_armSpark = new SparkMax(armCANId, MotorType.kBrushless);


        // m_intakeVictor.configure(Configs.DefaultNeo.neoConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_armSpark.configure(Configs.DefaultNeo.neoArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armClosedLoopController = m_armSpark.getClosedLoopController();
    } 


     /**
     * Tell the manipulator to gawk gawk the coral in.
     * 
     * @param setpoint The desired value from 0 to 1.
     */
    public void IntakeAlgae(double setpoint) {
        m_intakeVictor.set(setpoint);
    }
    
    /**
     * Around around around the world
     * @param targetPosition The target in degrees for arm, "good enough" ~ Jesse
     */
    public void AlgaeArmPosition(kLiftPosition targetPosition) {
        m_armClosedLoopController.setReference(targetPosition.CoralPoseDeg, ControlType.kPosition);
   }
}
