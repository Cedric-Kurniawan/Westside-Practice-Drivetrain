package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Lift;

public class CommandMultiPosition extends Command {
    private final Lift m_lift;
    private final CoralArm m_coral;
    private final AlgaeArm m_algaeArm;
    private final kLiftPosition m_position;

    public CommandMultiPosition(Lift lift, CoralArm coralArm, AlgaeArm algaeArm, kLiftPosition position) {
        m_lift = lift;
        m_coral = coralArm;
        m_algaeArm = algaeArm;
        m_position = position;

        addRequirements(m_lift, m_coral, m_algaeArm);
    }

    @Override
    public void execute() {
        m_algaeArm.AlgaeArmPosition(m_position);
        m_coral.CoralArmPosition(m_position);
        m_lift.setLiftPosition(m_position);
    }

    @Override public boolean isFinished() {
        return true;
    }
}
