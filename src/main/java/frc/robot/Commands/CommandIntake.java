package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.Vars.Throttles;

public class CommandIntake extends Command {
    private final AlgaeArm m_algaeArm;
    private final CoralArm m_coralArm;
    
    public CommandIntake(AlgaeArm algaeArm, CoralArm coralArm) {
        m_coralArm = coralArm;
        m_algaeArm = algaeArm;

    addRequirements(m_algaeArm, m_coralArm);
    }

    @Override
    public void execute() {
        m_algaeArm.IntakeAlgae(1 * Throttles.kAlgaeIntakeThrottle);
        m_coralArm.IntakeCoral(-1 * Throttles.kCoralIntakeThrottle);
    }

    @Override
    public void end(boolean interrupted) {
        m_algaeArm.IntakeAlgae(0);
        m_coralArm.IntakeCoral(0);
    }
    
}
