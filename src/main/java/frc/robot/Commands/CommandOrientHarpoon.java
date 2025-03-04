package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints.kHarpoonPosition;
import frc.robot.subsystems.Harpoon;

public class CommandOrientHarpoon extends Command {
    private final Harpoon m_orientHarpoon;
    private final kHarpoonPosition m_harpoonPosition;

    public CommandOrientHarpoon(Harpoon harpoon, kHarpoonPosition position) {
        m_orientHarpoon = harpoon;
        m_harpoonPosition = position;
        addRequirements(m_orientHarpoon);

    }

    @Override 
    public void execute() {
        m_orientHarpoon.OrientHarpoon(m_harpoonPosition.Degrees);
    }  

    @Override public boolean isFinished() {
        return true;
    }
}
