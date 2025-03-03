package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Setpoints.kLiftPosition;
import frc.robot.subsystems.Harpoon;

public class CommandYeetHarpoon extends Command {
    private final Harpoon m_harpoon;

    public CommandYeetHarpoon(Harpoon harpoon) {
        m_harpoon = harpoon;
        addRequirements(m_harpoon);
    }

    @Override
    public void execute() {
        
        m_harpoon.PositionHarpoon(0);
    }

}
