package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars.Throttles;

public class CommandSetCreep extends Command {
    private final double m_throttle;

    public CommandSetCreep(double throttle){
        m_throttle = throttle;
    
    }
    
    @Override 
    public void execute(){
        Throttles.kDriveThrottle = m_throttle;
    }

}
