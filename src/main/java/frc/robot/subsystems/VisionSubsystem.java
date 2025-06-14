package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;

public class VisionSubsystem {
    public PhotonCamera cameraL;
    public PhotonCamera cameraR;
    public final List<Integer> REEF_TAGS;

    public final static class VisionConstants{
        //Pulled From reefscape-25 Discobots Repo
        public static final double kTargetX = 0.38;
        public static final double kTargetY = 0.0;
        public static final double kTargetYTol = 0.01;
        public static final double kTargetXTol = 0.01;

    }
    
    VisionSubsystem(){
    cameraL = new PhotonCamera("FrontL");
    cameraR = new PhotonCamera("FrontR");
    REEF_TAGS =  Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    }
}
