package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SoundManager {

    private boolean goldFound;      // Sound file present flags
    private boolean silverFound;
    private boolean autoCompleteFound;
    private boolean robotInitFound;

    private int silverSoundID;
    private int goldSoundID;
    private int autoCompleteId;
    private int robotInitID;

    HardwareMap hardwareMap;


    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        // Determine Resource IDs for sounds built into the RC application.
        silverSoundID  = hardwareMap.appContext.getResources().getIdentifier("silver", "raw", hardwareMap.appContext.getPackageName());
        goldSoundID    = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());
        autoCompleteId = hardwareMap.appContext.getResources().getIdentifier("auto_complete",   "raw", hardwareMap.appContext.getPackageName());
        robotInitID    = hardwareMap.appContext.getResources().getIdentifier("robot_init",   "raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (goldSoundID != 0)
            goldFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, goldSoundID);

        if (silverSoundID != 0)
            silverFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, silverSoundID);

        if (autoCompleteId != 0)
            autoCompleteFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, autoCompleteId);

        if (robotInitID != 0)
            robotInitFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, robotInitID);
    }

    public void playSilver(){
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
    }

    public void playGold(){
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
    }

    public void playAutoComplete(){
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, autoCompleteId);
    }

    public void playRobotInit(){
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, robotInitID);
    }

}
