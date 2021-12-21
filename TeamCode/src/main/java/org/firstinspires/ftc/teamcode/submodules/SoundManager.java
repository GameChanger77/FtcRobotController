package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SoundManager {

    private boolean goldFound;
    private boolean silverFound;
    private boolean autoCompleteFound;
    private boolean robotInitFound;
    private boolean trainingWheelsFound;
    private boolean adminOverrideFound;
    private boolean autoAlignFound;
    private boolean rickrollFound;

    private int silverSoundID;
    private int goldSoundID;
    private int autoCompleteId;
    private int robotInitID;
    private int trainingWheelsID;
    private int adminOverrideID;
    private int autoAlignID;
    private int rickrollID;

    HardwareMap hardwareMap;

    /**
     * Load and initialize all the sound files
     * @param hardwareMap hardware map
     */
    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        // Determine Resource IDs for sounds built into the RC application.
        silverSoundID     = hardwareMap.appContext.getResources().getIdentifier("silver", "raw", hardwareMap.appContext.getPackageName());
        goldSoundID       = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());
        autoCompleteId    = hardwareMap.appContext.getResources().getIdentifier("auto_complete",   "raw", hardwareMap.appContext.getPackageName());
        robotInitID       = hardwareMap.appContext.getResources().getIdentifier("robot_init",   "raw", hardwareMap.appContext.getPackageName());
        trainingWheelsID  = hardwareMap.appContext.getResources().getIdentifier("training_wheels",   "raw", hardwareMap.appContext.getPackageName());
        adminOverrideID   = hardwareMap.appContext.getResources().getIdentifier("admin_override",   "raw", hardwareMap.appContext.getPackageName());
        autoAlignID       = hardwareMap.appContext.getResources().getIdentifier("auto_alignment",   "raw", hardwareMap.appContext.getPackageName());
        rickrollID        = hardwareMap.appContext.getResources().getIdentifier("rickroll",   "raw", hardwareMap.appContext.getPackageName());

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

        if (trainingWheelsID != 0)
            trainingWheelsFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, trainingWheelsID);

        if (adminOverrideID != 0)
            adminOverrideFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, adminOverrideID);

        if (autoAlignID != 0)
            autoAlignFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, autoAlignID);

        if (rickrollID != 0)
            rickrollFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, rickrollID);
    }

    public void playSilver(){
        if (silverFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
    }

    public void playGold(){
        if (goldFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
    }

    public void playAutoComplete(){
        if (autoCompleteFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, autoCompleteId);
    }

    public void playRobotInit(){
        if (robotInitFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, robotInitID);
    }

    public void playAdminOverride(){
        if (adminOverrideFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, adminOverrideID);
    }

    public void playTrainingWheels(){
        if (trainingWheelsFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, trainingWheelsID);
    }

    public void playAutoAlign(){
        if (autoAlignFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, autoAlignID);
    }

    public void playRickroll(){
        if (rickrollFound) SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, rickrollID);
    }

}
