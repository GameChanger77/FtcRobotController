package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {

    public DcMotor spinner;
    public String name = "spinner";

    public double power = .8;

    int oldPos = 0, pos, deltaPos,
            oldWVelocity = 0, wVelocity,
            oldWAcc = 0, wAcc;

    /**
     * Initialize and setup the duck carousel spinner.
     * @param hm The hardware map to use.
     */
    public void init(HardwareMap hm){
        spinner = hm.dcMotor.get(name);

        spinner.setDirection(DcMotorSimple.Direction.FORWARD);
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Used in the OdometryBase thread to keep track of the velocity of the wheel
     */
    public void update(long interval){
        pos = spinner.getCurrentPosition();
        deltaPos = pos - oldPos;
        wVelocity = pos - oldPos;
        wAcc = wVelocity - oldWVelocity;


        oldPos = pos;
        oldWVelocity = wVelocity;
    }

}
