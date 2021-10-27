package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {

    public DcMotor spinner;
    public String name = "spinner";

    public int pos = 10000;
    public double power = .8;

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
     * This will make the motor spin the carousel just enough to dop off one duck.
     */
    public void spinOnce(){
        spinner.setTargetPosition(pos);
        spinner.setPower(power);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
