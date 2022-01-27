package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conveyor {

    public DcMotor top;
    public DcMotor bottom;
    public String name = "conveyor_top";
    public String name2 = "conveyor_bottom";

    /**
     * Initialize and setup the conveyor motor.
     * @param hm The hardware map to use.
     */
//    public void init(HardwareMap hm){
//        top = hm.dcMotor.get(name);
//
//        top.setDirection(DcMotorSimple.Direction.REVERSE);
//        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        bottom = hm.dcMotor.get(name2);
//
//        bottom.setDirection(DcMotorSimple.Direction.FORWARD);
//        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }

    /**
     * Runs both motors
     * @param p
     */
    public void power(double p){
        top.setPower(p);
        bottom.setPower(p);
    }

}
