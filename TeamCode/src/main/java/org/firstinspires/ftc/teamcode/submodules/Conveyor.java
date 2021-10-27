package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conveyor {

    public DcMotor motor;
    public String name = "conveyor";

    /**
     * Initialize and setup the conveyor motor.
     * @param hm The hardware map to use.
     */
    public void init(HardwareMap hm){
        motor = hm.dcMotor.get(name);

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
