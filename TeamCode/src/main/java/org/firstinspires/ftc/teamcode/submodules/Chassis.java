package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class Chassis {

    public DcMotor fl,fr,bl,br;
    public static final String[] configNames = Constants.chassis.clone();

    /**
     * Initializes the motors and sets them up
     * @param hm The hardware map object to use
     */
    public void init(HardwareMap hm){
        fl = hm.dcMotor.get(configNames[0]);
        fr = hm.dcMotor.get(configNames[1]);
        br = hm.dcMotor.get(configNames[2]);
        bl = hm.dcMotor.get(configNames[3]);

        fl.setDirection(DcMotorSimple.Direction.FORWARD); // 0
        fr.setDirection(DcMotorSimple.Direction.REVERSE); // 1
        bl.setDirection(DcMotorSimple.Direction.FORWARD); // 3
        br.setDirection(DcMotorSimple.Direction.REVERSE); // 2

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Sets the power of each wheel so the robot can move in the desired direction
     * Negatives will reverse the direction
     * @param x The right component 1 to -1
     * @param y The forward component 1 to -1
     * @param r The clockwise rotation component 1 to -1
     * @param p The overall power / speed control 1 to -1
     */
    public void move(double x, double y, double r, double p){
        fl.setPower((+x + y + r) * p);
        fr.setPower((-x + y - r) * p);
        bl.setPower((-x + y + r) * p);
        br.setPower((+x + y - r) * p);

    }

    public void forward(){
        fl.setPower(1);
        fr.setPower(1);
        br.setPower(1);
        bl.setPower(1);
    }

    /**
     * This is just for fun. The robot wheels should spin but in a way that causes the robot to stay in place.
     * @param power The power of the wheels during the burnout 1 to -1
     */
    public void burnout(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(-power);
        br.setPower(-power);
    }

    /**
     * Makes all the wheel powers' equal to zero
     */
    public void stop(){
        move(0,0,0,0);
    }

}
