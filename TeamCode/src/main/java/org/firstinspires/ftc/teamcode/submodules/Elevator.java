package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevator {

    public DcMotor lift;
    public Servo angle;
    public CRServo intake;

    // Configuration variables
    public double pickup = 0.5,
                  level = 0.2;

    public void init(HardwareMap hm){
        lift = hm.dcMotor.get("linear_slide");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angle = hm.servo.get("angle");

        intake = hm.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void level(){
        angle.setPosition(level);
    }

    public void pickup(){
        angle.setPosition(pickup);
    }

    public void up() { angle.setPosition(0); }

    public void intake(double power){
        intake.setPower(power);
    }

    public void lift(double power){
        lift.setPower(power);
        if (getLiftPos() > 3000 && getLiftPos() < 6705)
            level();
    }

    public int getLiftPos(){
        return lift.getCurrentPosition() * -1;
    }

}
