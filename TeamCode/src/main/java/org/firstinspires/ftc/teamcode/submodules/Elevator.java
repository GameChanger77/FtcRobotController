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
    public double pickup = 0.5,            // The angle servo's position for intake / pickup
                  hold = 0.2,              // The angle servo's position for holding
                  defaultLiftPower = -0.5; // This might need to be positive if the lift motor goes the wrong way

    public int pickupPos = 837,    // Actual mostly correct value for pickup
               level1Pos = 2_000,  // Test value for level 1
               level2Pos = 3_982,  // Actual mostly correct value for level 2
               level3Pos = 8_000;  // test value for the level 3

    /**
     * Initializes the lift, angle, and intake hardware devices
     * @param hm The hardware map
     */
    public void init(HardwareMap hm){
        lift = hm.dcMotor.get("linear_slide");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angle = hm.servo.get("angle");

        intake = hm.crservo.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Iteratively moves the lift and the servo towards the level 1 position.
     * @return True if moving towards the target / False if it has reached the target.
     */
    public boolean level3(){
        if (lift(defaultLiftPower, level3Pos)) {
            if (Math.abs(angle.getPosition() - hold) >= .1)  // If the servo isn't in the hold position
                hold();      // Set the servo to the hold position for going down
            return true; // Lift is moving towards the target
        }
        else             // Lift has reached the target
            pickup();    // Set angle servo to the pickup position
        return false;
    }

    /**
     * Iteratively moves the lift and the servo towards the level 1 position.
     * @return True if moving towards the target / False if it has reached the target.
     */
    public boolean level2(){
        if (lift(defaultLiftPower, level2Pos)) {
            if (Math.abs(angle.getPosition() - hold) >= .1)  // If the servo isn't in the hold position
                hold();      // Set the servo to the hold position for going down
            return true; // Lift is moving towards the target
        }
        else             // Lift has reached the target
            pickup();    // Set angle servo to the pickup position
        return false;
    }

    /**
     * Iteratively moves the lift and the servo towards the level 1 position.
     * @return True if moving towards the target / False if it has reached the target.
     */
    public boolean level1(){
        if (lift(defaultLiftPower, level1Pos)) {
            if (Math.abs(angle.getPosition() - hold) >= .1)  // If the servo isn't in the hold position
                hold();      // Set the servo to the hold position for going down
            return true; // Lift is moving towards the target
        }
        else             // Lift has reached the target
            pickup();    // Set angle servo to the pickup position
        return false;
    }

    /**
     * Iteratively moves the lift and the servo towards the pickup position.
     * @return True if moving towards the target / False if it has reached the target.
     */
    public boolean pickup_block(){
        if (lift(defaultLiftPower, pickupPos)) {
            if (Math.abs(angle.getPosition() - hold) >= .1)  // If the servo isn't in the hold position
                hold();      // Set the servo to the hold position for going down
            return true; // Lift is moving towards the target
        }
        else             // Lift has reached the target
            pickup();    // Set angle servo to the pickup position
        return false;
    }

    /**
     * Makes the lift move towards the target position
     * @param power The motor power
     * @param target The target position
     * @return True if the lift is moving towards the target / False if it has reached the target
     */
    public boolean lift(double power, int target){
        int dpos = target - getLiftPos();

        if (dpos > 250 || dpos < -250)  // 250 is the amount of allowed error in the target pos
            return lift(power);
        return false;
    }

    /**
     * Makes the lift move up and down. Makes the angle servo adjust at the correct position.
     * @param power The motor power
     * @return Always true because it doesn't have a specific point to reach
     */
    public boolean lift(double power){
        lift.setPower(power);
        if (getLiftPos() > 3000 && getLiftPos() < 6705) // Adjusts the servo angle for avoiding the bar
            hold();
        return true;
    }

    /**
     *
     * @return The encoder position of the motor adjusted for motor direction.
     */
    public int getLiftPos(){
        return lift.getCurrentPosition() * -1;
    }

    /**
     * The upwards position that is at a bit of an angle for holding the element
     * while lifting and lowering the scoop.
     */
    public void hold(){
        angle.setPosition(hold);
    }

    /**
     * The intake / pickup position.
     */
    public void pickup(){
        angle.setPosition(pickup);
    }

    /**
     * The straight up position for fitting in the box.
     */
    public void up() { angle.setPosition(0); }

    /**
     * Controls the continuous rotation servo to pick up the elements.
     * @param power
     */
    public void intake(double power){
        intake.setPower(power);
    }

}
