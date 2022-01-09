package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spinner implements Runnable {

    public DcMotor spinner;
    public String name = "spinner";
    final double CPR = 140;

    public double power = 0;

    int oldPos = 0, pos, deltaPos;
    double oldWVelocity = 0, wVelocity, wAcc;

    private boolean isRunning = true;

    long time = System.currentTimeMillis(), previousTime = time, deltaTime = 0;

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
     * It is being switched to a spinner specific thread for more accurate detection
     */
    public void update(double interval){
        pos        = spinner.getCurrentPosition();
        deltaPos   = pos - oldPos;
        wVelocity  = ((pos - oldPos) / CPR) / interval;  // revolutions per time
        wAcc       = (wVelocity - oldWVelocity) / interval;

        oldPos = pos;
        oldWVelocity = wVelocity;
    }

    public void print(Telemetry telemetry){
        telemetry.addData("SPINNER",
                String.format("Pos: %d | dPos: %d | Vel: %.2f | Acc: %.2f",
                spinner.getCurrentPosition(), deltaPos, wVelocity, wAcc));
    }

    /**
     * Gives the duck spinner a certain amount of power to reach a certain speed.
     * @param rps The revolutions per second of the duck spinner.
     */
    public void runAtRPS(double rps){
        double delta = rps - wVelocity;
        power += delta / 100;
        spinner.setPower(power);
    }

    @Override
    public void run() {
        previousTime = time;
        while (isRunning){
            time = System.currentTimeMillis();
            deltaTime = time - previousTime;
            update(deltaTime);

            previousTime = time;
        }
    }

    public void stop(){ isRunning = false; }

}
