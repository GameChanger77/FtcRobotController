package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class RawEncoderTest extends OpMode {

    DcMotor vl, vr, h;

    int vlo = 0, vro = 0, ho = 0;

    @Override
    public void init() {
        vl = hardwareMap.dcMotor.get(Constants.chassis[0]); // 0
        vr = hardwareMap.dcMotor.get(Constants.chassis[2]); // 2
        h = hardwareMap.dcMotor.get(Constants.chassis[1]); // 1

        vl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        vl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vl.setDirection(DcMotorSimple.Direction.FORWARD);
        vr.setDirection(DcMotorSimple.Direction.REVERSE);
        h.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad1.b) {
            vlo = vl.getCurrentPosition();
            vro = vr.getCurrentPosition();
            ho = h.getCurrentPosition();
        }

        telemetry.addData("RIGHT: ", vr.getCurrentPosition() - vro);
        telemetry.addData("LEFT: ", vl.getCurrentPosition() - vlo);
        telemetry.addData("HORIZONTAL: ", h.getCurrentPosition() - ho);
        telemetry.update();
    }
}
