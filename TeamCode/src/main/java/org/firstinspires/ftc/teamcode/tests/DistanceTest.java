package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.submodules.Sonar;

@TeleOp
public class DistanceTest extends OpMode {

    Sonar sonar = new Sonar();

    @Override
    public void init() {
        sonar.init(hardwareMap);
    }

    @Override
    public void loop() {
        sonar.printDistance(telemetry);
    }
}
