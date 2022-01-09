package org.firstinspires.ftc.teamcode.submodules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalTelemetry;

public class RobotHardware {

    public Chassis chassis = new Chassis();
    public Gyro gyro = new Gyro();
    public Conveyor conveyor = new Conveyor();
    public Spinner spinner = new Spinner();
    public SoundManager sound = new SoundManager();
    public Sonar sonar = new Sonar();

    public Thread gyroThread = new Thread(gyro);

    public VoltageSensor voltage;

    public GlobalTelemetry gt;

    /**
     * Inject the telemetry dependency
     * @param gt GlobalTelemetry object for messaging
     */
    public RobotHardware(GlobalTelemetry gt) {
        this.gt = gt;
    }

    /**
     * Initialize the robot's submodules
     * @param hm hardware map for the op mode
     */
    public void init(HardwareMap hm){
        chassis.init(hm);
        gyro.init(hm);
        conveyor.init(hm);
        spinner.init(hm);
        sound.init(hm);
        sonar.init(hm);

        gyroThread.start();

        voltage = hm.get(VoltageSensor.class, "Control Hub");

        sound.playRobotInit();
    }

}
