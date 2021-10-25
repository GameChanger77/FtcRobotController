package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class GlobalTelemetry {
    Telemetry telemetry;
    String caption = "/> ";
    ArrayList<String> messages = new ArrayList<>();

    public GlobalTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addLine(String message){
        messages.add(message);
    }

    public void addLine(int index, String message){
        messages.add(index, message);
    }

    public void addData(String cap, String message){
        messages.add(cap+message);
    }

    public void addData(String cap, double message){
        messages.add(cap+message);
    }

    public void setCaption(String cap){
        this.caption = cap;
    }

    public void print(){
        for (String e : messages) {
            telemetry.addData("/>", e);
        }
        messages.clear();
    }
}
