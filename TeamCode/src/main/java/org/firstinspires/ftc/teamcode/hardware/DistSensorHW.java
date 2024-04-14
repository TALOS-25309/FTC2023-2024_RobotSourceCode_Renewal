package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistSensorHW extends Hardware{
    private DistanceSensor dist;

    private boolean is_color_sensor = false;


    public DistSensorHW(String name, HardwareMap hwm, Telemetry tel, Boolean is_color_sensor) {
        super(name, hwm, tel);
        this.dist = hwm.get(DistanceSensor.class, this.name);
        this.is_color_sensor = is_color_sensor;
    }

    public double getDistance() {
        return this.dist.getDistance(DistanceUnit.CM) * (this.is_color_sensor ? 2.0 : 1.0);
    }
    public boolean isObjectDetected() {
        return (this.dist.getDistance(DistanceUnit.CM) < 10);
    }

    public boolean isObjectDetected(int distcm) {
        return (this.dist.getDistance(DistanceUnit.CM) < distcm);
    }

    @Override
    public void update() {
    }
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void emergencyStop() {

    }
}
