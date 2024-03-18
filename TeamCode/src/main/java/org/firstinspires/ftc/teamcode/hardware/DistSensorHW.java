package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistSensorHW extends Hardware{
    private DistanceSensor dist;


    public DistSensorHW(String name, HardwareMap hwm, Telemetry tel) {
        super(name, hwm, tel);
        this.dist = hwm.get(DistanceSensor.class, this.name);
    }

    public double getDistance() {
        return this.dist.getDistance(DistanceUnit.CM);
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
