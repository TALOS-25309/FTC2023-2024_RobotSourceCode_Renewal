package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Hardware {
    protected Telemetry telemetry;
    protected HardwareMap hardware_map;
    protected String name;

    public Hardware(String name, HardwareMap hwm, Telemetry tel) {
        this.name = name;
        this.hardware_map = hwm;
        this.telemetry = tel;
    }
    public abstract void update();
    public abstract boolean isFinished();
    public abstract void emergencyStop();
}
