package org.firstinspires.ftc.teamcode.hardware;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;
public class MagSensorHW extends Hardware {
    private TouchSensor mag;

    private boolean until_ac = false;
    private boolean until_inac = false;

    public MagSensorHW(String name, HardwareMap hwm, Telemetry tel) {
        super(name, hwm, tel);
        this.mag = hwm.get(TouchSensor.class, this.name);
        this.until_ac = false;
        this.until_inac = false;
    }

    public boolean isActivated() {
        return this.mag.isPressed();
    }

    public void untilActivated() {
        this.until_ac = true;
        this.until_inac = false;
    }

    public void untilInactivate() {
        this.until_inac = true;
        this.until_ac = false;
    }

    public void notUse() {
        this.until_ac = false;
        this.until_inac = false;
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        if (until_ac) {
            return isActivated();
        } else if (until_inac) {
            return !isActivated();
        } else {
            return true;
        }
    }

    @Override
    public void emergencyStop() {

    }
}
