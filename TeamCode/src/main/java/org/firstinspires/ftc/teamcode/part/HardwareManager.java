package org.firstinspires.ftc.teamcode.part;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.util.ArrayList;

public class HardwareManager {
    private ArrayList<Hardware> hw_list = new ArrayList<>();

    public HardwareManager registerHardware(Hardware hardware){
        this.hw_list.add(hardware);
        return this;
    }

    public void clearHardware(){
        this.hw_list.clear();
    }

    public void update() {
        for (Hardware hw : this.hw_list) {
            hw.update();
        }
    }

    public boolean isFinished() {
        for (Hardware hw : this.hw_list) {
            if (!hw.isFinished()) return false;
        }
        return true;
    }

    public void emergencyStop() {
        for (Hardware hw : this.hw_list){
            hw.emergencyStop();
        }
    }
}
