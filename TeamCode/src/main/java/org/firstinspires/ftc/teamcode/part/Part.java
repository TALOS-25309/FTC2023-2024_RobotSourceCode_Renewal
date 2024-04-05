package org.firstinspires.ftc.teamcode.part;

/*
*
*/

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

import java.util.ArrayList;

public abstract class Part {

    public enum Command implements RobotCommand {
        NONE
    }

    protected Telemetry telemetry;
    protected HardwareMap hardware_map;
    protected HardwareManager hardware_manager;
    protected RobotCommand current_command = Command.NONE;
    protected int step = -1;

    private long delay_time = 0;
    private boolean finish = true;

    protected void delayTime(long delay) {
        delay_time = System.currentTimeMillis() + delay;
    }

    // Constructor
    public Part(HardwareMap hwm, Telemetry tel) {
        this.telemetry = tel;
        this.hardware_map = hwm;
        this.hardware_manager = new HardwareManager();
    }

    // Begin the specific step
    public void startStep(RobotCommand cmd){
        if(this.isFinished()) {
            this.current_command = cmd;
            this.step = 0;
            this.finish = false;
            this.nextStep();
        }
    }

    // Execute the next step (Internal)
    // This function contains the procedure of the IO process of the robot
    protected abstract void nextStep(); // IMPORTANT : EVERY PARTS SHOULD OVERRIDE THIS FUNCTION

    // Change to the next step
    private void changeToTheNextStep() {
        this.step++;
        this.nextStep();
    }

    // Update the hardware objects of the part and check the step was finished
    public void update(){
        hardware_manager.update();
        if(hardware_manager.isFinished() && System.currentTimeMillis() > this.delay_time){
            this.changeToTheNextStep();
        }
    }

    // Stop the robot on the emergency situation
    public void emergencyStop() {
        hardware_manager.emergencyStop();
        this.current_command = Command.NONE;
        this.step = -1;
        this.finish = true;
    }

    // Change to normal state
    public void changeNormalState() {

    }

    // Finish the step (Internal)
    protected void finishStep(){
        this.current_command = Command.NONE;
        this.finish = true;
    }

    // Check that the assigned command is finished
    public boolean isFinished(){
        return this.finish;
    }

    public static class HardwareManager {
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
}
