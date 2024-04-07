package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;

public class EaterPart extends Part {
    private final DcMotorHW conveyor, rotor;
    private final double conveyor_speed = 1.0;
    private final double rotor_speed = 0.5;

    public enum Command implements RobotCommand {
        MOVE_UP,
        MOVE_DOWN,
        STOP,
        AUTO_PIXEL_EAT
    }

    public EaterPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.conveyor = new DcMotorHW("conveyor", hwm, tel);
        this.rotor = new DcMotorHW("rotor", hwm, tel);

        this.conveyor.setDirection(DcMotor.Direction.REVERSE);
        this.rotor.setDirection(DcMotor.Direction.FORWARD);

        this.conveyor.setUsingEncoder(false).setUsingFixation(false).setUsingBrake(false);
        this.rotor.setUsingEncoder(false).setUsingFixation(false).setUsingBrake(false);

        this.hardware_manager.registerHardware(this.conveyor);
        this.hardware_manager.registerHardware(this.rotor);
    }

    public void stop() {
        this.conveyor.stop();
        this.rotor.stop();
    }

    public void moveUp() {
        this.conveyor.move(conveyor_speed);
        this.rotor.move(rotor_speed);
    }

    public void moveDown() {
        this.conveyor.move(-conveyor_speed);
        this.rotor.move(-rotor_speed);
    }

    public void update() {
        super.update();
    }

    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (cmd == Command.MOVE_UP){
            switch(this.step) {
                case 0:
                    this.moveUp();
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.MOVE_DOWN){
            switch(this.step) {
                case 0:
                    this.moveDown();
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.STOP){
            switch(this.step) {
                case 0:
                    this.stop();
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.AUTO_PIXEL_EAT){
            switch(this.step) {
                case 0:
                    this.moveUp();
                    this.delayTime(5000);
                    break;
                case 1:
                    this.stop();
                    break;
                case 2:
                    this.finishStep();
                    break;
            }
        }
    }

    @Override
    public void emergencyStop() {
        this.stop();
    }
}
