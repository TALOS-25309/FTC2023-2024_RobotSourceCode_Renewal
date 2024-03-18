package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ServoHW;

public class PincerPart extends Part {
    ServoHW finger1, finger2, arm1, arm2, wrist;
    private final double fingerClosePosition = 0.33;
    private final double fingerOpenPosition = 0.46;

    private final double offset_for_left = 0.03;


    private final double wristDropPosition = 0.87;
    private final double wristDropPositionForAuto = 0.75;
    private final double wristGrabPosition = 0.51;

    private final double armGrabPosition = 0.985;
    private final double armDropPosition = 0.25;
    private final double armDropPositionForAuto = 0.15;

    public boolean is_left_opend = true;
    public boolean is_right_opend = true;
    public boolean is_drop_position = false;
    public boolean is_able_to_move_linear = false;

    public enum Command implements RobotCommand {
        GRAB_PIXEL_LEFT,
        GRAB_PIXEL_RIGHT,
        MOVE_DROP_POSITION,
        DROP_PIXEL_LEFT,
        DROP_PIXEL_RIGHT,
        MOVE_GRAB_POSITION,
        GRAB_OR_DROP_PIXEL_LEFT,
        GRAB_OR_DROP_PIXEL_RIGHT,
        MOVE_DROP_OR_GRAB_POSITION,
        AUTO_MOVE_DROP_OR_GRAB_POSITION,
        AUTO_WRIST_SETTING
    }

    public boolean isAbleToMovingLinear() {
        return this.is_able_to_move_linear;
    }

    // Constructor
    public PincerPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.finger1 = new ServoHW("s0", hwm, telemetry);
        this.finger2 = new ServoHW("s1", hwm, telemetry);
        this.wrist = new ServoHW("s2", hwm, telemetry);
        this.arm1 = new ServoHW("s3", hwm, telemetry);
        this.arm2 = new ServoHW("s4", hwm, telemetry);

        finger1.setDirection(Servo.Direction.FORWARD);
        finger2.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);
        arm1.setDirection(Servo.Direction.FORWARD);
        arm2.setDirection(Servo.Direction.REVERSE);

        finger1.setInitialPosition(fingerOpenPosition);
        finger2.setInitialPosition(fingerOpenPosition + offset_for_left);
        wrist.setInitialPosition(wristGrabPosition);
        arm1.setInitialPosition(armGrabPosition);
        arm2.setInitialPosition(armGrabPosition);

        this.hardware_manager.registerHardware(this.finger1).registerHardware(this.finger2);
        this.hardware_manager.registerHardware(this.wrist);
        this.hardware_manager.registerHardware(this.arm1).registerHardware(this.arm2);

        this.is_left_opend = true;
        this.is_right_opend = true;
    }

    public PincerPart(HardwareMap hwm, Telemetry tel, boolean auto) {
        super(hwm, tel);

        this.finger1 = new ServoHW("s0", hwm, telemetry);
        this.finger2 = new ServoHW("s1", hwm, telemetry);
        this.wrist = new ServoHW("s2", hwm, telemetry);
        this.arm1 = new ServoHW("s3", hwm, telemetry);
        this.arm2 = new ServoHW("s4", hwm, telemetry);

        finger1.setDirection(Servo.Direction.FORWARD);
        finger2.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);
        arm1.setDirection(Servo.Direction.FORWARD);
        arm2.setDirection(Servo.Direction.REVERSE);

        finger1.setInitialPosition(fingerClosePosition);
        finger2.setInitialPosition(fingerClosePosition + offset_for_left);
        wrist.setInitialPosition(wristDropPositionForAuto);
        arm1.setInitialPosition(armGrabPosition);
        arm2.setInitialPosition(armGrabPosition);

        this.hardware_manager.registerHardware(this.finger1).registerHardware(this.finger2);
        this.hardware_manager.registerHardware(this.wrist);
        this.hardware_manager.registerHardware(this.arm1).registerHardware(this.arm2);

        this.is_left_opend = true;
        this.is_right_opend = true;
    }

    public void closeLeftFinger() {
        this.is_left_opend = false;
        this.finger2.moveDirectly(fingerClosePosition + offset_for_left);
    }
    public void closeRightFinger() {
        this.is_right_opend = false;
        this.finger1.moveDirectly(fingerClosePosition);
    }
    public void openLeftFinger() {
        this.is_left_opend = true;
        this.finger2.moveDirectly(fingerOpenPosition + offset_for_left);
    }
    public void openRightFinger() {
        this.is_right_opend = true;
        this.finger1.moveDirectly(fingerOpenPosition);
    }

    public void controlLeftFinger() {
        if (this.is_left_opend) {
            this.closeLeftFinger();
        }
        else {
            this.openLeftFinger();
        }
    }

    public void controlRightFinger() {
        if (this.is_right_opend) {
            this.closeRightFinger();
        }
        else {
            this.openRightFinger();
        }
    }

    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (cmd == Command.GRAB_PIXEL_LEFT) {
            switch (this.step){
                case 0:
                    this.closeLeftFinger();
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.GRAB_PIXEL_RIGHT) {
            switch (this.step){
                case 0:
                    this.closeRightFinger();
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.DROP_PIXEL_LEFT) {
            switch (this.step){
                case 0:
                    this.openLeftFinger();
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.DROP_PIXEL_RIGHT) {
            switch (this.step){
                case 0:
                    this.openRightFinger();
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_DROP_OR_GRAB_POSITION) {
            if (!is_drop_position) {
                switch (this.step) {
                    case 0:
                        this.is_able_to_move_linear = true;
                        this.arm1.moveWithInterval(armDropPosition, 5000);
                        this.arm2.moveWithInterval(armDropPosition, 5000);
                        break;
                    case 1:

                        this.wrist.moveDirectly(wristDropPosition);
                        break;
                    case 2:
                        is_drop_position = true;
                        this.finishStep();
                        break;
                }
            } else {
                switch (this.step){
                    case 0:
                        this.closeLeftFinger();
                        this.closeRightFinger();
                        this.delayTime(500);
                        break;
                    case 1:
                        this.is_able_to_move_linear = false;
                        this.wrist.moveDirectly(wristGrabPosition);
                        this.arm1.moveWithInterval(armGrabPosition, 3000);
                        this.arm2.moveWithInterval(armGrabPosition, 3000);
                        break;
                    case 2:
                        this.openLeftFinger();
                        this.openRightFinger();
                        is_drop_position = false;
                        this.finishStep();
                        break;
                }
            }
        }
        else if (cmd == Command.AUTO_MOVE_DROP_OR_GRAB_POSITION) {
            if (!is_drop_position) {
                switch (this.step) {
                    case 0:
                        this.is_able_to_move_linear = true;
                        this.arm1.moveWithInterval(armDropPositionForAuto, 7000);
                        this.arm2.moveWithInterval(armDropPositionForAuto, 7000);
                        break;
                    case 1:
                        this.wrist.moveDirectly(wristDropPositionForAuto);
                        break;
                    case 2:
                        is_drop_position = true;
                        this.finishStep();
                        break;
                }
            } else {
                switch (this.step){
                    case 0:
                        this.closeLeftFinger();
                        this.closeRightFinger();
                        this.delayTime(500);
                        break;
                    case 1:
                        this.is_able_to_move_linear = false;
                        this.wrist.moveDirectly(wristGrabPosition);
                        this.arm1.moveWithInterval(armGrabPosition, 5000);
                        this.arm2.moveWithInterval(armGrabPosition, 5000);
                        break;
                    case 2:
                        this.openLeftFinger();
                        this.openRightFinger();
                        is_drop_position = false;
                        this.finishStep();
                        break;
                }
            }
        }
        /*
        else if (cmd == Command.MOVE_GRAB_POSITION) {
            switch (this.step){
                case 0:
                    this.wrist.moveDirectly(wristGrabPosition);
                    break;
                case 1:
                    this.arm1.moveWithInterval(armGrabPosition, 2000);
                    this.arm2.moveWithInterval(armGrabPosition, 2000);
                    break;
                case 2:
                    this.finishStep();
                    break;
            }
        }

         */
        else if (cmd == Command.GRAB_OR_DROP_PIXEL_LEFT) {
            this.controlLeftFinger();
            this.finishStep();
        }
        else if (cmd == Command.GRAB_OR_DROP_PIXEL_RIGHT) {
            this.controlRightFinger();
            this.finishStep();
        }
        else if (cmd == Command.AUTO_WRIST_SETTING) {
            this.wrist.moveDirectly(this.wristGrabPosition);
            this.finishStep();
        }
    }
}
