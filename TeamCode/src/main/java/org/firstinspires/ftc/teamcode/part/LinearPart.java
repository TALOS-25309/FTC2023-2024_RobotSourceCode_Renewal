package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;
import org.firstinspires.ftc.teamcode.hardware.MagSensorHW;

public class LinearPart extends Part {
    DcMotorHW linear1, linear2;
    MagSensorHW mag;
    private boolean expand = true;
    private final double linear_speed_go_down = 0.4;
    private final double linear_speed_go_up = 0.6;
    private boolean correcting_limit = false;

    public int dropPosition = 700;
    public int upPosition = 300;
    public int endPosition = 200;

    public enum Command implements RobotCommand {
        MOVE_UP,
        MOVE_DOWN,
        STOP,
        MOVE_DROP_POSITION,
        MOVE_PSEUDO_UP_POSITION,
        MOVE_DOWN_POWERFUL,
        MOVE_ORIGINAL_POSITION,
        MOVE_AUTO_ORIGINAL_POSITION,
        MOVE_PSEUDO_END_POSITION,
        TEST_MOVE,
        MOVE_AUTO_POSITION
    }

    // Constructor
    public LinearPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.linear1 = new DcMotorHW("linear1", hwm, tel);
        this.linear2 = new DcMotorHW("linear2", hwm, tel);
        this.mag = new MagSensorHW("mag", hwm, tel);

        this.mag.notUse();

        linear1.setUsingBrake(true).setUsingFixation(true).setUsingEncoder(false);
        linear2.setUsingBrake(true).setUsingFixation(true).setUsingEncoder(false);

        this.hardware_manager.registerHardware(linear1).registerHardware(linear2).registerHardware(mag);

        linear2.setSyncedDC(this.linear1);
        linear1.setThisDcAsSynced();
    }

    public boolean isLinearDown() {
        return mag.isActivated();
    }

    public double getLength() {
        return (linear1.getAccumulatedMovingDistance() + linear2.getAccumulatedMovingDistance()) * 0.5 / 43.0;
    }

    public void moveLinear(double magnitude) {
        linear1.setDirection(expand ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        linear2.setDirection(expand ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        linear1.move(magnitude);
        linear2.move(magnitude);
    }

    public void moveLinearWithTargetTicks(double magnitude, int targetTicks) {
        linear1.setDirection(expand ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        linear2.setDirection(expand ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        linear1.move(magnitude, targetTicks);
        linear2.move(magnitude, targetTicks);
    }

    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (cmd == Command.MOVE_UP) {
            switch (this.step) {
                case 0:
                    expand = true;
                    moveLinear(linear_speed_go_up);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_DOWN) {
            switch (this.step) {
                case 0:
                    expand = false;
                    moveLinear(linear_speed_go_down);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_DOWN_POWERFUL) {
            switch (this.step) {
                case 0:
                    expand = false;
                    moveLinear(0.8);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.STOP) {
            switch (this.step) {
                case 0:
                    linear1.stop();
                    linear2.stop();
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_DROP_POSITION) {
            switch (this.step) {
                case 0:
                    expand = true;
                    moveLinearWithTargetTicks(linear_speed_go_up, dropPosition);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_AUTO_POSITION) {
            switch (this.step) {
                case 0:
                    expand = true;
                    moveLinearWithTargetTicks(linear_speed_go_up, 700);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_PSEUDO_UP_POSITION) {
            switch (this.step) {
                case 0:
                    this.delayTime(2000);
                    break;
                case 1:
                    expand = true;
                    moveLinearWithTargetTicks(linear_speed_go_up, upPosition);
                    break;
                case 2:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_PSEUDO_END_POSITION) {
            switch (this.step) {
                case 0:
                    expand = true;
                    moveLinearWithTargetTicks(linear_speed_go_down, endPosition);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_ORIGINAL_POSITION) {
            switch (this.step) {
                case 0:
                    expand = false;
                    moveLinearWithTargetTicks(linear_speed_go_down, dropPosition + upPosition + endPosition);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.MOVE_AUTO_ORIGINAL_POSITION) {
            switch (this.step) {
                case 0:
                    expand = false;
                    moveLinearWithTargetTicks(linear_speed_go_down, 700);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.TEST_MOVE) {
            switch (this.step) {
                case 0:
                    expand = true;
                    moveLinearWithTargetTicks(linear_speed_go_up, 1500);
                    break;
                case 1:
                    expand = false;
                    moveLinearWithTargetTicks(linear_speed_go_down, 1500);
                    break;
                case 2:
                    this.finishStep();
                    break;
            }
        }
    }

    @Override
    public void update(){
        super.update();
        if (mag.isActivated() && !expand) {
            linear1.stop();
            linear2.stop();
            linear1.accumulated_moving_distance = 0.0;
            linear2.accumulated_moving_distance = 0.0;
        }
    }

    @Override
    public void emergencyStop() {
        super.emergencyStop();
        this.linear1.stop();
        this.linear2.stop();
        this.linear1.setUsingFixation(false);
        this.linear2.setUsingFixation(false);
    }

    @Override
    public void changeNormalState() {
        super.changeNormalState();
        this.linear1.setUsingFixation(true);
        this.linear2.setUsingFixation(true);
    }
}
