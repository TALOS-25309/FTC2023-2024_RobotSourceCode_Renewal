package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;
import org.firstinspires.ftc.teamcode.hardware.DistSensorHW;

public class WheelPart extends Part {
    // FR: Front-Right, FL: Front-Left, BR: Back-Right, BL: Back-Left
    private final DcMotorHW wheelFR, wheelFL, wheelBR, wheelBL;

    private double wheelSpeed = 0.5;
    private double wheelSpeedFast = 1.0;

    public enum Command implements RobotCommand {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_LEFT,
        MOVE_RIGHT,
        TURN_LEFT,
        TURN_RIGHT,
        STOP
    }

    public enum Direction {
        Forward(new DirectionData(1,1,1,1)),
        Backward(new DirectionData(-1,-1,-1, -1)),
        Left(new DirectionData(-1,1,1,-1)),
        Right(new DirectionData(1,-1,-1,1)),
        TurnLeft(new DirectionData(-1,1,-1,1)),
        TurnRight(new DirectionData(1,-1,1,-1));

        private final DirectionData value;
        Direction(DirectionData i) {this.value = i;}
        DirectionData get_value() {return this.value;}

        public static class DirectionData{
            private double front_left_speed = 1.0;
            private double front_right_speed = 1.0;
            private double back_left_speed = 1.0;
            private double back_right_speed = 1.0;

            public double front_left, front_right, back_left, back_right;
            public DirectionData(double front_left, double front_right, double back_left, double back_right){
                this.front_left = front_left * front_left_speed;
                this.front_right = front_right * front_right_speed;
                this.back_left = back_left * back_left_speed;
                this.back_right = back_right * back_right_speed;
            }
        }
    }

    public void setTeleWheelSpeed() {
        this.wheelSpeed = 0.2;
    }

    public WheelPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.wheelFR = new DcMotorHW("wheelFR", hwm, tel);
        this.wheelFL = new DcMotorHW("wheelFL", hwm, tel);
        this.wheelBR = new DcMotorHW("wheelBR", hwm, tel);
        this.wheelBL = new DcMotorHW("wheelBL", hwm, tel);

        wheelFR.setUsingBrake(true).setUsingEncoder(false).setDirection(DcMotorSimple.Direction.FORWARD);
        wheelFL.setUsingBrake(true).setUsingEncoder(false).setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBR.setUsingBrake(true).setUsingEncoder(false).setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBL.setUsingBrake(true).setUsingEncoder(false).setDirection(DcMotorSimple.Direction.REVERSE);

        wheelFR.setUsingFixation(false);
        wheelFL.setUsingFixation(false);
        wheelBR.setUsingFixation(false);
        wheelBL.setUsingFixation(false);


        this.hardware_manager.registerHardware(this.wheelFR).registerHardware(this.wheelFL);
        this.hardware_manager.registerHardware(this.wheelBR).registerHardware(this.wheelBL);
    }

    public void stop() {
        wheelFR.stop();
        wheelFL.stop();
        wheelBR.stop();
        wheelBL.stop();
    }

    public void move(double speed, Direction dir){
        this.wheelFL.move(speed * dir.get_value().front_left);
        this.wheelFR.move(speed * dir.get_value().front_right);
        this.wheelBL.move(speed * dir.get_value().back_left);
        this.wheelBR.move(speed * dir.get_value().back_right);
    }

    public boolean turn = false;

    public void move(double speed, Direction dir, int ticks) {
        this.wheelFL.move(speed * dir.get_value().front_left, ticks);
        this.wheelFR.move(speed * dir.get_value().front_right, ticks);
        this.wheelBL.move(speed * dir.get_value().back_left, ticks);
        this.wheelBR.move(speed * dir.get_value().back_right, ticks);
    }

    public void moveFreely(double x, double y) {
        double frlb_factor = (-x * Math.sqrt(0.5) + -y * Math.sqrt(0.5));
        double flbr_factor = (x * Math.sqrt(0.5) + -y * Math.sqrt(0.5));
        this.wheelFL.move(this.wheelSpeedFast * flbr_factor);
        this.wheelFR.move(this.wheelSpeedFast * frlb_factor);
        this.wheelBL.move(this.wheelSpeedFast * frlb_factor);
        this.wheelBR.move(this.wheelSpeedFast * flbr_factor);
    }

    public void update(double linear_length) {
        super.update();
    }

    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;

        // Move
        if (cmd == WheelPart.Command.MOVE_FORWARD) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, Direction.Forward);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == WheelPart.Command.MOVE_BACKWARD) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, Direction.Backward);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == WheelPart.Command.MOVE_LEFT) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, Direction.Left);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == WheelPart.Command.MOVE_RIGHT) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, Direction.Right);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }

        // Turn
        else if (cmd == WheelPart.Command.TURN_LEFT) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, Direction.TurnLeft);
                    this.finishStep();
                    break;
            }
        } else if (cmd == WheelPart.Command.TURN_RIGHT) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, Direction.TurnRight);
                    this.finishStep();
                    break;
            }
        }

        // Stop
        else if (cmd == WheelPart.Command.STOP) {
            switch (this.step) {
                case 0:
                    this.stop();
                    this.finishStep();
                    break;
            }
        }
    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void emergencyStop(){
        super.emergencyStop();
        this.stop();
    }
}
