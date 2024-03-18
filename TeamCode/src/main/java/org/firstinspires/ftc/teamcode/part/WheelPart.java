package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;
import org.firstinspires.ftc.teamcode.hardware.DistSensorHW;
import org.firstinspires.ftc.teamcode.hardware.IMUHW;
import org.firstinspires.ftc.teamcode.mainop.AutoOpModeLeft;

public class WheelPart extends Part {
    // FR: Front-Right, FL: Front-Left, BR: Back-Right, BL: Back-Left
    public DcMotorHW wheelFR, wheelFL, wheelBR, wheelBL;
    DistSensorHW backboard_dist_sensor;

    private final double length_of_robot = 28.0;
    private final double length_of_pincer = 28.0;
    private final double height_of_wheel = 8.0;
    private final double angle_of_backboard = 62.0;
    private final double angle_of_linear = 53.0;

    private double backboard_dist = 0.0;
    private boolean use_auto = false;

    private double autoview_angle;
    private double autoview_speed;
    private boolean autoview_running = false;

    public static WheelPart.Direction auto_wheel_move_dir = Direction.Forward;
    public static int auto_wheel_move_length = 0;
    public final static int auto_detect_pos_length = 1000;

    public IMUHW imuhw;
    public enum Command implements RobotCommand {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_LEFT,
        MOVE_RIGHT,
        TURN_LEFT,
        TURN_RIGHT,
        VIEW_RIGHT,
        VIEW_LEFT,
        VIEW_FORWARD,
        VIEW_BACKWARD,
        STOP,
        MOVE_DETECT_POS,
        AUTO_MOVE,
        AUTO_LEFT_WITH_SOME_DELAY,
        AUTO_RIGHT_WITH_SOME_DELAY
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

    public double wheelSpeed = 0.5;
    public double wheelSpeedFast = 1.0;

    public void setTeleWheelSpeed() {
        this.wheelSpeed = 0.2;
    }

    public WheelPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.wheelFR = new DcMotorHW("wheelFR", hwm, tel);
        this.wheelFL = new DcMotorHW("wheelFL", hwm, tel);
        this.wheelBR = new DcMotorHW("wheelBR", hwm, tel);
        this.wheelBL = new DcMotorHW("wheelBL", hwm, tel);

        this.backboard_dist_sensor = new DistSensorHW("backboard", hwm, tel);

        this.imuhw = new IMUHW("imu", hwm, tel);

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
        this.hardware_manager.registerHardware(this.backboard_dist_sensor);
        this.hardware_manager.registerHardware(this.imuhw);
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

    public void move(double speed, double angle) {
        this.imuhw.setIsNotFinished();
        this.autoview_angle = angle;
        this.autoview_speed = speed;
        this.autoview_running = true;
    }

    private void autoview() {
        double angle = this.autoview_angle;
        double speed = this.autoview_speed;
        double currentAngle = imuhw.getAngle();
        Direction dir;
        double right = (currentAngle - angle + 360) % 360;
        double left = (angle - currentAngle + 360) % 360;
        double level = Math.min(left, right);

        if (left < right) {
            dir = Direction.TurnLeft;
            // level *= 1.01;
        } else {
            dir = Direction.TurnRight;
            // level *= 0.99;
        }
        if (currentAngle != angle) {
            if (level < 10) {
                speed *= level / 10;
                // speed *= level;
                if (speed < 0.1) speed = 0.1;
            }
            move(speed, dir);
        }
        if (level < 0.5) {
            this.imuhw.setIsFinished();
            this.autoview_running = false;
            stop();
        }
    }

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

    public void onAutoDistance() {
        this.use_auto = true;
    }

    public void offAutoDistance() {
        this.use_auto = false;
    }

    public void update(double linear_length) {
        super.update();
        double D2 = this.length_of_pincer;
        double D3 = this.length_of_robot;
        double d2 = linear_length * Math.cos(Math.toRadians(this.angle_of_linear));
        double d1 = (linear_length * Math.sin(Math.toRadians(this.angle_of_linear)) + this.height_of_wheel)
                / Math.tan(Math.toRadians(this.angle_of_backboard));
        this.backboard_dist = D2-D3+d2-d1;
        double cur_dist = this.backboard_dist_sensor.getDistance();

        if (this.use_auto && this.backboard_dist > 2.0){
            double range = 0.5;
            if (this.backboard_dist > cur_dist + range) {
                double f = this.backboard_dist - (cur_dist + range);
                f /= 5.0;
                if (f > 1.0) f = 1.0;
                this.move(wheelSpeed * f, Direction.Backward);
            }
            else if (this.backboard_dist < cur_dist - range) {
                double f = cur_dist - range - this.backboard_dist;
                f /= 5.0;
                if (f > 1.0) f = 1.0;
                this.move(wheelSpeed * f, Direction.Forward);
            }
        }
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
        } else if (cmd == WheelPart.Command.VIEW_RIGHT) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, -90.0);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == WheelPart.Command.VIEW_LEFT) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, 90.0);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == WheelPart.Command.VIEW_FORWARD) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, 0.0);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == WheelPart.Command.VIEW_BACKWARD) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, 180.0);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }

        else if (cmd == WheelPart.Command.MOVE_DETECT_POS) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, Direction.Forward, auto_detect_pos_length);
                    break;
                case 1:
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

        else if (cmd == WheelPart.Command.AUTO_MOVE) {
            switch (this.step) {
                case 0:
                    this.move(wheelSpeed, auto_wheel_move_dir, auto_wheel_move_length);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }

        else if (cmd == Command.AUTO_LEFT_WITH_SOME_DELAY) {
            switch (this.step) {
                case 0:
                    this.delayTime(2000);
                    break;
                case 1:
                    this.move(wheelSpeed, auto_wheel_move_dir, auto_wheel_move_length);
                    break;
                case 2:
                    this.move(wheelSpeed, 90.0); // LEFT VIEW
                    break;
                case 3:
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.AUTO_RIGHT_WITH_SOME_DELAY) {
            switch (this.step) {
                case 0:
                    this.delayTime(2000);
                    break;
                case 1:
                    this.move(wheelSpeed, auto_wheel_move_dir, auto_wheel_move_length);
                    break;
                case 2:
                    this.move(wheelSpeed, -90.0); // RIGHT VIEW
                    break;
                case 3:
                    this.finishStep();
                    break;
            }
        }
    }

    @Override
    public void update() {
        super.update();
        if(this.autoview_running){
            this.autoview();
        }
    }

    @Override
    public void emergencyStop(){
        super.emergencyStop();
        this.stop();
    }
}
