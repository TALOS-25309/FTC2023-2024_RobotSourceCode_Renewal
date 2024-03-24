package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Odometry {
    private final DcMotor odometry;
    private int sign = 1;
    private int last_tick = 0;
    public Odometry(String name, HardwareMap hwm) {
        this.odometry = hwm.get(DcMotor.class, name);
        this.odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.odometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public int getDeltaTick() {
        int delta = this.odometry.getCurrentPosition() * this.sign - this.last_tick;
        this.last_tick += delta;
        return delta;
    }

    public void reverse() {
        this.sign = -1;
    }
}

class Wheel {
    private final DcMotor wheel;

    public Wheel(String name, HardwareMap hwm) {
        this.wheel = hwm.get(DcMotor.class, name);
        this.wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setDirection(DcMotorSimple.Direction direction) {
        this.wheel.setDirection(direction);
    }
    public void move(double power) {
        this.wheel.setPower(power);
    }
    public void stop() {
        this.wheel.setPower(0);
    }
}

class RobotPosition {
    public double x, y, theta;
    public RobotPosition(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}

public class AutoWheelPart extends Part {
    private final RobotPosition target = new RobotPosition(0,0,0);
    private final RobotPosition current = new RobotPosition(0, 0, 0);
    private final Wheel wheelFR, wheelFL, wheelBR, wheelBL;
    private final Odometry odometryXL, odometryXR, odometryY;

    // Constants
    // TODO : Change the values
    private final double X_OFFSET = 0.241;
    private final double Y_OFFSET = 0.25;
    private final double TILE_RATIO = 0.000124;
    private final double ABLE_DISTANCE_ERROR = 0.005;
    private final double ABLE_ANGLE_ERROR = 0.005;
    private final double ROTATION_SPEED_FACTOR = X_OFFSET + Y_OFFSET;
    private final double SPEED_FACTOR = 0.5;
    private final int STOP_LIMIT = 50;

    private int stop_counter = 0;

    private void setTarget(double x, double y, double angle) {
        this.target.x = x;
        this.target.y = y;
        this.target.theta = angle * Math.PI / 180;
    }

    public enum Command implements RobotCommand {
        MOVE,
        RETURN
    }
    public AutoWheelPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.wheelFR = new Wheel("wheelFR", hwm);
        this.wheelFL = new Wheel("wheelFL", hwm);
        this.wheelBR = new Wheel("wheelBR", hwm);
        this.wheelBL = new Wheel("wheelBL", hwm);

        this.wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.wheelFR.setDirection(DcMotorSimple.Direction.FORWARD);
        this.wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.wheelBR.setDirection(DcMotorSimple.Direction.FORWARD);

        // TODO : Change the names
        this.odometryXL = new Odometry("wheelFL", hwm);
        this.odometryXR = new Odometry("wheelFR", hwm);
        this.odometryY = new Odometry("wheelBL", hwm);
        this.odometryXL.reverse();
        this.odometryXR.reverse();
        this.odometryY.reverse();
    }

    // Set Commands : Positions and Angles
    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (cmd == Command.MOVE) {
            switch(this.step) {
                case 0:
                    this.setTarget(0, 2, 180);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
        else if (cmd == Command.RETURN) {
            switch(this.step) {
                case 0:
                    this.setTarget(0, 0, 0);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
    }

    // New Update Function for Autonomous Period
    public void update() {
        // Position Calculation (By using Odometry)
        // TODO : Change the formulas (Fit to the robot) + Add IMU
        double dxl_odm, dxr_odm, dy_odm;
        dxl_odm = (double)this.odometryXL.getDeltaTick() * TILE_RATIO;
        dxr_odm = (double)this.odometryXR.getDeltaTick() * TILE_RATIO;
        dy_odm = (double)this.odometryY.getDeltaTick() * TILE_RATIO;

        double dx, dy, dtheta;

        dx = (dxl_odm + dxr_odm) / 2.0;
        dy = dy_odm + (dxr_odm - dxl_odm) / 2.0 / X_OFFSET * Y_OFFSET;
        dtheta = (dxr_odm - dxl_odm) / 2.0 / X_OFFSET;

        this.current.x += dx * Math.cos(this.current.theta) + dy * Math.sin(this.current.theta);
        this.current.y += dx * Math.sin(this.current.theta) + dy * Math.cos(this.current.theta);
        this.current.theta += dtheta;

        double delta_x, delta_y, delta_theta;
        delta_x = this.target.x - this.current.x;
        delta_y = this.target.y - this.current.y;
        delta_theta = this.target.theta - this.current.theta;

        telemetry.addData("Target X", this.target.x);
        telemetry.addData("Target Y", this.target.y);
        telemetry.addData("Target Theta", this.target.theta);
        telemetry.addData("Current X", this.current.x);
        telemetry.addData("Current Y", this.current.y);
        telemetry.addData("Current Theta", this.current.theta);


        // Mecanum Wheel Movement Calculation (https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html)

        double vx, vy, w;
        vx = delta_x * Math.cos(this.current.theta) + delta_y * Math.sin(this.current.theta);
        vy = delta_x * Math.sin(this.current.theta) + delta_y * Math.cos(this.current.theta);
        w = delta_theta;

        double abs_v = Math.abs(vx) + Math.abs(vy) + Math.abs(w * ROTATION_SPEED_FACTOR);
        if (abs_v > 1.0) {
            vx /= abs_v;
            vy /= abs_v;
            w /= abs_v;
        }

        double wheel_speed_FL = (vx + vy - w * ROTATION_SPEED_FACTOR) * SPEED_FACTOR;
        double wheel_speed_FR = (vx - vy + w * ROTATION_SPEED_FACTOR) * SPEED_FACTOR;
        double wheel_speed_BL = (vx - vy - w * ROTATION_SPEED_FACTOR) * SPEED_FACTOR;
        double wheel_speed_BR = (vx + vy + w * ROTATION_SPEED_FACTOR) * SPEED_FACTOR;
        this.wheelFL.move( wheel_speed_FL * 0.9 + 0.1 * (wheel_speed_FL > 0 ? 1.0 : -1.0));
        this.wheelFR.move( wheel_speed_FR * 0.9 + 0.1 * (wheel_speed_FR > 0 ? 1.0 : -1.0));
        this.wheelBL.move( wheel_speed_BL * 0.9 + 0.1 * (wheel_speed_BL > 0 ? 1.0 : -1.0));
        this.wheelBR.move( wheel_speed_BR * 0.9 + 0.1 * (wheel_speed_BR > 0 ? 1.0 : -1.0));

        // Check if the robot reached the target

        if (Math.abs(delta_x) < ABLE_DISTANCE_ERROR
                && Math.abs(delta_y) < ABLE_DISTANCE_ERROR
                && Math.abs(delta_theta) < ABLE_ANGLE_ERROR) {
            this.wheelFL.stop();
            this.wheelFR.stop();
            this.wheelBL.stop();
            this.wheelBR.stop();

            this.stop_counter++;
            if (this.stop_counter > STOP_LIMIT) {
                this.step++;
                this.nextStep();
                this.stop_counter = 0;
            }
        }
    }
}