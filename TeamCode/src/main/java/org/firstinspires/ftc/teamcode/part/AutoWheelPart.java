package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// https://www.youtube.com/watch?v=Av9ZMjS--gY

class Odometry {
    private final DcMotor odometry;
    private int previous_tick = 0;

    public Odometry(String name, HardwareMap hwm) {
        this.odometry = hwm.get(DcMotor.class, name);
        this.odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.odometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public int getDeltaTick() {
        int cur_tick = this.odometry.getCurrentPosition();
        int delta_tick = cur_tick - this.previous_tick;
        this.previous_tick = cur_tick;
        return delta_tick;
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

public class AutoWheelPart extends Part {
    private double target_x, target_y, target_theta;

    private final Wheel wheelFR, wheelFL, wheelBR, wheelBL;
    private final Odometry odometryXL, odometryXR, odometryY;

    // Constants
    // TODO : Change the values
    private final double X_OFFSET = 1.0;
    private final double Y_OFFSET = 1.0;
    private final double TILE_RATIO = 1.0;
    private final double ABLE_DISTANCE_ERROR = 0.1;
    private final double ROTATION_SPEED_FACTOR = 0.7; // X length + Y length

    private void setTarget(double x, double y, double theta) {
        this.target_x = x;
        this.target_y = y;
        this.target_theta = theta;
    }

    public enum Command implements RobotCommand {
        MOVE
    }
    public AutoWheelPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);
        this.wheelFR = new Wheel("wheelFR", hwm);
        this.wheelFL = new Wheel("wheelFL", hwm);
        this.wheelBR = new Wheel("wheelBR", hwm);
        this.wheelBL = new Wheel("wheelBL", hwm);

        // TODO : Change the names
        this.odometryXL = new Odometry("odometryXL", hwm);
        this.odometryXR = new Odometry("odometryXR", hwm);
        this.odometryY = new Odometry("odometryY", hwm);
    }

    // Set Commands : Positions and Angles
    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (cmd == Command.MOVE) {
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
        double dxl, dxr, dy;
        dxl = (double)this.odometryXL.getDeltaTick() * TILE_RATIO;
        dxr = (double)this.odometryXR.getDeltaTick() * TILE_RATIO;
        dy = (double)this.odometryY.getDeltaTick() * TILE_RATIO;

        double cur_x, cur_y, cur_theta;
        cur_x = (dxl + dxr) / 2.0;
        cur_y = dy - Y_OFFSET * (dxr - dxl) / 2.0 / X_OFFSET;
        cur_theta = (dxr - dxl) / 2.0 / X_OFFSET;

        double delta_x, delta_y, delta_theta;
        delta_x = this.target_x - cur_x;
        delta_y = this.target_y - cur_y;
        delta_theta = this.target_theta - cur_theta;

        // Mecanum Wheel Movement Calculation (https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html)

        double vx, vy, w;
        vx = delta_x;
        vy = delta_y;
        w = delta_theta;

        double abs_v = Math.abs(vx) + Math.abs(vy) + Math.abs(w * ROTATION_SPEED_FACTOR);
        if (abs_v > 1.0) {
            vx /= abs_v;
            vy /= abs_v;
            w /= abs_v;
        }

        this.wheelFL.move(vx - vy - w * ROTATION_SPEED_FACTOR);
        this.wheelFR.move(vx + vy + w * ROTATION_SPEED_FACTOR);
        this.wheelBL.move(vx + vy - w * ROTATION_SPEED_FACTOR);
        this.wheelBR.move(vx - vy + w * ROTATION_SPEED_FACTOR);

        // Check if the robot reached the target

        double ABLE_ANGLE_ERROR = 0.1;
        if (Math.abs(delta_x) < ABLE_DISTANCE_ERROR
                && Math.abs(delta_y) < ABLE_DISTANCE_ERROR
                && Math.abs(delta_theta) < ABLE_ANGLE_ERROR) {
            this.wheelFL.stop();
            this.wheelFR.stop();
            this.wheelBL.stop();
            this.wheelBR.stop();

            this.step++;
            this.nextStep();
        }
    }
}
