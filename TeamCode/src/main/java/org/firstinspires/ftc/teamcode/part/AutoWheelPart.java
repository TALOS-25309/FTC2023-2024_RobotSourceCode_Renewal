package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Odometry {
    private final DcMotor odometry;
    private int last_tick = 0;
    public Odometry(String name, HardwareMap hwm) {
        this.odometry = hwm.get(DcMotor.class, name);
        this.odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.odometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public int getTick() {
        return this.odometry.getCurrentPosition() - this.last_tick;
    }
    public void reset() {
        this.last_tick = this.odometry.getCurrentPosition();
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

class Position {
    public double x, y, theta;
    public Position(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}

public class AutoWheelPart extends Part {
    private Position target;
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
        this.odometryXL.reset();
        this.odometryXR.reset();
        this.odometryY.reset();
        this.target.x = x;
        this.target.y = y;
        this.target.theta = theta;
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
        double dxl_odm, dxr_odm, dy_odm;
        dxl_odm = (double)this.odometryXL.getTick() * TILE_RATIO;
        dxr_odm = (double)this.odometryXR.getTick() * TILE_RATIO;
        dy_odm = (double)this.odometryY.getTick() * TILE_RATIO;

        double dx, dy, dtheta;

        dx = (dxl_odm + dxr_odm) / 2.0;
        dy = dy_odm - (dxr_odm - dxl_odm) / 2.0 / X_OFFSET * Y_OFFSET;
        dtheta = (dxr_odm - dxl_odm) / 2.0 / X_OFFSET;

        double delta_x, delta_y, delta_theta;
        delta_x = this.target.x - dx;
        delta_y = this.target.y - dy;
        delta_theta = this.target.theta - dtheta;

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
