package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// https://www.youtube.com/watch?v=Av9ZMjS--gY
public class AutoWheelPart extends Part {
    private double target_x, target_y, target_theta;

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
    }

    // Set Commands : Positions and Angles
    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (cmd == Command.MOVE) {
            setTarget(0, 0, 0);
        }
    }

    // New Update Function for Autonomous Period
    public void update() {
        double cur_x, cur_y, cur_theta;
    }
}
