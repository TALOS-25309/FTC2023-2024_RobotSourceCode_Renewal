package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoHW extends Hardware {
    private Servo servo;

    private double target_value = 0.0;
    private double begin_value = 0.0;
    private long command_begin_time = 0;
    private long command_end_time = 0;

    // ==================== Initialization ====================
    // Initialize the DC Motor with the standard settings
    public ServoHW(String name, HardwareMap hwm, Telemetry tel) {
        super(name, hwm, tel);
        this.servo = hwm.get(Servo.class, this.name);
    }

    public void setInitialPosition(double pos) {
        this.servo.setPosition(pos);
    }

    // ==================== Settings ====================
    // Set the direction of the motor (FORWARD or REVERSE)
    public ServoHW setDirection(Servo.Direction direction) {
        this.servo.setDirection(direction);
        return this;
    }

    // ==================== Getters ====================
    public double getPosition() {
        return this.servo.getPosition();
    }

    // ==================== Ordering Commands ====================
    // Move the motor with the given power
    public void moveDirectly(double position) {
        this.target_value = position;
        this.begin_value = this.servo.getPosition();
        this.command_begin_time = System.currentTimeMillis();
        this.command_end_time = this.command_begin_time;
    }
    // Move the motor with the given power and the given ticks
    public void moveWithInterval(double position, long time_interval) {
        this.target_value = position;
        this.begin_value = this.servo.getPosition();
        this.command_begin_time = System.currentTimeMillis();
        this.command_end_time = this.command_begin_time + time_interval;
    }
    // Stop the servo
    public void stop() {
        this.target_value = this.servo.getPosition();
        this.begin_value = this.target_value;
        this.command_begin_time = 0;
        this.command_end_time = 0;
        this.servo.setPosition(this.target_value);
    }

    @Override
    public void update() {
        if (!this.isFinished()) {
            if (this.command_end_time <= System.currentTimeMillis()) { // When the action is ended
                this.servo.setPosition(this.target_value);
                this.command_end_time = 0;
            } else {
                double progress = (System.currentTimeMillis() - this.command_begin_time) / (double) (this.command_end_time - this.command_begin_time);
                if (progress > 1.0) progress = 1.0;
                if (!(progress >= 0.0)) progress = 0.0;
                this.servo.setPosition(this.begin_value + (this.target_value - this.begin_value) * progress);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return this.command_end_time == 0;
    }

    @Override
    public void emergencyStop() {
        this.stop();
    }
}
