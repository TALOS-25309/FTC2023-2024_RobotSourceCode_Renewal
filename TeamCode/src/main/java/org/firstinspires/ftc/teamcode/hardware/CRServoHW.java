package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CRServoHW extends Hardware {
    private final CRServo motor;
    private double target_time = 0;
    private boolean is_busy = false;

    // ==================== Initialization ====================
    // Initialize the DC Motor with the standard settings
    public CRServoHW(String name, HardwareMap hwm, Telemetry tel) {
        super(name, hwm, tel);
        this.motor = hwm.get(CRServo.class, this.name);
        this.motor.setPower(0);
        this.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // ==================== Settings ====================
    // Set the direction of the motor (FORWARD or REVERSE)
    public CRServoHW setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
        return this;
    }

    // ==================== Ordering Commands ====================
    // Move the motor with the given power : moving infinitely
    public void move(double power) {
        this.target_time = 0;
        this.is_busy = false;
        this.motor.setPower(power);
    }

    // Move the motor with the given power and the given ticks
    public void move(double power, double time) {
        this.target_time = time + System.currentTimeMillis();
        this.is_busy = true;
        this.motor.setPower(power);
    }

    // Stop the motor
    public void stop() {
        this.is_busy = false;
        this.motor.setPower(0);
    }

    @Override
    public void update() {
        if (this.is_busy) {
            if (System.currentTimeMillis() > this.target_time) {
                this.stop();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !this.is_busy;
    }

    @Override
    public void emergencyStop() {
        this.stop();
    }

}
