package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DcMotorHW extends Hardware {
    private DcMotor motor;
    private double target_ticks = 0;
    private boolean is_busy = false;
    private boolean is_free_moving = false;
    private boolean using_fixation = false;
    private boolean move_until_stuck = false;

    private double fixation_power = 2.0;
    public double accumulated_moving_distance = 0.0;

    private DcMotorHW synced_dc;
    private boolean synced = false;
    private boolean syncing = false;

    // ==================== Initialization ====================
    // Initialize the DC Motor with the standard settings
    public DcMotorHW(String name, HardwareMap hwm, Telemetry tel) {
        super(name, hwm, tel);
        this.motor = hwm.get(DcMotor.class, this.name);
        this.motor.setPower(0);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setDirection(DcMotor.Direction.FORWARD);
        this.setUsingEncoder(false).setUsingBrake(true);
        this.setUsingFixation(false);
    }

    public void initEncoder() {
        this.accumulated_moving_distance += this.motor.getCurrentPosition() * (this.motor.getDirection() == DcMotor.Direction.FORWARD ? 1.0 : -1.0);
        DcMotor.RunMode mode = this.motor.getMode();
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(mode);
    }

    public void setSyncedDC(DcMotorHW dc) {
        this.synced_dc = dc;
        this.syncing = true;
    }

    public void setThisDcAsSynced() {
        this.synced = true;
    }

    // ==================== Settings ====================
    // Set the direction of the motor (FORWARD or REVERSE)
    public DcMotorHW setDirection(DcMotor.Direction direction) {
        this.motor.setDirection(direction);
        return this;
    }
    // Set the mode of the motor (false : RUN_WITHOUT_ENCODER or true : RUN_USING_ENCODER)
    public DcMotorHW setUsingEncoder(boolean use_encoder) {
        if (use_encoder) {
            this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return this;
    }
    // Set the brake mode of the motor (false : FLOAT or true : BRAKE)
    public DcMotorHW setUsingBrake(boolean use_brake) {
        if (use_brake) {
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        return this;
    }
    // Set the fixation mode of the motor (false : not using or true : using)
    public DcMotorHW setUsingFixation(boolean use_fixation) {
        this.using_fixation = use_fixation;
        return this;
    }

    // ==================== Getters ====================
    // Get Accumulated Moving Distance
    public double getAccumulatedMovingDistance() {
        return Math.abs(this.accumulated_moving_distance);
    }

    public double getCurrentTick() {
        return this.motor.getCurrentPosition();
    }

    // ==================== Ordering Commands ====================
    // Move the motor with the given power : moving infinitely
    public void move(double power) {
        this.initEncoder();
        this.target_ticks = 0;
        this.is_busy = false;
        this.is_free_moving = true;
        this.motor.setPower(power);
    }
    // Move the motor with the given power and the given ticks
    // : moving with the given ticks, fixation is automated
    public void move(double power, double ticks) {
        this.initEncoder();
        this.target_ticks = ticks;
        this.is_busy = true;
        this.is_free_moving = false;
        this.fixation_power = 2.0; // AUTOMATED
        this.motor.setPower(power);
    }
    // Move the motor with the given power and the given ticks and the given fixation power
    // : moving with the given ticks, fixation is manual
    /*
    public void move(double power, int ticks, double fixation_power) {
        this.initEncoder();
        this.target_ticks = ticks;
        this.is_busy = true;
        this.is_free_moving = false;
        this.fixation_power = fixation_power;
        this.motor.setPower(power);
    }
    */
    // Stop the motor
    public void stop() {
        if(this.is_free_moving || this.is_busy) {
            this.target_ticks = this.getCurrentTick();
            this.is_busy = false;
            this.is_free_moving = false;
            this.fixation_power = 2.0; // AUTOMATED
            this.motor.setPower(0);
        }
    }

    public void moveUntilStuck(double power) {
        this.initEncoder();
        this.is_busy = true;
        this.move_until_stuck = true;
        this.is_free_moving = false;
        this.fixation_power = 2.0; // AUTOMATED
        this.motor.setPower(power);
    }

    @Override
    public void update() {
        if (this.is_busy) {
            if(this.move_until_stuck) {
                if (Math.abs(this.motor.getCurrentPosition()) < 0.1) {
                    this.is_busy = false;
                    this.move_until_stuck = false;
                    this.motor.setPower(0);
                }
                this.initEncoder();
            } else {
                if (Math.abs(this.motor.getCurrentPosition()) > this.target_ticks) {
                    this.is_busy = false;
                    this.motor.setPower(0);
                }
            }
        }
        else if (this.using_fixation && !this.is_free_moving && !this.synced) {
            double power = this.fixation_power;
            if (power == 2.0) { // AUTOMATED MODE
                // ideal_abs_power : ideal motor power (+ or -)
                // current_abs_power : current motor power (+ or -)
                double ideal_power = (double)(this.target_ticks - Math.abs(this.motor.getCurrentPosition())) / this.motor.getMotorType().getTicksPerRev();
                double current_power = this.motor.getPower();
                if (ideal_power * current_power < 0) {
                    // if the motor is going to the opposite direction, power = ideal_power
                    power = ideal_power;
                } else {
                    // if the motor is going to the same direction, power = current_power + ideal_power
                    power = current_power + ideal_power;
                }
                // power should be between 0.0 and 1.0
                power = Math.abs(power);
                if (power > 1.0) {
                    power = 1.0;
                }
            }
            if (this.target_ticks > 10.0) {
                if (Math.abs(this.motor.getCurrentPosition()) > this.target_ticks) {
                    this.motor.setPower(-power);
                    if (this.syncing) {
                        this.synced_dc.motor.setPower(-power);
                    }
                } else if (Math.abs(this.motor.getCurrentPosition()) < this.target_ticks) {
                    this.motor.setPower(power);
                    if (this.syncing) {
                        this.synced_dc.motor.setPower(power);
                    }
                } else {
                    this.motor.setPower(0);
                    if (this.syncing) {
                        this.synced_dc.motor.setPower(0);
                    }
                }
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
