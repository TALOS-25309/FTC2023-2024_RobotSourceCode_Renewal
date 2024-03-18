package org.firstinspires.ftc.teamcode.testop.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.time.Duration;
import java.time.Instant;

import org.firstinspires.ftc.teamcode.hardware.IMUHW;
/*
@TeleOp(name = "IMU_TestOp", group = "")
public class IMU_TestOpMode extends OpMode {
    IMUHW imu;

    public double currentAngle = 0;
    public long prev_time = 0;

    public YawPitchRollAngles orientation;
    public AngularVelocity angularVelocity;
    public double yaw_init;
    public double pitch_init;
    public double roll_init;



    @Override
    public void init() {
        this.imu = new IMUHW("imu", hardwareMap, telemetry);
        imu.setOrientation();

    }

    @Override
    public void start() {
        prev_time = System.currentTimeMillis();
        orientation = imu.getOrientation();
        angularVelocity = imu.getAngularVelocity();
        yaw_init = orientation.getYaw(AngleUnit.DEGREES);
        pitch_init = orientation.getPitch(AngleUnit.DEGREES);
        roll_init = orientation.getRoll(AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        long timeChange = System.currentTimeMillis() - prev_time;
        prev_time = System.currentTimeMillis();
        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", imu.logoDirection, imu.usbDirection);

        orientation = imu.getOrientation();
        angularVelocity = imu.getAngularVelocity();

        telemetry.addData("Yaw (Z)", "%.2f Deg.", orientation.getYaw(AngleUnit.DEGREES) - yaw_init);
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES) - pitch_init);
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES) - roll_init);

        telemetry.addData("Yaw (Z)", "%.2f Deg.", imu.getAngle('Z', timeChange));
        telemetry.addData("Pitch (X)", "%.2f Deg.", imu.getAngle('X', timeChange));
        telemetry.addData("Roll (Y)", "%.2f Deg.", imu.getAngle('Y', timeChange));
        telemetry.update();
    }

}

*/