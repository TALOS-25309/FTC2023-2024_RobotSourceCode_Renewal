package org.firstinspires.ftc.teamcode.testop.low;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;
import org.firstinspires.ftc.teamcode.hardware.ServoHW;

@TeleOp(name = "Pincer_TestOp", group = "Low")
public class Pincer_TestOpMode extends OpMode {
    ServoHW finger1, finger2, arm1, arm2, wrist;

    @Override
    public void init() {
        this.finger1 = new ServoHW("s0", hardwareMap, telemetry);
        this.finger2 = new ServoHW("s1", hardwareMap, telemetry);
        this.wrist = new ServoHW("s2", hardwareMap, telemetry);
        this.arm1 = new ServoHW("s3", hardwareMap, telemetry);
        this.arm2 = new ServoHW("s4", hardwareMap, telemetry);
        finger1.setDirection(Servo.Direction.FORWARD);
        finger2.setDirection(Servo.Direction.REVERSE);
        arm1.setDirection(Servo.Direction.FORWARD);
        arm2.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);
        finger1.setInitialPosition(0.5);
        finger2.setInitialPosition(0.5);
        arm1.setInitialPosition(0.5);
        arm2.setInitialPosition(0.5);
        wrist.setInitialPosition(0.5);
    }

    @Override
    public void start() {
        if (finger1.isFinished()) finger1.moveDirectly(0.5, 1000);
        if (finger2.isFinished()) finger2.moveDirectly(0.5, 1000);
        if (arm1.isFinished()) arm1.moveWithInterval(0.5, 2000, 1000);
        if (arm2.isFinished()) arm2.moveWithInterval(0.5, 2000, 1000);
        if (wrist.isFinished()) wrist.moveWithInterval(0.5, 1000, 1000);
    }

    @Override
    public void loop() {
        finger1.update();
        finger2.update();
        //arm1.update();
        //arm2.update();
        //wrist.update();

        if (finger1.isFinished()) finger1.moveDirectly(0.5, 1000);
        if (finger2.isFinished()) finger2.moveDirectly(0.5, 1000);
        if (arm1.isFinished()) arm1.moveWithInterval(0.5, 2000, 1000);
        if (arm2.isFinished()) arm2.moveWithInterval(0.5, 2000, 1000);
        if (wrist.isFinished()) wrist.moveWithInterval(0.5, 1000, 1000);
    }
}
