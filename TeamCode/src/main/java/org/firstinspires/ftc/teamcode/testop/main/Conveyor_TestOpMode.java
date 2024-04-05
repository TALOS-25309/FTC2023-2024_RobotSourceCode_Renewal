package org.firstinspires.ftc.teamcode.testop.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp


public class Conveyor_TestOpMode extends LinearOpMode {
    private DcMotor conveyor;
    private DcMotor eater;
    private CRServo servoTest;
    private Servo wrist;


    @Override
    public void runOpMode() {
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        eater = hardwareMap.get(DcMotor.class, "eater");
        servoTest = hardwareMap.get(CRServo.class, "servoTest");
        wrist = hardwareMap.get(Servo.class, "wrist");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        wrist.setPosition(0.85);
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
            eater.setDirection(DcMotorSimple.Direction.FORWARD);
            servoTest.setDirection(DcMotorSimple.Direction.FORWARD);
            conveyor.setPower(1);
            eater.setPower(0.3);
            servoTest.setPower(1);


        }
    }
}