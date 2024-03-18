package org.firstinspires.ftc.teamcode.testop.low;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;
import org.firstinspires.ftc.teamcode.hardware.ServoHW;

@TeleOp(name = "Main TestOp", group = "Low")
public class TestOpMode extends OpMode {
    DcMotorHW motor1, motor2;
    ServoHW servo1, servo2;

    @Override
    public void init() {
        this.motor1 = new DcMotorHW("motor1", hardwareMap, telemetry);
        this.motor2 = new DcMotorHW("motor2", hardwareMap, telemetry);
        this.servo1 = new ServoHW("servo1", hardwareMap, telemetry);
        this.servo2 = new ServoHW("servo2", hardwareMap, telemetry);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor1.setUsingBrake(true).setUsingEncoder(false).setUsingFixation(false);
        motor2.setUsingBrake(true).setUsingEncoder(false).setUsingFixation(false);
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);
        servo1.setInitialPosition(0.8);
        servo2.setInitialPosition(0.8);
    }

    @Override
    public void start() {
        motor1.move(1.0);
        motor2.move(1.0);
        servo1.moveDirectly(1.0, 8000);
        servo2.moveDirectly(1.0, 8000);
    }

    @Override
    public void loop() {
        motor1.update();
        motor2.update();
        servo1.update();
        servo2.update();

        if (servo1.isFinished()) {
            servo1.moveDirectly(0.8, 1000);
        }
        if (servo2.isFinished()) {
            servo2.moveDirectly(0.8, 1000);
        }
    }
}
