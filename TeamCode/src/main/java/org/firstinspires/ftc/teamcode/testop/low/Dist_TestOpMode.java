package org.firstinspires.ftc.teamcode.testop.low;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.DistSensorHW;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "Dist_TestOpMode", group = "Low")
public class Dist_TestOpMode extends OpMode {
    DistSensorHW dist1;

    @Override
    public void init() {
        this.dist1 = new DistSensorHW("dist1", hardwareMap, telemetry);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double distSensorValue = dist1.getDistance();
        telemetry.addData("cm",distSensorValue);
        telemetry.update();
    }
}
