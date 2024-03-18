package org.firstinspires.ftc.teamcode.testop.main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;

import org.firstinspires.ftc.teamcode.part.LinearPart;

@TeleOp(name = "Linear_TestOp", group = "")
public class Linear_TestOpMode extends OpMode {
    DcMotorHW linear1, linear2;
    LinearPart linearPart = new LinearPart(hardwareMap, telemetry);


    @Override
    public void init() {
    }
    @Override
    public void start() {
        linearPart.startStep(LinearPart.Command.MOVE_UP);
        linearPart.startStep(LinearPart.Command.MOVE_DOWN);
    }

    @Override
    public void loop() {
        linearPart.update();
    }
}
