package org.firstinspires.ftc.teamcode.testop.main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.part.AutoWheelPart;

@TeleOp(name = "AutoDriver_TestOpMode", group = "Low")
public class AutoDriver_TestOpMode extends OpMode {
    AutoWheelPart autoWheelPart;
    int step = 0;
    @Override
    public void init() {
        autoWheelPart = new AutoWheelPart(hardwareMap, telemetry);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        autoWheelPart.update();
        if (autoWheelPart.isFinished()) {
            switch (this.step) {
                case 0:
                    autoWheelPart.startStep(AutoWheelPart.Command.MOVE);
                    break;
                case 1:
                    autoWheelPart.startStep(AutoWheelPart.Command.RETURN);
                    break;
            }
            this.step++;
        }
    }
}
