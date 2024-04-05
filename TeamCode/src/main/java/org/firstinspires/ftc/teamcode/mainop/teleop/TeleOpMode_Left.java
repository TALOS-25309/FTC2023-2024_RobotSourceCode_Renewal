package org.firstinspires.ftc.teamcode.mainop.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "[BLUE] TeleOpMode", group = "")
public class TeleOpMode_Left extends TeleOpMode_Main {

    @Override
    protected double getX() {
        return -gamepad1.left_stick_y;
    }

    @Override
    protected double getY() {
        return gamepad1.left_stick_x;
    }
}
