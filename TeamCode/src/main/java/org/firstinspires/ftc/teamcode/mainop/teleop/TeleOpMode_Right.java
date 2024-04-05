package org.firstinspires.ftc.teamcode.mainop.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "[RED] TeleOpMode", group = "")
public class TeleOpMode_Right extends TeleOpMode_Main {

    @Override
    protected double getX() {
        return gamepad1.left_stick_y;
    }

    @Override
    protected double getY() {
        return -gamepad1.left_stick_x;
    }
}
