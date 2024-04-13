package org.firstinspires.ftc.teamcode.mainop.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "[BLUE BACK] AutoOpMode", group = "")
public class AutoOpMode_LeftBack extends AutoOpMode_Main {
    @Override
    protected void setRobotStartPosition() {
        this.awheel_part.setBeginPosition("left_back");
    }
}
