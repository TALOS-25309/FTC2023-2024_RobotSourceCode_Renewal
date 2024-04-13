package org.firstinspires.ftc.teamcode.mainop.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "[BLUE FRONT] AutoOpMode", group = "")
public class AutoOpMode_LeftFront extends AutoOpMode_Main {
    @Override
    protected void setRobotStartPosition() {
        this.awheel_part.setBeginPosition("left_front");
    }
}
