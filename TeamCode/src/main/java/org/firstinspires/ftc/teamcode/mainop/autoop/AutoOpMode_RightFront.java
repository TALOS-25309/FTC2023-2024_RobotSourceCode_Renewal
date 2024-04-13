package org.firstinspires.ftc.teamcode.mainop.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "[RED FRONT] AutoOpMode", group = "")
public class AutoOpMode_RightFront extends AutoOpMode_Main {
    @Override
    protected void setRobotStartPosition() {
        this.awheel_part.setBeginPosition("right_front");
    }
}
