package org.firstinspires.ftc.teamcode.testop.low;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Odometry_TestOp", group = "Low")
public class Odometry_TestOpMode extends OpMode {

    DcMotor odometry;
    double ticks_per_rotation;

    @Override
    public void init() {
        odometry = hardwareMap.get(DcMotor.class, "odm");

        ticks_per_rotation = odometry.getMotorType().getTicksPerRev();

        //Test 1 : Get the ticks per rotation of the motor
        telemetry.addData("tpr", ticks_per_rotation);
        telemetry.update();

        DcMotor.RunMode mode;

        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        odometry.setMode(mode);

        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        odometry.setMode(mode);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        int cur_pos = odometry.getCurrentPosition();
        telemetry.addData("cur_pos", cur_pos);
    }
}
