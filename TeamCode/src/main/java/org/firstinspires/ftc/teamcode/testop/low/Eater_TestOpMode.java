// Linear Test Code

package org.firstinspires.ftc.teamcode.testop.low;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Eater_TestOp", group = "Low")
public class Eater_TestOpMode extends OpMode {
    DcMotor eater1, eater2;
    double ticks_per_rotation_1, ticks_per_rotation_2;
    boolean is_completed_1 = false, is_completed_2 = false;
    boolean expand = true;

    @Override
    public void init() {
        eater1= hardwareMap.get(DcMotor.class, "eater1");
        eater2 = hardwareMap.get(DcMotor.class, "eater2");

        ticks_per_rotation_1 = eater1.getMotorType().getTicksPerRev();
        ticks_per_rotation_2 = eater2.getMotorType().getTicksPerRev();

        //Test 1 : Get the ticks per rotation of the motor
        telemetry.addData("tpr1", ticks_per_rotation_1);
        telemetry.addData("tpr2", ticks_per_rotation_2);
        telemetry.update();

        DcMotor.RunMode mode;

        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        eater1.setMode(mode);
        eater2.setMode(mode);

        eater1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        eater1.setMode(mode);
        eater2.setMode(mode);
    }

    @Override
    public void start() {
        double magnitude = 1;
        eater1.setPower(magnitude * (expand ? -1.0 : 1.0));
        eater2.setPower(magnitude * (expand ? -1.0 : 1.0));
    }

    @Override
    public void loop() {
        // Test 2 : Check the speed of each motor and the accuracy of the encoder
    }
}
