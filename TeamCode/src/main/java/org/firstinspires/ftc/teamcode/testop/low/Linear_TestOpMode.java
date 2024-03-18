// Linear Test Code

package org.firstinspires.ftc.teamcode.testop.low;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Linear_TestOp", group = "Low")
public class Linear_TestOpMode extends OpMode {
    DcMotor linear1, linear2;
    double ticks_per_rotation_1, ticks_per_rotation_2;
    boolean is_completed_1 = false, is_completed_2 = false;
    boolean expand = true;

    @Override
    public void init() {
        linear1 = hardwareMap.get(DcMotor.class, "linear1");
        linear2 = hardwareMap.get(DcMotor.class, "linear2");

        ticks_per_rotation_1 = linear1.getMotorType().getTicksPerRev();
        ticks_per_rotation_2 = linear2.getMotorType().getTicksPerRev();

        //Test 1 : Get the ticks per rotation of the motor
        telemetry.addData("tpr1", ticks_per_rotation_1);
        telemetry.addData("tpr2", ticks_per_rotation_2);
        telemetry.update();

        DcMotor.RunMode mode;

        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        linear1.setMode(mode);
        linear2.setMode(mode);

        linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        linear1.setMode(mode);
        linear2.setMode(mode);
    }

    @Override
    public void start() {
        double magnitude = 0.3;
        linear1.setPower(magnitude * (expand ? -1.0 : 1.0));
        linear2.setPower(magnitude * (expand ? -1.0 : 1.0));
    }

    @Override
    public void loop() {
        // Test 2 : Check the speed of each motor and the accuracy of the encoder
        double target = 500; //MAX : 4000
        if (Math.abs(linear1.getCurrentPosition()) > target) {
            linear1.setPower(0);
            is_completed_1 = true;
        }
        if (Math.abs(linear2.getCurrentPosition()) > target) {
            linear2.setPower(0);
            is_completed_2 = true;
        }
        if (is_completed_1 && Math.abs(linear1.getCurrentPosition()) < target)
            linear1.setPower(0.1 * (expand ? -1.0 : 1.0));
        if (is_completed_2 && Math.abs(linear2.getCurrentPosition()) < target)
            linear2.setPower(0.1 * (expand ? -1.0 : 1.0));
    }
}
