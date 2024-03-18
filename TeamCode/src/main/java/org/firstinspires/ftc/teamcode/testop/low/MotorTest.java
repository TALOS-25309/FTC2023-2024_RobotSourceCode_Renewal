// Linear Test Code

package org.firstinspires.ftc.teamcode.testop.low;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTestOp", group = "Low")
public class MotorTest extends OpMode {
    DcMotor motor1;
    double ticks_per_rotation_1, ticks_per_rotation_2;
    boolean is_completed_1 = false, is_completed_2 = false;
    boolean expand = true;

    @Override
    public void init() {
        motor1= hardwareMap.get(DcMotor.class, "motor1");

        ticks_per_rotation_1 = motor1.getMotorType().getTicksPerRev();

        //Test 1 : Get the ticks per rotation of the motor
        telemetry.addData("tpr1", ticks_per_rotation_1);
        telemetry.update();

        DcMotor.RunMode mode;

        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        motor1.setMode(mode);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        motor1.setMode(mode);
    }

    @Override
    public void start() {
        double magnitude = 0.5;
        motor1.setPower(magnitude * (expand ? -1.0 : 1.0));
    }

    @Override
    public void loop() {
        // Test 2 : Check the speed of each motor and the accuracy of the encoder
    }
}
