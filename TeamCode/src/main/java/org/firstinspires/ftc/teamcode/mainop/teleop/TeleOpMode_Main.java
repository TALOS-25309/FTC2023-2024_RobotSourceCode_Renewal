package org.firstinspires.ftc.teamcode.mainop.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.part.AirplanePart;
import org.firstinspires.ftc.teamcode.part.BucketPart;
import org.firstinspires.ftc.teamcode.part.EaterPart;
import org.firstinspires.ftc.teamcode.part.LinearPart;
import org.firstinspires.ftc.teamcode.part.WheelPart;

public abstract class TeleOpMode_Main extends OpMode {
    private LinearPart linear_part;
    private WheelPart wheel_part;
    private BucketPart bucket_part;
    private EaterPart eater_part;
    private AirplanePart airplane_part;

    protected double getX() {
        return 0;
    }

    protected double getY() {
        return 0;
    }

    private boolean is_emergency_mode = false;

    private String prev_gamepad1_state = "";
    private String prev_gamepad2_state = "";

    @Override
    public void init() {
        this.linear_part = new LinearPart(hardwareMap, telemetry);
        this.wheel_part = new WheelPart(hardwareMap, telemetry);
        this.bucket_part = new BucketPart(hardwareMap, telemetry);
        this.eater_part = new EaterPart(hardwareMap, telemetry);
        this.airplane_part = new AirplanePart(hardwareMap, telemetry);

        this.wheel_part.setTeleWheelSpeed();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // Update the parts
        this.linear_part.update();
        this.wheel_part.update();
        this.bucket_part.update();
        this.eater_part.update();
        this.airplane_part.update();

        if (this.is_emergency_mode) {
            this.processGamepad1WhenEmergency();
            this.processGamepad2WhenEmergency();
        } else {
            this.processGamepad1();
            this.processGamepad2();
        }

        this.emergencyOnOFF();

        //this.telemetry.update();
    }

    private void processGamepad1() {
        if (gamepad1.dpad_up) {
            this.wheel_part.startStep(WheelPart.Command.MOVE_RIGHT);
        } else if (gamepad1.dpad_down) {
            this.wheel_part.startStep(WheelPart.Command.MOVE_LEFT);
        } else if (gamepad1.dpad_left) {
            this.wheel_part.startStep(WheelPart.Command.MOVE_FORWARD);
        } else if (gamepad1.dpad_right) {
            this.wheel_part.startStep(WheelPart.Command.MOVE_BACKWARD);
        } else if (gamepad1.left_bumper) {
            this.wheel_part.startStep(WheelPart.Command.TURN_LEFT);
        } else if (gamepad1.right_bumper) {
            this.wheel_part.startStep(WheelPart.Command.TURN_RIGHT);
        } else if (gamepad1.left_trigger > 0.1) {
            this.wheel_part.move(gamepad1.left_trigger * 0.8, WheelPart.Direction.TurnLeft);
        } else if (gamepad1.right_trigger > 0.1) {
            this.wheel_part.move(gamepad1.right_trigger * 0.8, WheelPart.Direction.TurnRight);
        } else {
            this.wheel_part.moveFreely(this.getX(),this.getY());
        }
    }

    private void processGamepad2() {
        // Linear Up and Down
        if (gamepad2.dpad_up) {
            this.linear_part.startStep(LinearPart.Command.MOVE_UP);
        } else if (gamepad2.dpad_down) {
            this.linear_part.startStep(LinearPart.Command.MOVE_DOWN);
        } else if (gamepad2.left_bumper && gamepad2.right_bumper) {
            this.linear_part.startStep(LinearPart.Command.MOVE_DOWN_POWERFUL);
        } else {
            this.linear_part.startStep(LinearPart.Command.STOP);
        }

        // For optimization : do not process (start commands) if the gamepad2 state is not changed
        String gamepad2_state = gamepad2.toString();
        if (gamepad2_state.equals(this.prev_gamepad2_state)) return;
        this.prev_gamepad2_state = gamepad2_state;

        // Bucket Up and Down
        if (gamepad2.square) {
            if (this.bucket_part.isBucketUp()) {
                this.bucket_part.startStep(BucketPart.Command.BUCKET_DOWN);
            } else {
                this.bucket_part.startStep(BucketPart.Command.BUCKET_UP);
            }
        } else if (gamepad2.circle) {
            if(!this.bucket_part.isBucketUp()) {
                this.bucket_part.startStep(BucketPart.Command.BUCKET_IN);
            }
        } else if (gamepad2.triangle) {
            if(this.bucket_part.isBucketUp()) {
                this.bucket_part.startStep(BucketPart.Command.BUCKET_OUT);
            }
        } else {
            this.bucket_part.startStep(BucketPart.Command.STOP);
        }

        if (gamepad2.circle && !this.bucket_part.isBucketUp() && this.linear_part.isLinearDown()) {
            this.eater_part.startStep(EaterPart.Command.MOVE_UP);
        } else if (gamepad2.cross) {
            this.eater_part.startStep(EaterPart.Command.MOVE_DOWN);
        } else {
            this.eater_part.startStep(EaterPart.Command.STOP);
        }

        // Airplane
        if (gamepad2.left_stick_button || gamepad2.right_stick_button) {
            this.airplane_part.startStep(AirplanePart.Command.FLY);
        }
    }

    private void emergencyOnOFF() {
        if (this.is_emergency_mode) {
            if (gamepad1.touchpad && gamepad2.touchpad) {
                // NORMAL STATE
                this.is_emergency_mode = false;
                gamepad1.setLedColor(0, 0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(1.0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                this.gamepad1.rumble(100);
                this.gamepad2.rumble(100);
            }
        } else {
            if ((gamepad1.right_bumper && gamepad1.right_trigger > 0.5
                    && gamepad1.left_bumper && gamepad1.left_trigger > 0.5)
                    ||(gamepad2.right_bumper && gamepad2.right_trigger > 0.5
                    && gamepad2.left_bumper && gamepad2.left_trigger > 0.5)) {
                // EMERGENCY STATE
                this.is_emergency_mode = true;
                this.linear_part.emergencyStop();
                this.wheel_part.emergencyStop();
                this.bucket_part.emergencyStop();
                this.eater_part.emergencyStop();
                gamepad1.setLedColor(1.0, 0.5, 0, Gamepad.LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(1.0, 0.5, 0, Gamepad.LED_DURATION_CONTINUOUS);
                this.gamepad1.rumble(500);
                this.gamepad2.rumble(500);
            }
        }
    }

    private void processGamepad1WhenEmergency () {

    }

    private void processGamepad2WhenEmergency () {

    }
}
