package org.firstinspires.ftc.teamcode.mainop.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DistSensorHW;
import org.firstinspires.ftc.teamcode.part.BucketPart;
import org.firstinspires.ftc.teamcode.part.LinearPart;
import org.firstinspires.ftc.teamcode.part.Part;
import org.firstinspires.ftc.teamcode.part.RobotCommand;
import org.firstinspires.ftc.teamcode.part.WheelPart;
import org.firstinspires.ftc.teamcode.part.AutoWheelPart;


@Autonomous(name = "[BLUE] AutoOpMode", group = "")
public class AutoOpMode extends LinearOpMode {
    public LinearPart linear_part;
    public WheelPart wheel_part;
    public AutoWheelPart awheel_part;
    public BucketPart bucket_part;
    public DistSensorHW dist_front_1, dist_front_2, dist_right_1, dist_right_2;

    protected RobotCommand current_command = Part.Command.NONE;
    protected int step = 0;
    private boolean finish = true;
    private long delay_time = 0;

    public int robotPixelPos = 0;
    public String pixelPos = "default";

    public boolean isObjectDetected(String position) {
        double detectDist = 3;
        switch (position) {
            case "front":
                if (dist_front_1.getDistance() < detectDist || dist_front_2.getDistance() < detectDist) {
                    return true;
                }
                break;
            case "right":
                if (dist_right_1.getDistance() < detectDist || dist_right_2.getDistance() < detectDist) {
                    return true;
                }
                break;
            case "left":
                if (dist_right_1.getDistance() > detectDist && dist_right_2.getDistance() > detectDist && dist_front_1.getDistance() > detectDist && dist_front_2.getDistance() > detectDist) {
                    return true;
                }
                break;
        }
        return false;
    }

    public void initAutoOp() {
        this.linear_part = new LinearPart(hardwareMap, telemetry);
        this.wheel_part = new WheelPart(hardwareMap, telemetry);

        dist_front_1 = new DistSensorHW("dist_front_1", hardwareMap, telemetry);
        dist_front_2 = new DistSensorHW("dist_front_2", hardwareMap, telemetry);
        dist_right_1 = new DistSensorHW("dist_right_1", hardwareMap, telemetry);
        dist_right_2 = new DistSensorHW("dist_right_2", hardwareMap, telemetry);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // init
        this.initAutoOp();
        waitForStart();

        // start
        // startStep(Command.DETECT_PIXELS);

        // Command Procedure
        Command[] command_procedure = {
                Command.RESET,
                /* 1. */ Command.DETECT_PIXELS,
                /* 2. */ Command.DROP_PIXELS,
                /* 3. */ Command.PARK
        };
        int procedure_step = -1;

        // loop
        while (true) {
            this.linear_part.update();
            this.wheel_part.update();
            this.update();

            this.telemetry.update();

            if (this.isFinished()) {
                if (++procedure_step >= command_procedure.length) break;
                this.startStep(command_procedure[procedure_step]);
            }
        }
    }

    public enum Command implements RobotCommand {
        DETECT_PIXELS,
        DROP_PIXELS,
        PARK,
        RESET
    }

    // Begin the specific step
    public void startStep(RobotCommand cmd){
        if(this.isFinished()) {
            this.current_command = cmd;
            this.step = 0;
            this.finish = false;
            this.nextStep();
        }
    }

    public void nextStep() {
        double pixelDist = 0;
        RobotCommand cmd = this.current_command;
        if (cmd == Command.RESET) {
            switch (this.step) {
                case 0:
                    break;
                case 1:
                    this.finishCommand();
                    break;
            }
        } else if (cmd == Command.DETECT_PIXELS) {
            switch (this.step) {
                case 0:
                    // move until object is detected
                    // use color sensor
                    awheel_part.startStep(AutoWheelPart.Command.MOVE_DETECT_POS);
                    break;
                case 1:
                    if (isObjectDetected("left")) {
                        pixelPos = "left";
                        pixelDist = 2;
                    } else if (isObjectDetected("right")) {
                        pixelPos = "right";
                        pixelDist = 1;
                    } else {
                        pixelPos = "front";
                        pixelDist = 0;
                    }
                    break;
                case 2:
                    // turn to position to drop pixel
                    if (pixelPos == "left") {
                        // turn left
                        awheel_part.startStep(AutoWheelPart.Command.DROP_LEFT);
                    } else if (pixelPos == "right") {
                        // turn right
                        awheel_part.startStep(AutoWheelPart.Command.DROP_RIGHT);
                    }
                    break;
                case 3:
                    // drop pixel
                    break;
                case 4:
                    // turn to original position
                    if (pixelPos == "front") {
                        // turn left
                        awheel_part.startStep(AutoWheelPart.Command.DROP_FRONT);
                    }
                    else if (pixelPos == "right") {
                        // turn 180deg
                        awheel_part.startStep(AutoWheelPart.Command.DROP_RIGHT);
                    }
                    break;
                case 5:
                    this.finishCommand();
                    break;
            }
        } else if (cmd == Command.DROP_PIXELS) {
            switch (this.step) {
                case 0:
                    // move until backdrop is detected
                    awheel_part.startStep(AutoWheelPart.Command.MOVE_BACKDROP);
                    break;
                case 1:
                    // move to pixel position

                    awheel_part.pixelPos = (0.2 * pixelDist);
                    awheel_part.startStep(AutoWheelPart.Command.MOVE_PIXEL);
                    break;
                case 2:
                    // turn wrist
                    bucket_part.startStep(BucketPart.Command.BUCKET_UP);
                    break;
                case 3:
                    // extend linear
                    linear_part.startStep(LinearPart.Command.MOVE_DROP_POSITION);
                    break;
                case 4:
                    // drop pixel
                    bucket_part.startStep(BucketPart.Command.BUCKET_OUT);
                    break;
                case 5:
                    // move to park position
                    break;
                case 6:
                    this.finishCommand();
                    break;
            }
        } else if (cmd == Command.PARK) {
            switch (this.step) {
                case 0:
                    // move left
                    // move forward
                    awheel_part.startStep(AutoWheelPart.Command.PARK);
                    break;
                case 1:
                    this.finishCommand();
                    break;
            }
        }
    }

    protected void delayTime(long delay) {
        delay_time = System.currentTimeMillis() + delay;
    }

    // Change to the next step
    private void changeToTheNextStep() {
        this.step++;
        this.nextStep();
    }

    // Update the hardware objects of the part and check the step was finished
    public void update(){
        if(this.linear_part.isFinished() && this.wheel_part.isFinished() && System.currentTimeMillis() > this.delay_time) {
            this.changeToTheNextStep();
        }
    }

    // Check that the assigned command is finished
    public boolean isFinished(){
        return this.finish;
    }

    private void finishCommand() { this.finish = true; }
}


