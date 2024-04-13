package org.firstinspires.ftc.teamcode.mainop.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DistSensorHW;
import org.firstinspires.ftc.teamcode.part.BucketPart;
import org.firstinspires.ftc.teamcode.part.EaterPart;
import org.firstinspires.ftc.teamcode.part.LinearPart;
import org.firstinspires.ftc.teamcode.part.Part;
import org.firstinspires.ftc.teamcode.part.RobotCommand;
import org.firstinspires.ftc.teamcode.part.AutoWheelPart;


public abstract class AutoOpMode_Main extends LinearOpMode {
    private LinearPart linear_part;
    protected AutoWheelPart awheel_part;
    private BucketPart bucket_part;
    private EaterPart eater_part;
    private double pixelDist = 0;

    private DistSensorHW
            dist_front_1, dist_front_2,
            dist_right_1, dist_right_2, dist_right_3, dist_right_4;

    protected RobotCommand current_command = Part.Command.NONE;
    protected int step = 0;
    private boolean finish = true;
    private long delay_time = 0;
    private String pixelPos = "default";

    protected abstract void setRobotStartPosition();

    public boolean isObjectDetected(String position) {
        double detectDist = 4;
        switch (position) {
            case "front":
                if (dist_front_1.getDistance() < detectDist || dist_front_2.getDistance() < detectDist) {
                    return true;
                }
                break;
            case "right":
                if (dist_right_1.getDistance() < detectDist || dist_right_2.getDistance() < detectDist
                        || dist_right_3.getDistance() < detectDist || dist_right_4.getDistance() < detectDist) {
                    return true;
                }
                break;
            case "left":
                if (dist_right_1.getDistance() > detectDist && dist_right_2.getDistance() > detectDist
                        && dist_front_1.getDistance() > detectDist && dist_front_2.getDistance() > detectDist
                        && dist_right_3.getDistance() > detectDist && dist_right_4.getDistance() > detectDist) {
                    return true;
                }
                break;
        }
        return false;
    }

    public void initAutoOp() {
        this.linear_part = new LinearPart(hardwareMap, telemetry);
        this.awheel_part = new AutoWheelPart(hardwareMap, telemetry);
        this.bucket_part = new BucketPart(hardwareMap, telemetry);
        this.eater_part = new EaterPart(hardwareMap, telemetry);

        dist_front_1 = new DistSensorHW("dist_front_1", hardwareMap, telemetry);
        dist_front_2 = new DistSensorHW("dist_front_2", hardwareMap, telemetry);
        dist_right_1 = new DistSensorHW("dist_right_1", hardwareMap, telemetry);
        dist_right_2 = new DistSensorHW("dist_right_2", hardwareMap, telemetry);
        dist_right_3 = new DistSensorHW("dist_right_3", hardwareMap, telemetry);
        dist_right_4 = new DistSensorHW("dist_right_4", hardwareMap, telemetry);
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
            this.awheel_part.update();
            this.bucket_part.update();
            this.eater_part.update();
            this.update();


            this.telemetry.addData("Left : ", isObjectDetected("left"));
            this.telemetry.addData("Right : ", isObjectDetected("right"));
            this.telemetry.addData("Front : ", isObjectDetected("front"));

            this.telemetry.addData("SensorValueF1 : ", this.dist_front_1.getDistance());
            this.telemetry.addData("SensorValueF2 : ", this.dist_front_2.getDistance());
            this.telemetry.addData("SensorValueR1 : ", this.dist_right_1.getDistance());
            this.telemetry.addData("SensorValueR2 : ", this.dist_right_2.getDistance());
            this.telemetry.addData("SensorValueR3 : ", this.dist_right_3.getDistance());
            this.telemetry.addData("SensorValueR4 : ", this.dist_right_4.getDistance());


            this.telemetry.update();

            /*
            if (this.isFinished()) {
                if (++procedure_step >= command_procedure.length) break;
                this.startStep(command_procedure[procedure_step]);
            }
            */
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
        RobotCommand cmd = this.current_command;
        if (cmd == Command.RESET) {
            switch (this.step) {
                case 0:
                    bucket_part.startStep(BucketPart.Command.BUCKET_DOWN);
                    linear_part.startStep(LinearPart.Command.MOVE_ORIGINAL_POSITION);
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
                    awheel_part.startStep(AutoWheelPart.Command. MOVE_DETECT_POS);
                    break;
                case 1:
                    if (isObjectDetected("front")) {
                        pixelPos = "front";
                        pixelDist = 1;
                    } else if (isObjectDetected("right")) {
                        pixelPos = "right";
                        pixelDist = -1.6;
                    } else {
                        pixelPos = "left";
                        pixelDist = 2;
                    }

                case 4:
                    // turn to position to drop pixel
                    if (pixelPos == "left") {
                        // turn left
                        awheel_part.startStep(AutoWheelPart.Command.DROP_LEFT);
                    } else if (pixelPos == "right") {
                        // turn right
                        awheel_part.startStep(AutoWheelPart.Command.DROP_RIGHT);
                    } else {
                        // front
                        awheel_part.startStep(AutoWheelPart.Command.DROP_FRONT);
                    }
                    break;
                case 5:
                    eater_part.startStep(EaterPart.Command.MOVE_DOWN);
                    delayTime(2000);
                    break;
                case 6:
                    eater_part.startStep(EaterPart.Command.STOP);
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
                    linear_part.startStep(LinearPart.Command.MOVE_AUTO_POSITION);
                    break;
                case 4:
                    // drop pixel
                    bucket_part.startStep(BucketPart.Command.BUCKET_OUT);
                    delayTime(2000);
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
                    linear_part.startStep(LinearPart.Command.MOVE_AUTO_ORIGINAL_POSITION);
                    break;
                case 2:
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
        telemetry.update();
        if(this.linear_part.isFinished() && this.eater_part.isFinished() && this.awheel_part.isFinished() && this.bucket_part.isFinished() && System.currentTimeMillis() > this.delay_time) {
            this.changeToTheNextStep();
        }
    }

    // Check that the assigned command is finished
    public boolean isFinished(){
        return this.finish;
    }

    private void finishCommand() { this.finish = true; }
}


