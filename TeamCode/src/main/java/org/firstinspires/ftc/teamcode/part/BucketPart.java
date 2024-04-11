package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.CRServoHW;
import org.firstinspires.ftc.teamcode.hardware.ServoHW;

public class BucketPart extends Part {
    private final ServoHW bucket_wrist;
    private final CRServoHW bucket_rotor;

    private final double CRServo_speed = 1.0;
    private final double low_position = 0.27;
    private final double high_position = 0.55;

    private boolean is_bucket_up = false;

    public enum Command implements RobotCommand {
        BUCKET_UP,
        BUCKET_DOWN,
        BUCKET_IN,
        BUCKET_OUT,
        STOP
    }

    public BucketPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.bucket_wrist = new ServoHW("bucket_wrist", hwm, tel);
        this.bucket_rotor = new CRServoHW("bucket_rotor", hwm, tel);

        this.bucket_wrist.setDirection(Servo.Direction.REVERSE);
        this.bucket_rotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.bucket_wrist.setInitialPosition(low_position);

        this.hardware_manager.registerHardware(this.bucket_wrist);
        this.hardware_manager.registerHardware(this.bucket_rotor);
    }

    public boolean isBucketUp() {
        return this.is_bucket_up;
    }

    @Override
    public void nextStep() {
        RobotCommand cmd = this.current_command;
        if(cmd == Command.BUCKET_UP) {
            switch(this.step) {
                case 0:
                    this.is_bucket_up = true;
                    this.bucket_rotor.stop();
                    this.bucket_wrist.moveWithInterval(high_position, 300);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.BUCKET_DOWN) {
            switch(this.step) {
                case 0:
                    this.bucket_rotor.stop();
                    this.bucket_wrist.moveWithInterval(low_position, 300);
                    break;
                case 1:
                    this.is_bucket_up = false;
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.BUCKET_IN) {
            switch(this.step) {
                case 0:
                    this.bucket_rotor.move(CRServo_speed);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.BUCKET_OUT) {
            switch(this.step) {
                case 0:
                    this.bucket_rotor.move(-CRServo_speed);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        } else if (cmd == Command.STOP) {
            switch(this.step) {
                case 0:
                    this.bucket_rotor.stop();
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
    }

    @Override
    public void emergencyStop() {
        this.bucket_rotor.stop();
        this.bucket_wrist.stop();
    }
}
