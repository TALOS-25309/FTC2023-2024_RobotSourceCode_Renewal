package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ServoHW;

public class AirplanePart extends Part {
    double airplaneInitPos = 0.9;
    double airplaneFlyPos = 0.4;
    ServoHW airplane;
    public enum Command implements RobotCommand {
        FLY
    }
    public AirplanePart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.airplane = new ServoHW("airplane", hwm, tel);
        this.hardware_manager.registerHardware(airplane);

        this.airplane.setInitialPosition(airplaneInitPos);
    }
    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (cmd == Command.FLY) {
            switch (this.step) {
                case 0:
                    airplane.moveWithInterval(airplaneFlyPos,1000);
                    break;
                case 1:
                    this.finishStep();
                    break;
            }
        }
    }
}
