package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DcMotorHW;
import org.firstinspires.ftc.teamcode.hardware.MagSensorHW;

public class DdalggakPart extends Part {
    DcMotorHW ddalggak1, ddalggak2;

    private boolean isDdalggakOpen = true;

    private long time_limit = 2000;

    private boolean is_usable_1 = true;
    private boolean is_usable_2 = true;

    public enum Command implements RobotCommand {
        OPEN_OR_CLOSE_DDALGGAK,
        OPEN_OR_CLOSE_DDALGGAK_GENTLY,
        RESET_DDALGGAK,
        CLOSE_PERFECTLY
    }

    // Constructor
    public DdalggakPart(HardwareMap hwm, Telemetry tel) {
        super(hwm, tel);

        this.ddalggak1 = new DcMotorHW("ddalggak1", hwm, tel);
        this.ddalggak2 = new DcMotorHW("ddalggak2", hwm, tel);

        ddalggak1.setUsingBrake(true).setUsingFixation(false).setUsingEncoder(false);
        ddalggak2.setUsingBrake(true).setUsingFixation(false).setUsingEncoder(false);

        this.hardware_manager.registerHardware(ddalggak1).registerHardware(ddalggak2);
    }

    public void inactiveLeftDdalggak() {
        this.is_usable_1 = false;
    }

    public void inactiveRightDdalggak() {
        this.is_usable_2 = false;
    }

    public void activateAllDdalggak() {
        this.is_usable_1 = true;
        this.is_usable_2 = true;
    }

    private void setCloseDirection() {
        ddalggak1.setDirection(DcMotorSimple.Direction.REVERSE);
        ddalggak2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void setOpenDirection() {
        ddalggak1.setDirection(DcMotorSimple.Direction.FORWARD);
        ddalggak2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean isOpenState() {
        return this.isDdalggakOpen;
    }

    @Override
    protected void nextStep() {
        RobotCommand cmd = this.current_command;
        if (this.current_command == Command.OPEN_OR_CLOSE_DDALGGAK) {
            if (!isDdalggakOpen) {
                /*
                switch (this.step) {
                    case 0:
                        this.setOpenDirection();
                        ddalggak1.move(0.8, 75);
                        ddalggak2.move(0.8, 75);
                        int trigger = 0;
                        while (trigger != 3) {
                            ddalggak1.update();
                            ddalggak2.update();
                            if (trigger != 1 && ddalggak1.isFinished()) { ddalggak1.stop(); trigger |= 1;}
                            if (trigger != 2 && ddalggak2.isFinished()) { ddalggak2.stop(); trigger |= 2;}
                        }
                        isDdalggakOpen = true;
                        this.finishStep();
                        break;
                }
                */
                this.finishStep();
                this.startStep(Command.RESET_DDALGGAK);
            }
            else {
                switch (this.step) {
                    case 0 :
                        this.setCloseDirection();
                        if(is_usable_1) ddalggak1.move(0.8, 60);
                        if(is_usable_2) ddalggak2.move(0.8, 60);
                        int trigger = 0;
                        long begin_time = System.currentTimeMillis();
                        while (trigger != 3) {
                            ddalggak1.update();
                            ddalggak2.update();
                            if (trigger != 1 && ddalggak1.isFinished()) { ddalggak1.stop(); trigger |= 1;}
                            if (trigger != 2 && ddalggak2.isFinished()) { ddalggak2.stop(); trigger |= 2;}
                            if (System.currentTimeMillis() - begin_time > this.time_limit) {
                                break;
                            }
                        }
                        if (trigger == 3) {
                            isDdalggakOpen = false;
                            this.finishStep();
                        }
                        else {
                            this.finishStep();
                            this.startStep(Command.RESET_DDALGGAK);
                        }
                        break;
                }
            }
        }
        if (this.current_command == Command.OPEN_OR_CLOSE_DDALGGAK_GENTLY) {
            if (!isDdalggakOpen) {
                /*
                switch (this.step) {
                    case 0:
                        this.setOpenDirection();
                        ddalggak1.move(0.2, 75);
                        ddalggak2.move(0.2, 75);
                        int trigger = 0;
                        while (trigger != 3) {
                            ddalggak1.update();
                            ddalggak2.update();
                            if (trigger != 1 && ddalggak1.isFinished()) { ddalggak1.stop(); trigger |= 1;}
                            if (trigger != 2 && ddalggak2.isFinished()) { ddalggak2.stop(); trigger |= 2;}
                        }
                        isDdalggakOpen = true;
                        this.finishStep();
                        break;
                }
                */
                this.finishStep();
                this.startStep(Command.RESET_DDALGGAK);
            }
            else {
                switch (this.step) {
                    case 0 :
                        this.setCloseDirection();
                        if (is_usable_1) ddalggak1.move(0.2, 85);
                        if (is_usable_2) ddalggak2.move(0.2, 85);
                        int trigger = 0;
                        long begin_time = System.currentTimeMillis();
                        while (trigger != 3) {
                            ddalggak1.update();
                            ddalggak2.update();
                            if (trigger != 1 && ddalggak1.isFinished()) { ddalggak1.stop(); trigger |= 1;}
                            if (trigger != 2 && ddalggak2.isFinished()) { ddalggak2.stop(); trigger |= 2;}
                            if (System.currentTimeMillis() - begin_time > this.time_limit) {
                                break;
                            }
                        }
                        if (trigger == 3) {
                            isDdalggakOpen = false;
                            this.finishStep();
                            this.setOpenDirection();
                            if (is_usable_1) ddalggak1.move(0.2, 10);
                            if (is_usable_2) ddalggak2.move(0.2, 10);
                            trigger = 0;
                            begin_time = System.currentTimeMillis();
                            while (trigger != 3) {
                                ddalggak1.update();
                                ddalggak2.update();
                                if (trigger != 1 && ddalggak1.isFinished()) { ddalggak1.stop(); trigger |= 1;}
                                if (trigger != 2 && ddalggak2.isFinished()) { ddalggak2.stop(); trigger |= 2;}
                                if (System.currentTimeMillis() - begin_time > this.time_limit) {
                                    break;
                                }
                            }
                        }
                        else {
                            this.finishStep();
                            this.startStep(Command.RESET_DDALGGAK);
                        }
                        break;
                }
            }
        }
        if (this.current_command == Command.RESET_DDALGGAK) {
            switch (this.step) {
                case 0 :
                    this.isDdalggakOpen = true;
                    if(is_usable_1) {
                        this.setOpenDirection();
                        ddalggak1.moveUntilStuck(0.1);
                        ddalggak2.stop();
                    }
                    break;
                case 1 :
                    if(is_usable_2) {
                        this.setOpenDirection();
                        ddalggak1.stop();
                        ddalggak2.moveUntilStuck(0.1);
                    }
                    break;
                case 2:
                    ddalggak1.stop();
                    ddalggak2.stop();
                    this.finishStep();
                    break;
            }
        }
        if (this.current_command == Command.CLOSE_PERFECTLY) {
            switch (this.step) {
                case 0 :
                    this.isDdalggakOpen = true;
                    if(is_usable_1) {
                        this.setCloseDirection();
                        ddalggak1.moveUntilStuck(0.1);
                        ddalggak2.stop();
                    }
                    break;
                case 1 :
                    if(is_usable_2) {
                        this.setCloseDirection();
                        ddalggak1.stop();
                        ddalggak2.moveUntilStuck(0.1);
                    }
                    break;
                case 2:
                    ddalggak1.stop();
                    ddalggak2.stop();
                    this.finishStep();
                    break;
            }
        }
    }
}
