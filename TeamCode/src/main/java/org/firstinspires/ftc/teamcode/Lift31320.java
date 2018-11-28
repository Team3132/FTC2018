package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Lift31320 {
    private DcMotor motor;
    private DigitalChannel limitSwitch;

    private static int LIFT_TOP = 5000; // Lift Max Height
    private static double LIFT_SPEED = 0.5;

    private enum State {
        UP,
        DOWN,
        STOP,
    }

    private State state;

    public void Lift31320(DcMotor lift, DigitalChannel limitSwitch) {
        this.motor = motor;
        this.limitSwitch = limitSwitch;
        this.state = state.STOP;

        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (limitSwitch.getState()) {
            motor.setPower(0.2);
        }

        while (!limitSwitch.getState()) {
            motor.setPower(-0.2);
        }
        resetEncoder();
    }
    public void up() {
        if (motor.getCurrentPosition() < LIFT_TOP && this.state == State.UP) {
            this.state = State.UP;
        }
    }

    public void down() {
        if (motor.getCurrentPosition() > 0 && this.state == State.DOWN)
            this.state = State.DOWN;
    }

    public void stop() {
        this.state = State.STOP;
    }

    public void resetEncoder(){
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getCurrentPosition () {
        return this.motor.getCurrentPosition();
    }

    public void update() {
        if (motor.getCurrentPosition() < LIFT_TOP && this.state == State.UP) {
            this.state = State.UP;
        } else if (limitSwitch.getState() && this.state == State.DOWN) {
            this.state = State.DOWN;
        } else {
            this.state = State.STOP;
        }

        switch (state) {
            case UP:
                motor.setPower(LIFT_SPEED);
                break;
            case DOWN:
                motor.setPower(LIFT_SPEED);
                break;
            case STOP:
                motor.setPower(LIFT_SPEED);
                break;
        }

    }
}
