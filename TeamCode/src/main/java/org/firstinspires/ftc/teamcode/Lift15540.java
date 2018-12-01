package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Lift15540 {
    private DcMotor motor;
    private RevTouchSensor limitSwitch;

    private static int LIFT_TOP = 5000; // Lift Max Height
    private static double LIFT_SPEED = 1.0;

    private enum State {
        UP,
        DOWN,
        STOP,
    }

    private State state;

    public Lift15540(DcMotor lift, RevTouchSensor limitSwitch) {
        this.motor = lift;
        this.limitSwitch = limitSwitch;
        this.state = state.STOP;

        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoder();
    }

    public void up() {
        if (motor.getCurrentPosition() < LIFT_TOP && this.state != State.UP) {
            this.state = State.UP;
        }
    }

    public void down() {
        if (motor.getCurrentPosition() > 0 && !limitSwitch.isPressed() && this.state != State.DOWN)
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
        if (limitSwitch.isPressed() && this.state == State.DOWN) {
            resetEncoder();
        }

        if (motor.getCurrentPosition() < LIFT_TOP && this.state == State.UP) {
            this.state = State.UP;
        } else if (!limitSwitch.isPressed() && this.state == State.DOWN) {
            this.state = State.DOWN;
        } else {
            this.state = State.STOP;
        }

        switch (state) {
            case UP:
                motor.setPower(LIFT_SPEED);
                break;
            case DOWN:
                motor.setPower(-LIFT_SPEED);
                break;
            case STOP:
                motor.setPower(0);
                break;
        }

    }
}
