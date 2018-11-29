package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Lift31320 {
    private DcMotor lift;
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
        this.lift = lift;
        this.limitSwitch = limitSwitch;
        this.state = state.STOP;

        this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (limitSwitch.getState()) {
            lift.setPower(0.2);
        }

        while (!limitSwitch.getState()) {
            lift.setPower(-0.2);
        }
        resetEncoder();
    }
    public void up() {
        if (lift.getCurrentPosition() < LIFT_TOP && this.state == State.UP) {
            this.state = State.UP;
        }
    }

    public void down() {
        if (lift.getCurrentPosition() > 0 && this.state == State.DOWN)
            this.state = State.DOWN;
    }

    public void stop() {
        this.state = State.STOP;
    }

    public void resetEncoder(){
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getCurrentPosition () {
        return this.lift.getCurrentPosition();
    }

    public void update() {
        if (lift.getCurrentPosition() < LIFT_TOP && this.state == State.UP) {
            this.state = State.UP;
        } else if (limitSwitch.getState() && this.state == State.DOWN) {
            this.state = State.DOWN;
        } else {
            this.state = State.STOP;
        }

        switch (state) {
            case UP:
                lift.setPower(LIFT_SPEED);
                break;
            case DOWN:
                lift.setPower(LIFT_SPEED);
                break;
            case STOP:
                lift.setPower(LIFT_SPEED);
                break;
        }

    }
}
