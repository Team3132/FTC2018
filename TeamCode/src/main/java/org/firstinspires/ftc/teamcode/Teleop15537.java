/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="15537 Teleop", group="Iterative Opmode")
public class Teleop15537 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor driveLeftFront;
    private DcMotor driveRightFront;
    private DcMotor driveLeftBack;
    private DcMotor driveRightBack;
    private DcMotor liftMotor;
    private Lift15537 lift;
    private Servo marker;
    private RevTouchSensor liftLimitSwitch;

    private static int LIFT_TOP = 5000; // Lift Max Height

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        driveLeftFront = hardwareMap.get(DcMotor.class, "driveLeftFront");
        driveRightFront = hardwareMap.get(DcMotor.class, "driveRightFront");
        driveLeftBack = hardwareMap.get(DcMotor.class, "driveLeftBack");
        driveRightBack = hardwareMap.get(DcMotor.class, "driveRightBack");
        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        marker = hardwareMap.get(Servo.class, "marker");

        liftLimitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        driveLeftFront.setDirection(DcMotor.Direction.REVERSE);
        driveLeftBack.setDirection(DcMotor.Direction.REVERSE);
        driveRightFront.setDirection(DcMotor.Direction.FORWARD);
        driveRightBack.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        lift = new Lift15537(liftMotor, liftLimitSwitch);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.a) {
            lift.down();
        } else if (gamepad1.y && lift.getCurrentPosition() <= LIFT_TOP) {
            //lift.setTargetPosition(LIFT_TOP);
            lift.up();
        } else {
            lift.stop();
        }
        lift.update();
        //Arcade Drive Code, does nothing when left & right


        double x = gamepad1.right_stick_x;
        double y = gamepad1.left_stick_y;

        driveLeftBack.setPower(x+y);
        driveLeftFront.setPower(x+y);
        driveRightBack.setPower(x-y);
        driveRightFront.setPower(x-y);

        if (gamepad1.dpad_down) {
            marker.setPosition(0.5);
        } else if (gamepad1.dpad_up) {
            marker.setPosition(1.0);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Lift Encoder", lift.getCurrentPosition());
        telemetry.addData("Set Position", liftMotor.getTargetPosition());
        telemetry.addData("Limit switch", liftLimitSwitch.isPressed());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
