package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Hardware {
    public DcMotor driveFrontLeft;
    public DcMotor driveBackLeft;
    public DcMotor driveFrontRight;
    public DcMotor driveBackRight;
    public BNO055IMU imu;

    public Hardware(HardwareMap hardwareMap) {
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveLeftFront"); // 0
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveLeftBack"); // 1
        driveFrontRight = hardwareMap.get(DcMotor.class, "driveRightFront"); // 2
        driveBackRight = hardwareMap.get(DcMotor.class, "driveRightBack"); // 3


    }
}
