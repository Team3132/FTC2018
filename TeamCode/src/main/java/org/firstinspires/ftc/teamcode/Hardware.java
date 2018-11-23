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
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");


    }
}
