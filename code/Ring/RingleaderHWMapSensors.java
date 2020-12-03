package org.firstinspires.ftc.Season20and21.code.Ring;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//This is NOT an OpMode

public class RingleaderHWMapSensors {
    /* Public OpMode members. */
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight  = null;
    public DcMotor RearLeft  = null;
    public DcMotor RearRight  = null;
    public DcMotor Collector  = null;
    public DcMotor Launcher  = null;
    public DcMotor Elevator = null;

    public BNO055IMU imu = null;

    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;
    public DistanceSensor sensorRangeTop = null;
    public DistanceSensor sensorRangeBottom = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    public RingleaderHWMapSensors(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrontLeft   = hwMap.dcMotor.get("frontLeft");
        FrontRight  = hwMap.dcMotor.get("frontRight");
        RearLeft = hwMap.dcMotor.get("rearLeft");
        RearRight = hwMap.dcMotor.get("rearRight");
        Collector = hwMap.dcMotor.get("collector");
        Launcher = hwMap.dcMotor.get("launcher");
        Elevator = hwMap.dcMotor.get("elevator");

        imu = hwMap.get(BNO055IMU.class, "imu");

        sensorColor = hwMap.get(ColorSensor.class, "colorSensor");
        sensorDistance = hwMap.get(DistanceSensor.class, "colorSensor");
        sensorRangeTop = hwMap.get(DistanceSensor.class, "rangeSensorTop");
        sensorRangeBottom = hwMap.get(DistanceSensor.class, "rangeSensorBottom");

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
    }
}