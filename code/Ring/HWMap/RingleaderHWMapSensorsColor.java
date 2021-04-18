package org.firstinspires.ftc.Season20and21.code.Ring.HWMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//This is NOT an OpMode

public class RingleaderHWMapSensorsColor {
    /* Public OpMode members. */
    public DcMotorEx FrontLeft = null;
    public DcMotorEx FrontRight  = null;
    public DcMotorEx RearLeft  = null;
    public DcMotorEx RearRight  = null;
    public DcMotorEx Collector  = null;
    public DcMotorEx Launcher  = null;
    public DcMotorEx Elevator = null;
    public DcMotorEx Wobble = null;

    public CRServo ServoElevate = null;
    public Servo WobbleServo = null;
    public Servo WobbleRotate = null;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver blinkinLedDriver2;

    public BNO055IMU imu = null;

    public ColorSensor sensorColor = null;
    public ColorSensor ringSensorColor = null;
    public DistanceSensor sensorDistance = null;
    public DistanceSensor ringSensorDistance = null;
    public DistanceSensor sensorRangeTop = null;
    public DistanceSensor sensorRangeBottom = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    public RingleaderHWMapSensorsColor(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrontLeft   = (DcMotorEx) hwMap.dcMotor.get("frontLeft");
        FrontRight  = (DcMotorEx) hwMap.dcMotor.get("frontRight");
        RearLeft = (DcMotorEx) hwMap.dcMotor.get("rearLeft");
        RearRight = (DcMotorEx) hwMap.dcMotor.get("rearRight");
        Collector = (DcMotorEx) hwMap.dcMotor.get("collector");
        Launcher = (DcMotorEx) hwMap.dcMotor.get("launcher");
        Elevator = (DcMotorEx) hwMap.dcMotor.get("elevator");
        Wobble = (DcMotorEx) hwMap.dcMotor.get("wobble");

        ServoElevate = hwMap.crservo.get("Servo");
        WobbleServo = hwMap.servo.get("WobbleServo");
        WobbleRotate = hwMap.servo.get("WobbleRotate");

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver2 = hwMap.get(RevBlinkinLedDriver.class, "blinkin2");

        imu = hwMap.get(BNO055IMU.class, "imu");

        sensorColor = hwMap.get(ColorSensor.class, "colorSensor");
        sensorDistance = hwMap.get(DistanceSensor.class, "colorSensor");
        ringSensorColor = hwMap.get(ColorSensor.class, "ringColorSensor");
        ringSensorDistance = hwMap.get(DistanceSensor.class, "ringColorSensor");
        sensorRangeTop = hwMap.get(DistanceSensor.class, "rangeSensorTop");
        sensorRangeBottom = hwMap.get(DistanceSensor.class, "rangeSensorBottom");
    }
}