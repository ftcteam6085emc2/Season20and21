package org.firstinspires.ftc.Season20and21.code.Ring.HWMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//This is NOT an OpMode

public class RingleaderHWMapSensorsColor {
    /* Public OpMode members. */
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight  = null;
    public DcMotor RearLeft  = null;
    public DcMotor RearRight  = null;
    public DcMotor Collector  = null;
    public DcMotor Launcher  = null;
    public DcMotor Elevator = null;
    public DcMotor Wobble = null;

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
        FrontLeft   = hwMap.dcMotor.get("frontLeft");
        FrontRight  = hwMap.dcMotor.get("frontRight");
        RearLeft = hwMap.dcMotor.get("rearLeft");
        RearRight = hwMap.dcMotor.get("rearRight");
        Collector = hwMap.dcMotor.get("collector");
        Launcher = hwMap.dcMotor.get("launcher");
        Elevator = hwMap.dcMotor.get("elevator");
        Wobble = hwMap.dcMotor.get("wobble");

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

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
    }
}