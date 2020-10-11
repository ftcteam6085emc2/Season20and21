package org.firstinspires.ftc.teamcode.Season20and21.code.Touchdown;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.VuforiaWebcam;

//This is NOT an OpMode

public class HWMapTouchdown {
    /* Public OpMode members. */
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor RearLeft = null;
    public DcMotor RearRight = null;
    public DcMotor SpinLeft = null;
    public DcMotor SpinRight = null;
    public DcMotor ArmLeft = null;
    public DcMotor ArmRight = null;

    public Servo GrabLeft = null;
    public Servo GrabRight = null;
    public Servo FoundationServoLeft = null;
    public Servo FoundationServoRight = null;

    public WebcamName Webcam1 = null;

    public BNO055IMU imu = null;

    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    public RevBlinkinLedDriver RaveShadowLegends = null;

    public TouchSensor touchSensor = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HWMapTouchdown() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FrontLeft = hwMap.dcMotor.get("FrontLeft");
        FrontRight = hwMap.dcMotor.get("FrontRight");
        RearLeft = hwMap.dcMotor.get("RearLeft");
        RearRight = hwMap.dcMotor.get("RearRight");
        SpinLeft = hwMap.get(DcMotor.class, "SpinLeft");
        SpinRight = hwMap.get(DcMotor.class, "SpinRight");
        ArmLeft = hwMap.get(DcMotor.class, "ArmLeft");
        ArmRight = hwMap.get(DcMotor.class, "ArmRight");

        GrabLeft = hwMap.get(Servo.class, "GrabLeft");
        GrabRight = hwMap.get(Servo.class, "GrabRight");
        FoundationServoLeft = hwMap.get(Servo.class, "FoundationServoLeft");
        FoundationServoRight = hwMap.get(Servo.class, "FoundationServoRight");

        Webcam1 = hwMap.get(WebcamName.class, "Webcam 1");

        imu = hwMap.get(BNO055IMU.class, "imu");

        sensorColor = hwMap.get(ColorSensor.class, "colorSensor");
        sensorDistance = hwMap.get(DistanceSensor.class, "colorSensor");

        RaveShadowLegends = hwMap.get(RevBlinkinLedDriver.class, "RaveShadowLegends");

        touchSensor = hwMap.get(TouchSensor.class, "touchSensor");

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }


}