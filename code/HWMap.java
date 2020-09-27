 package org.firstinspires.ftc.teamcode.Season20and21.code;

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

//This is NOT an OpMode

 public class HWMap {
     /* Public OpMode members. */
     public DcMotor FrontLeft   = null;
     public DcMotor FrontRight  = null;
     public DcMotor RearLeft  = null;
     public DcMotor RearRight  = null;

     public BNO055IMU imu = null;

     public ColorSensor sensorColor = null;
     public DistanceSensor sensorDistance = null;

     /* local OpMode members. */
     HardwareMap hwMap           =  null;
     private ElapsedTime period  = new ElapsedTime();

     /* Constructor */
     public HWMap(){

     }

     /* Initialize standard Hardware interfaces */
     public void init(HardwareMap ahwMap) {
         // Save reference to Hardware map
         hwMap = ahwMap;

         // Define and Initialize Motors
         FrontLeft   = hwMap.dcMotor.get("motor_1");
         FrontRight  = hwMap.dcMotor.get("motor_0");
         RearLeft = hwMap.dcMotor.get("motor_2");
         RearRight = hwMap.dcMotor.get("motor_3");

         imu = hwMap.get(BNO055IMU.class, "imu");

         sensorColor = hwMap.get(ColorSensor.class, "colorSensor");
         sensorDistance = hwMap.get(DistanceSensor.class, "colorSensor");

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

         long  remaining = periodMs - (long)period.milliseconds();

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
