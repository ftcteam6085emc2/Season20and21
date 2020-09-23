 package org.firstinspires.ftc.teamcode.Season20and21.code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//This is NOT an OpMode

 public class HWMapDuck {
     /* Public OpMode members. */
     public DcMotor Left   = null;
     public DcMotor Right  = null;

     /* local OpMode members. */
     HardwareMap hwMap           =  null;
     private ElapsedTime period  = new ElapsedTime();

     /* Constructor */
     public HWMapDuck(){

     }

     /* Initialize standard Hardware interfaces */
     public void init(HardwareMap ahwMap) {
         // Save reference to Hardware map
         hwMap = ahwMap;

         // Define and Initialize Motors
         Left   = hwMap.dcMotor.get("left");
         Right  = hwMap.dcMotor.get("right");

         Left.setPower(0);
         Right.setPower(0);
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
