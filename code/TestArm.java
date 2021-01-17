package org.firstinspires.ftc.Season20and21.code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestArm", group = "Arm")
public class TestArm extends OpMode {
    DcMotor Wobble;
    Servo WobbleServo;
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight  = null;
    public DcMotor RearLeft  = null;
    public DcMotor RearRight  = null;

    public void init(){
        Wobble = hardwareMap.dcMotor.get("wobble");
        WobbleServo = hardwareMap.servo.get("WobbleServo");
        FrontLeft   = hardwareMap.dcMotor.get("frontLeft");
        FrontRight  = hardwareMap.dcMotor.get("frontRight");
        RearLeft = hardwareMap.dcMotor.get("rearLeft");
        RearRight = hardwareMap.dcMotor.get("rearRight");
    }

    public void loop(){
        FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

        FrontRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);
        RearRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
        if(gamepad1.right_trigger > 0){
            Wobble.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger > 0){
            Wobble.setPower(-gamepad1.left_trigger);
        }
        else {
            Wobble.setPower(0);
        }

        if(gamepad1.right_bumper){
            WobbleServo.setPosition(0.5);
        }
        else if (gamepad1.left_bumper){
            WobbleServo.setPosition(0);
        }
    }
}
