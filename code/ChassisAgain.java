package org.firstinspires.ftc.Season20and21.code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ChassisAgain", group = "Test")
public class ChassisAgain extends OpMode {
    HWMapAgain robot = new HWMapAgain();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.Collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

        robot.FrontRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);
        robot.RearRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);

        if(gamepad1.b){
            robot.Collector.setPower(1);
        }
        else if (gamepad1.x){
            robot.Collector.setPower(-1);
        }
        else {
            robot.Collector.setPower(0);
        }
    }
}
