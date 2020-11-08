package org.firstinspires.ftc.teamcode.Season20and21.code.Ring;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "RingleaderV1", group = "Test")
public class RingleaderV1 extends OpMode {

    RingleaderHWMap robot = new RingleaderHWMap();
    double power = 0.5;
    boolean powerIncrement = true;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

        robot.FrontRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
        robot.RearRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);

        if(gamepad1.right_bumper){
            robot.Collector.setPower(0);
        }
        else if(gamepad1.b){
            robot.Collector.setPower(1);
        }
        else if(gamepad1.x){
            robot.Collector.setPower(-1);
        }

        if(gamepad1.left_bumper){
            robot.Launcher.setPower(0);
        }
        else if(gamepad1.y){
            robot.Launcher.setPower(Math.abs(power));
        }
        else if(gamepad1.a){
            robot.Launcher.setPower(-Math.abs(power));
        }

        if(gamepad1.dpad_down && powerIncrement){   // 0.9 & 1.0 are too high, 0.7 & 0.8 go into the high goal,
            power -= 0.1;   // 0.6 & 0.7 (rebound) can knock down powershots, 0.5 & 0.6 go into middle goal,
            if(power < 0.1){   // and 0.4 & 0.5 go into the low goal, all from right behind white line
                power = 0.1;   // Also there's a 15-30 degree angle to the right that the rings will travel
            }
            powerIncrement = false;
        }
        else if (gamepad1.dpad_up && powerIncrement){
            power += 0.1;
            if (power > 1){
                power = 1;
            }
            powerIncrement = false;
        }
        if(gamepad1.dpad_right){
            powerIncrement = true;
        }
        telemetry.addLine("Power is at: "+power);
        telemetry.update();
    }
}