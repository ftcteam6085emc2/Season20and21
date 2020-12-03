package org.firstinspires.ftc.Season20and21.code.Duck;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Duck2", group="Test")
public class Duck2 extends OpMode {

    HWMapDuck robot = new HWMapDuck();
    boolean sluggish = true;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        if(gamepad1.start){
            sluggish = !sluggish;
        }

        if(sluggish) {
            robot.Left.setPower(gamepad1.left_stick_y/2);
            robot.Right.setPower(-gamepad1.right_stick_y/2);
        }
        else{
            robot.Left.setPower(gamepad1.left_stick_y);
            robot.Right.setPower(-gamepad1.right_stick_y);
        }

        if(gamepad1.a && sluggish){
            DriveStraightDistance(1000, 0.4);
        }
        else if (gamepad1.a){
            DriveStraightDistance(1000, 0.8);
        }

        if(gamepad1.b){
            Spinjitzu(10000, 1);
        }

        if(gamepad1.x){
            try {
                DeceasedAt63();
            } catch (InterruptedException e) {
                telemetry.addLine("Wait broke");
            }
        }

        if(gamepad1.y){
            Spinjitzu(-100, -0.3);
        }
    }

    private void DriveStraightDistance(int distance, double power){
        telemetry.addData("Driving", "Yes");
        robot.Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Right.setTargetPosition(distance);
        robot.Left.setTargetPosition(-distance);

        robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.Right.setPower(power);
        robot.Left.setPower(-power);
        while((robot.Right.isBusy() && robot.Left.isBusy() )){}

        robot.Right.setPower(0);
        robot.Left.setPower(0);
        robot.Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void Spinjitzu(int distance, double power){
        telemetry.addData("Driving", "Yes");
        robot.Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Right.setTargetPosition(distance);
        robot.Left.setTargetPosition(distance);

        robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.Right.setPower(power);
        robot.Left.setPower(power);
        while((robot.Right.isBusy() && robot.Left.isBusy() )){}

        robot.Right.setPower(0);
        robot.Left.setPower(0);
        robot.Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void DeceasedAt63() throws InterruptedException {
        for(int i = 0; i < 500; i++) {
            robot.Left.setPower(1);
            robot.Right.setPower(-1);
            wait(10);
            robot.Left.setPower(-1);
            robot.Right.setPower(1);
            wait(10);
        }
    }
}