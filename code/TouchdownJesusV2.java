package org.firstinspires.ftc.teamcode.Season20and21.code;

import android.media.MediaPlayer;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.code.code.HWMapTouchdown;

@TeleOp(name="TouchdownJesusV2", group="Test")
public class TouchdownJesusV2 extends OpMode {

    private int currentPos = 0;
    private double armPower = 0.3;
    private boolean SpinCheck = false;
    private double servoPower = -0.2;
    private boolean servoSlow = false;
    private boolean touchStop = false;
    private boolean released = true;
    private RevBlinkinLedDriver.BlinkinPattern LEDColor = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    HWMapTouchdown robot = new HWMapTouchdown();
    private MediaPlayer mediaPlayer = new MediaPlayer();


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.SpinLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.SpinRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.ArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        Telemetry();
        double left1;
        double right1;
        double leftx1;
        double rightx1;
        if(!servoSlow) {
            left1 = -gamepad1.left_stick_y; //these are reversed for kevin I guess
            right1 = gamepad1.right_stick_y;
            leftx1 = -gamepad1.left_stick_x;
            rightx1 = -gamepad1.right_stick_x;
        }
        else{
            left1 = -gamepad1.left_stick_y-0.1; //these are reversed for kevin I guess
            right1 = gamepad1.right_stick_y-0.1;
            leftx1 = -gamepad1.left_stick_x-0.1;
            rightx1 = -gamepad1.right_stick_x-0.1;
        }

        if(gamepad2.dpad_right){
            robot.Arm1.setPosition(1);
            robot.Arm2.setPosition(1);
        }
        else if(gamepad2.dpad_left){
            robot.Arm1.setPosition(0);
            robot.Arm2.setPosition(0);
        }

        if(gamepad2.start || gamepad1.start){
            robot.FoundationServoLeft.setPosition(0.5);
            robot.FoundationServoRight.setPosition(-0.5);
            servoSlow = false;
        }
        if(gamepad2.back || gamepad1.back){
            robot.FoundationServoLeft.setPosition(0.1);
            robot.FoundationServoRight.setPosition(0.5);
            servoSlow = true;
        }
        if(gamepad2.left_stick_button || gamepad1.left_stick_button){
            robot.FoundationServoLeft.setPosition(1);
            robot.FoundationServoRight.setPosition(1);
        }
        if(gamepad2.dpad_down && released){
            SpinCheck = !SpinCheck;
            released = false;
        }
        else if (!gamepad2.dpad_down){
            released = true;
        }
        if(gamepad2.x) {
            robot.SpinRight.setPower(-1);
            robot.SpinLeft.setPower(1);
        }
        else if(gamepad2.b) {
            robot.SpinRight.setPower(1);
            robot.SpinLeft.setPower(-1);
        }
        else if (!SpinCheck) {
            robot.SpinRight.setPower(0);
            robot.SpinLeft.setPower(0);
        }
        if(robot.touchSensor.isPressed() && touchStop == false){
            robot.SpinRight.setPower(0);
            robot.SpinLeft.setPower(0);
            touchStop = true;
            LEDColor = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
            robot.RaveShadowLegends.setPattern(LEDColor);
        }
        if(!robot.touchSensor.isPressed()){
            touchStop = false;
            if(LEDColor == RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN) {
                LEDColor = RevBlinkinLedDriver.BlinkinPattern.RED;
            }
            robot.RaveShadowLegends.setPattern(LEDColor);
        }
        if(gamepad2.left_bumper){
            robot.GrabRight.setPosition(0.1);
            robot.GrabLeft.setPosition(0.2);
        }
        if(gamepad2.right_bumper){
            robot.GrabRight.setPosition(-0.1);
            robot.GrabLeft.setPosition(-0.1);
        }

        if(gamepad2.y){
            robot.ArmRight.setPower(armPower);
            robot.ArmLeft.setPower(-armPower);
        }
        else if(gamepad2.a){
            robot.ArmRight.setPower(-armPower);
            robot.ArmLeft.setPower(armPower);
        }
        else {
            robot.ArmRight.setPower(0);
            robot.ArmLeft.setPower(0);
        }
        if(gamepad2.left_trigger > 0){
            robot.RaveShadowLegends.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        }
        else {
            robot.RaveShadowLegends.setPattern(LEDColor);
        }


        if(gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0){
            robot.FrontLeft.setPower(left1+leftx1);
            robot.RearLeft.setPower(left1-leftx1);

            robot.FrontRight.setPower(right1+rightx1);
            robot.RearRight.setPower(right1-rightx1);
        }
        else {
            robot.FrontLeft.setPower(left1);
            robot.RearLeft.setPower(left1);

            robot.FrontRight.setPower(right1);
            robot.RearRight.setPower(right1);
        }
            /*if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                robot.FrontLeft.setPower((left1 + leftx1)*Multiplier);
                robot.RearLeft.setPower((left1 - leftx1)*Multiplier);
                robot.FrontRight.setPower((right1 - rightx1)*Multiplier);
                robot.RearRight.setPower((right1 + rightx1)*Multiplier);
            } else {
                robot.FrontLeft.setPower(left1*Multiplier);
                robot.RearLeft.setPower(left1*Multiplier);
                robot.FrontRight.setPower(-right1*Multiplier);
                robot.RearRight.setPower(-right1*Multiplier);
            }
            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
                robot.FrontLeft.setPower((left2 + leftx2)*Multiplier);
                robot.RearLeft.setPower((left2 - leftx2)*Multiplier);
                robot.FrontRight.setPower((right2 - rightx2)*Multiplier);
                robot.RearRight.setPower((right2 + rightx2)*Multiplier);
            } else {
                robot.FrontLeft.setPower(left2*Multiplier);
                robot.RearLeft.setPower(left2*Multiplier);
                robot.FrontRight.setPower(-right2*Multiplier);
                robot.RearRight.setPower(-right2*Multiplier);
            }*/
    }
    private void DriveStraight(double power){
        /*if(strafeCancel){
            robot.FrontRight.setPower(power - 0.2);
            robot.FrontLeft.setPower(-power);
            robot.RearRight.setPower(power);
            robot.RearLeft.setPower(-power - 0.2);
        }
        else {*/
        robot.FrontRight.setPower(-power);
        robot.FrontLeft.setPower(power);
        robot.RearRight.setPower(-power);
        robot.RearLeft.setPower(power);
        //}
    }
    private void StopDriving (){DriveStraight(0);}

    private void DriveStraightDistance(int distance, double power){
        telemetry.addData("Driving", "Yes");
        robot.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FrontRight.setTargetPosition(-distance);
        robot.FrontLeft.setTargetPosition(distance);
        robot.RearRight.setTargetPosition(-distance);
        robot.RearLeft.setTargetPosition(distance);

        robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveStraight(power);
        while((robot.FrontRight.isBusy() || robot.RearLeft.isBusy() || robot.RearRight.isBusy() || robot.FrontLeft.isBusy())){}

        StopDriving();
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void Strafe (int distance, double power){
        telemetry.addData("Driving", "Yes");
        robot.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FrontRight.setTargetPosition(distance);
        robot.FrontLeft.setTargetPosition(distance);
        robot.RearRight.setTargetPosition(-distance);
        robot.RearLeft.setTargetPosition(-distance);

        robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FrontRight.setPower(power);
        robot.FrontLeft.setPower(power);
        robot.RearRight.setPower(-power);
        robot.RearLeft.setPower(-power);

        while((robot.FrontRight.isBusy() || robot.RearLeft.isBusy() || robot.RearRight.isBusy() || robot.FrontLeft.isBusy())){}

        StopDriving();
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void Flip (){
        robot.ArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(currentPos == 0) {
            for (int i = 225; i <= 1100; i += 225) {
                robot.ArmLeft.setTargetPosition(i);
                robot.ArmRight.setTargetPosition(-i);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setPower(0.25);
                robot.ArmRight.setPower(-0.25);
                while (robot.ArmLeft.isBusy() || robot.ArmRight.isBusy()) {
                    telemetry.addData("encoder-ArmLeft", robot.ArmLeft.getCurrentPosition() + "  busy=" + robot.ArmLeft.isBusy());
                    telemetry.addData("encoder-ArmRight", robot.ArmRight.getCurrentPosition() + "  busy=" + robot.ArmRight.isBusy());
                    telemetry.update();
                }
                robot.ArmLeft.setPower(0);
                robot.ArmRight.setPower(0);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        else if(currentPos == 1){
            for (int i = 225; i <= 550; i += 225) {
                robot.ArmLeft.setTargetPosition(i);
                robot.ArmRight.setTargetPosition(-i);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setPower(0.25);
                robot.ArmRight.setPower(-0.25);
                while (robot.ArmLeft.isBusy() || robot.ArmRight.isBusy()) {
                    telemetry.addData("encoder-ArmLeft", robot.ArmLeft.getCurrentPosition() + "  busy=" + robot.ArmLeft.isBusy());
                    telemetry.addData("encoder-ArmRight", robot.ArmRight.getCurrentPosition() + "  busy=" + robot.ArmRight.isBusy());
                    telemetry.update();
                }
                robot.ArmLeft.setPower(0);
                robot.ArmRight.setPower(0);
            }
        }
        robot.GrabRight.setPosition(0.6);
        robot.GrabLeft.setPosition(0.6);
        robot.SpinRight.setPower(0);
        robot.SpinLeft.setPower(0);
        currentPos = 2;
        robot.ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void RevFlip() {
        robot.ArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(currentPos == 2) {
            for (int i = -225; i >= -1100; i -= 225) {
                robot.ArmLeft.setTargetPosition(i);
                robot.ArmRight.setTargetPosition(-i);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setPower(-0.25);
                robot.ArmRight.setPower(0.25);
                while (robot.ArmLeft.isBusy() || robot.ArmRight.isBusy()) {
                    telemetry.addData("encoder-ArmLeft", robot.ArmLeft.getCurrentPosition() + "  busy=" + robot.ArmLeft.isBusy());
                    telemetry.addData("encoder-ArmRight", robot.ArmRight.getCurrentPosition() + "  busy=" + robot.ArmRight.isBusy());
                    telemetry.update();
                }
                robot.ArmLeft.setPower(0);
                robot.ArmRight.setPower(0);
                if (i == -280) {
                    robot.GrabRight.setPosition(0.2);
                    robot.GrabLeft.setPosition(0.2);
                }
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        else if(currentPos == 1){
            for (int i = -225; i >= -550; i -= 225) {
                robot.ArmLeft.setTargetPosition(i);
                robot.ArmRight.setTargetPosition(-i);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setPower(-0.25);
                robot.ArmRight.setPower(0.25);
                while (robot.ArmLeft.isBusy() || robot.ArmRight.isBusy()) {
                    telemetry.addData("encoder-ArmLeft", robot.ArmLeft.getCurrentPosition() + "  busy=" + robot.ArmLeft.isBusy());
                    telemetry.addData("encoder-ArmRight", robot.ArmRight.getCurrentPosition() + "  busy=" + robot.ArmRight.isBusy());
                    telemetry.update();
                }
                robot.ArmLeft.setPower(0);
                robot.ArmRight.setPower(0);
            }
        }
        robot.GrabRight.setPosition(0.6);
        robot.GrabLeft.setPosition(0.6);
        currentPos = 0;
        robot.ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void Center (){
        robot.ArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(currentPos == 0) {
            for (int i = 225; i <= 550; i += 225) {
                robot.ArmLeft.setTargetPosition(i);
                robot.ArmRight.setTargetPosition(-i);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setPower(0.25);
                robot.ArmRight.setPower(-0.25);
                while (robot.ArmLeft.isBusy() || robot.ArmRight.isBusy()) {
                    telemetry.addData("encoder-ArmLeft", robot.ArmLeft.getCurrentPosition() + "  busy=" + robot.ArmLeft.isBusy());
                    telemetry.addData("encoder-ArmRight", robot.ArmRight.getCurrentPosition() + "  busy=" + robot.ArmRight.isBusy());
                    telemetry.update();
                }
                robot.ArmLeft.setPower(0);
                robot.ArmRight.setPower(0);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        else if(currentPos == 2){
            for (int i = -225; i >= -550; i -= 225) {
                robot.ArmLeft.setTargetPosition(i);
                robot.ArmRight.setTargetPosition(-i);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ArmLeft.setPower(-0.25);
                robot.ArmRight.setPower(0.25);
                while (robot.ArmLeft.isBusy() || robot.ArmRight.isBusy()) {
                    telemetry.addData("encoder-ArmLeft", robot.ArmLeft.getCurrentPosition() + "  busy=" + robot.ArmLeft.isBusy());
                    telemetry.addData("encoder-ArmRight", robot.ArmRight.getCurrentPosition() + "  busy=" + robot.ArmRight.isBusy());
                    telemetry.update();
                }
                robot.ArmLeft.setPower(0);
                robot.ArmRight.setPower(0);
            }
        }
        robot.GrabRight.setPosition(0.6);
        robot.GrabLeft.setPosition(0.6);
        robot.SpinRight.setPower(0);
        robot.SpinLeft.setPower(0);
        currentPos = 1;
        robot.ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void Telemetry () {
        telemetry.addData("FR_Power", "%.2f", robot.FrontRight.getPower());
        telemetry.addData("RR_Power", "%.2f", robot.RearRight.getPower());
        telemetry.addData("FL_Power", "%.2f", robot.FrontLeft.getPower());
        telemetry.addData("RL_Power", "%.2f", robot.RearLeft.getPower());
        telemetry.addData("Front Right Encoder Position", robot.FrontRight.getCurrentPosition());
        telemetry.addData("Rear Right Encoder Position", robot.RearRight.getCurrentPosition());
        telemetry.addData("Front Left Encoder Position", robot.FrontLeft.getCurrentPosition());
        telemetry.addData("Rear Left Encoder Position", robot.RearLeft.getCurrentPosition());
        telemetry.addData("encoder-ArmLeft", robot.ArmLeft.getCurrentPosition() + "  busy=" + robot.ArmLeft.isBusy());
        telemetry.addData("encoder-ArmRight", robot.ArmRight.getCurrentPosition() + "  busy=" + robot.ArmRight.isBusy());
        telemetry.update();
    }
}