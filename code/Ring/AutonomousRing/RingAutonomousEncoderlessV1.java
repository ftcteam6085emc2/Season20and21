package org.firstinspires.ftc.Season20and21.code.Ring.AutonomousRing;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.Season20and21.code.Ring.RingleaderHWMapSensorsColor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "RingAutonomousEncoderlessV1", group = "Concept")
public class RingAutonomousEncoderlessV1 extends LinearOpMode {
    int tZone = 0;
    int averageCount1 = 0;
    int averageCount2 = 0;
    int averageCount3 = 0;
    boolean whiteDetected = false;

    RingleaderHWMapSensorsColor robot = new RingleaderHWMapSensorsColor();

    @Override
    public void runOpMode() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        robot.init(hardwareMap);

        robot.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FrontRight.setTargetPosition(0);
        robot.FrontLeft.setTargetPosition(0);
        robot.RearRight.setTargetPosition(0);
        robot.RearLeft.setTargetPosition(0);

        robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

        Strafe(-100, 0.6);
        DriveStraightDistance(1400,0.8);
        DriveStraightDistanceSquared(400, 0.4);
        Strafe(1200, 0.6);
        DriveStraightDistance(2000, 0.8);
        DriveStraightDistance(500, 0.4); //was color
        if(whiteDetected){
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.GREEN);
                }
            });
            telemetry.addLine("Mayo Detected");
            telemetry.update();
        }

        if(averageCount1 > averageCount2 && averageCount1 > averageCount3){
            tZone = 1;
        }
        else if(averageCount2 > averageCount3 && averageCount2 > averageCount1){
            tZone = 2;
        }
        else{
            tZone = 3;
        }
        telemetry.update();
        switch (tZone) {
            case 1:
                telemetry.addLine("There are 0 rings in the stack");
                telemetry.update();
                robot.Collector.setPower(0.5);
                sleep(5000);
                robot.Collector.setPower(0);
                DriveStraightDistance(-1000, 0.8);
                Strafe(-1300, 0.6);
                Turn(2900, 0.8);
                robot.Launcher.setPower(0.7);
                sleep(500);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                sleep(3000);
                Strafe(500, 0.6);
                sleep(4000);
                robot.Launcher.setPower(0);
                Turn(2900, 0.8);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 180 && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 120cm - 180cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistance(-750, 0.8);
                break;
            case 2:
                telemetry.addLine("There is 1 ring in the stack");
                telemetry.update();
                DriveStraightDistance(1000, 0.8);
                Strafe(-1300, -0.6);
                robot.Collector.setPower(0.5);
                sleep(5000);
                robot.Collector.setPower(0);
                DriveStraightDistance(-1500, 0.8);
                Turn(2800, 0.8);
                robot.Launcher.setPower(0.7);
                sleep(2000);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(5000);
                robot.Launcher.setPower(0);
                Turn(2800, 0.8);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 120 && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 60cm - 120cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistance(-750, 0.8);
                break;
            case 3:
                telemetry.addLine("There are 4 rings in the stack");
                telemetry.update();
                DriveStraightDistance(2500, 0.8);
                robot.Collector.setPower(0.5);
                sleep(5000);
                robot.Collector.setPower(0);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 60  && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 0cm - 60cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistance(-3000, 0.8);
                Strafe(-1000, 0.6);
                Turn(2800, 0.8);
                robot.Launcher.setPower(0.7);
                sleep(1000);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                robot.Collector.setPower(1);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(5000);
                robot.Launcher.setPower(0);
                Turn(2800, 0.8);
                DriveStraightDistance(-750, 0.8);
        }
    }

    private void DriveStraight(double power) {
        robot.FrontRight.setPower(power);
        robot.FrontLeft.setPower(power);
        robot.RearRight.setPower(power);
        robot.RearLeft.setPower(power);
    }

    private void StopDriving() {
        DriveStraight(0);
    }

    private void DriveStraightDistance(int distance, double power) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
        }

        StopDriving();
    }

    private void DriveStraightDistanceSquared(int distance, double power) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            if(robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 14){
                averageCount3++;
            }
            else if (robot.sensorRangeBottom.getDistance(DistanceUnit.CM) < 14){
                averageCount2++;
            }
            else {
                averageCount1++;
            }
        }

        StopDriving();
    }

    private void DriveStraightDistanceColor(int distance, double power) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            if(robot.sensorColor.alpha() >= 2500){
                whiteDetected = true;
                break;
            }
        }

        StopDriving();
    }

    private void Turn(int distance, double power) {
        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
        }

        StopDriving();
    }

    private void Strafe(int distance, double power) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() - distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
        }

        StopDriving();
    }
}
