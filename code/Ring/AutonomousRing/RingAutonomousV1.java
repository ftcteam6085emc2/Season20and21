package org.firstinspires.ftc.Season20and21.code.Ring.AutonomousRing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.Season20and21.code.Ring.HWMap.RingleaderHWMapSensorsColor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.Season20and21.code.HeadingHolder;

@Autonomous(name = "RingAutonomousV1", group = "Autonomous", preselectTeleOp = "RingleaderV1")
public class RingAutonomousV1 extends LinearOpMode {
    int tZone = 0;
    double targetHeading = 0;
    double currentHeading = 0;
    double offset = 0;
    double p = 0;
    double k_p = 0.0028;
    double turn_k_p = 0.02;
    double steer = 0;
    double turn_error = 0;
    double output = 0;
    double drive_speed = 0.9;
    double turn_speed = 0.7;
    double drive_speed3 = 1;
    double turn_speed3 = 0.8;
    int averageCount1 = 0;
    int averageCount2 = 0;
    int averageCount3 = 0;
    boolean whiteDetected = false;
    boolean ringLoaded = false;

    Orientation angles;

    RingleaderHWMapSensorsColor robot = new RingleaderHWMapSensorsColor();

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.init(hardwareMap);
        robot.imu.initialize(parameters);

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
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("IMU calibration status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        /*relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });*/

        checkOrientation();
        offset = currentHeading;

        Strafe(-150);
        DriveStraightDistanceSquared(400);
        Strafe(1200);
        DriveStraightDistance(2000, false);
        DriveStraightDistanceColor(800);
        /*Strafe(500, 0.8);
        checkOrientation();
        offset -= currentHeading;
        Strafe(-50, 0.8);*/
        /*if(whiteDetected){
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.GREEN);
                }
            });
            telemetry.addLine("Mayo Detected");
            telemetry.update();
        }*/

        if (averageCount1 > averageCount2 && averageCount1 > averageCount3) {
            tZone = 1;
        } else if (averageCount2 > averageCount3 && averageCount2 > averageCount1) {
            tZone = 2;
        } else {
            tZone = 3;
        }
        telemetry.update();
        switch (tZone) {
            case 1:
                telemetry.addLine("There are 0 rings in the stack");
                telemetry.update();
                DriveStraightDistance(-500, false);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistance(-300, true);
                sleep(200);
                robot.Collector.setPower(0);

                robot.WobbleRotate.setPosition(0.2);
                robot.WobbleServo.setPosition(0.2);
                DriveStraightDistance(-3175, false);
                //offset -= currentHeading;
                //DriveStraightDistance(550, 1, false);
                Strafe(-1600);
                orient(0.1);
                robot.WobbleServo.setPosition(0.7);
                sleep(500);
                robot.Wobble.setPower(1);
                sleep(300);
                robot.Wobble.setPower(0);
                Strafe(1300);
                DriveStraightDistance(2300, false);
                DriveStraightDistanceColor(400);
                /*Strafe(500, 0.8);
                checkOrientation();
                offset -= currentHeading;*/
                targetHeading = -90;
                Turn(1475);
                robot.WobbleServo.setPosition(0.2);
                Strafe(-200);
                Strafe(200);
                robot.WobbleRotate.setPosition(0.7);
                robot.WobbleServo.setPosition(0.7);
                Strafe(300);
                robot.Wobble.setPower(-1);
                sleep(300);
                robot.Wobble.setPower(0);
                DriveStraightDistance(-400, false);

                targetHeading = -180;
                Turn(1475);
                //Strafe(-1000, 0.8);
                //offset -= currentHeading;
                Strafe(600);
                DriveStraightDistance(-600, false);
                DriveStraightDistance(200, false);
                ShootPowershots(0);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 180 && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 120cm - 180cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistanceColor(-1000);
                break;
            case 2:
                telemetry.addLine("There is 1 ring in the stack");
                telemetry.update();
                DriveStraightDistance(800, false);
                Strafe(-1600);
                robot.Collector.setPower(0.5);
                sleep(2000);
                DriveStraightDistance(-300, true);
                sleep(1000);
                robot.Collector.setPower(0);

                /*Strafe(-100, 0.8);
                robot.WobbleRotate.setPosition(0.2);
                robot.WobbleServo.setPosition(0.2);
                DriveStraightDistance(-3980, 1, false);
                Strafe(-100, 0.2);
                robot.WobbleServo.setPosition(0.7);
                sleep(300);
                robot.Wobble.setPower(1);
                sleep(100);
                robot.Wobble.setPower(0);
                Strafe(100, 0.8);
                DriveStraightDistance(2000, 1, false);
                DriveStraightDistanceColor(400, 0.6);
                Turn(1450, 0.8, true);
                robot.WobbleServo.setPosition(0.2);
                Strafe(-100, 1);
                Strafe(100, 1);
                robot.WobbleRotate.setPosition(0.7);
                robot.WobbleServo.setPosition(0.7);
                robot.Wobble.setPower(-1);
                sleep(100);
                robot.Wobble.setPower(0);
                Strafe(1450, 0.8);*/

                DriveStraightDistanceColor(-950);
                DriveStraightDistance(-500, false);
                targetHeading = -90;
                Turn(1475);
                targetHeading = -180;
                Turn(1475);
                //Strafe(-2000, 0.8);
                //offset -= currentHeading;
                //Strafe(-1600);
                DriveStraightDistance(600, false);
                DriveStraightDistance(-200, false);
                ShootPowershots(0);
                DriveStraightDistanceColor(-1000);
                break;
            case 3:
                telemetry.addLine("There are 4 rings in the stack");
                telemetry.update();
                drive_speed = drive_speed3;
                turn_speed = turn_speed3;
                DriveStraightDistance(2500, false);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistance(-300, true);
                sleep(200);
                robot.Collector.setPower(0);

                Strafe(-200);
                robot.WobbleRotate.setPosition(0.2);
                robot.WobbleServo.setPosition(0.2);
                DriveStraightDistance(-6250, false);
                //offset -= currentHeading;
                //DriveStraightDistance(550, 1, false);
                Strafe(-1400);
                orient(0.1);
                robot.WobbleServo.setPosition(0.7);
                sleep(500);
                robot.Wobble.setPower(1);
                sleep(300);
                robot.Wobble.setPower(0);
                Strafe(1300);
                DriveStraightDistance(5750, false);
                //DriveStraightDistance(2700, false);
                //DriveStraightDistanceColor(400);
                //DriveStraightDistance(2500, false);
                /*Strafe(500, 0.8);
                checkOrientation();
                offset -= currentHeading;*/
                targetHeading = -90;
                Turn(1475);
                robot.WobbleServo.setPosition(0.2);
                Strafe(-200);
                Strafe(200);
                robot.WobbleRotate.setPosition(0.7);
                robot.WobbleServo.setPosition(0.7);
                Strafe(3200);
                robot.Wobble.setPower(-1);
                sleep(300);
                robot.Wobble.setPower(0);

                //DriveStraightDistanceColor(-2700, 1);
                //DriveStraightDistance(-500, 1, false);
                //Strafe(-400, 0.8);
                targetHeading = -180;
                Turn(1475);
                DriveStraightDistance(400, false);
                //Strafe(-600, 0.8);
                //offset -= currentHeading;
                /////Strafe(990);
                /////DriveStraightDistance(-400, false);
                //DriveStraightDistance(200, 0.8, false);
                ShootPowershots(1);
                /*Strafe(-700, 0.8);
                DriveStraightDistance(300, 1);
                robot.Collector.setPower(1);
                robot.Elevator.setPower(0.5);
                DriveStraightDistance(750, 0.1);
                robot.Collector.setPower(0);
                robot.Elevator.setPower(0);
                DriveStraightDistance(-700, 1);
                Strafe(-1000, 0.8);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                robot.Collector.setPower(1);
                sleep(2000);*/
                DriveStraightDistanceColor(-1000);
        }
        checkOrientation();
        HeadingHolder.setHeading(currentHeading);
    }

    private void DriveStraight(double rightPower, double leftPower) {
        robot.FrontRight.setPower(rightPower);
        robot.FrontLeft.setPower(leftPower);
        robot.RearRight.setPower(rightPower);
        robot.RearLeft.setPower(leftPower);
    }

    private void StopDriving() {
        DriveStraight(0, 0);
    }

    private void DriveStraightDistance(int distance, boolean fast) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(drive_speed, drive_speed);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            checkOrientation();
            telemetry.addData("IMU: ", currentHeading);
            telemetry.update();
            p = Math.abs(k_p * (robot.FrontRight.getTargetPosition() - robot.FrontRight.getCurrentPosition()));
            turn_error = turn_k_p*(targetHeading - currentHeading);
            while(turn_error > 180){
                turn_error -= 360;
            }
            while(turn_error < -180){
                turn_error += 360;
            }
            output = Range.clip(p - turn_error, -drive_speed, drive_speed);
            steer = Range.clip(p + turn_error, -drive_speed, drive_speed);
            DriveStraight(output, steer);
            /*if (distance == Math.abs(distance)) {
                if (currentHeading > targetHeading + 1) {
                    robot.FrontRight.setPower(power * 0.9);
                    robot.FrontLeft.setPower(power * 1.1);
                    robot.RearRight.setPower(power * 0.9);
                    robot.RearLeft.setPower(power * 1.1);
                } else if (currentHeading < targetHeading - 1) {
                    robot.FrontRight.setPower(power * 1.1);
                    robot.FrontLeft.setPower(power * 0.9);
                    robot.RearRight.setPower(power * 1.1);
                    robot.RearLeft.setPower(power * 0.9);
                } else {
                    robot.FrontRight.setPower(power);
                    robot.FrontLeft.setPower(power);
                    robot.RearRight.setPower(power);
                    robot.RearLeft.setPower(power);
                }
            } else {
                if (currentHeading > targetHeading + 1) {
                    robot.FrontRight.setPower(power * 1.1);
                    robot.FrontLeft.setPower(power * 0.9);
                    robot.RearRight.setPower(power * 1.1);
                    robot.RearLeft.setPower(power * 0.9);
                } else if (currentHeading < targetHeading - 1) {
                    robot.FrontRight.setPower(power * 0.9);
                    robot.FrontLeft.setPower(power * 1.1);
                    robot.RearRight.setPower(power * 0.9);
                    robot.RearLeft.setPower(power * 1.1);
                } else {
                    robot.FrontRight.setPower(power);
                    robot.FrontLeft.setPower(power);
                    robot.RearRight.setPower(power);
                    robot.RearLeft.setPower(power);
                }
            }*/
        }
        StopDriving();
        if(!fast) {
            sleep(10);
        }
    }

    private void DriveStraightDistanceSquared(int distance) {
        DriveStraightDistance(1400, false);
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(drive_speed, drive_speed);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            if (robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 14) {
                averageCount3++;
            } else if (robot.sensorRangeBottom.getDistance(DistanceUnit.CM) < 14) {
                averageCount2++;
            } else {
                averageCount1++;
            }
        }
        StopDriving();
        sleep(10);
    }

    private void Strafe(int distance) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() - distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(turn_speed, turn_speed);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            checkOrientation();
            p = Math.abs(k_p * (robot.FrontRight.getTargetPosition() - robot.FrontRight.getCurrentPosition()));
            turn_error = turn_k_p*(targetHeading - currentHeading);
            while(turn_error > 180){
                turn_error -= 360;
            }
            while(turn_error < -180){
                turn_error += 360;
            }
            output = Range.clip(p - turn_error, -turn_speed, turn_speed);
            steer = Range.clip(p + turn_error, -turn_speed, turn_speed);
            DriveStraight(output, steer);
            /*checkOrientation();
            if (distance == Math.abs(distance)) {
                if (currentHeading > targetHeading + 1) {
                    robot.FrontRight.setPower(power * 0.9);
                    robot.FrontLeft.setPower(power * 0.9);
                    robot.RearRight.setPower(power * 1.1);
                    robot.RearLeft.setPower(power * 1.1);
                } else if (currentHeading < targetHeading - 1) {
                    robot.FrontRight.setPower(power * 1.1);
                    robot.FrontLeft.setPower(power * 1.1);
                    robot.RearRight.setPower(power * 0.9);
                    robot.RearLeft.setPower(power * 0.9);
                } else {
                    robot.FrontRight.setPower(power);
                    robot.FrontLeft.setPower(power);
                    robot.RearRight.setPower(power);
                    robot.RearLeft.setPower(power);
                }
            } else {
                if (currentHeading > targetHeading + 1) {
                    robot.FrontRight.setPower(power * 1.1);
                    robot.FrontLeft.setPower(power * 1.1);
                    robot.RearRight.setPower(power * 0.9);
                    robot.RearLeft.setPower(power * 0.9);
                } else if (currentHeading < targetHeading - 1) {
                    robot.FrontRight.setPower(power * 0.9);
                    robot.FrontLeft.setPower(power * 0.9);
                    robot.RearRight.setPower(power * 1.1);
                    robot.RearLeft.setPower(power * 1.1);
                } else {
                    robot.FrontRight.setPower(power);
                    robot.FrontLeft.setPower(power);
                    robot.RearRight.setPower(power);
                    robot.RearLeft.setPower(power);
                }
            }*/
        }
        StopDriving();
        sleep(10);
    }

    private void DriveStraightDistanceColor(int distance) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(drive_speed, drive_speed);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            if (robot.sensorColor.alpha() >= 800) {
                whiteDetected = true;
                telemetry.addLine("White Line Detected!");
                telemetry.update();
                break;
            }
        }
        StopDriving();
        sleep(10);
    }

    private void Turn(int distance) {
        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(turn_speed, turn_speed);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
            //p = robot.FrontRight.getTargetPosition() - robot.FrontRight.getCurrentPosition();
            //output = Range.clip(k_p*p, -1, 1);
            checkOrientation();
            telemetry.addData("IMU: ", currentHeading);
            telemetry.update();
            turn_error = targetHeading - currentHeading;
            while(turn_error > 180){
                turn_error -= 360;
            }
            while(turn_error < -180){
                turn_error += 360;
            }
            steer = Range.clip(turn_k_p*turn_error, -turn_speed, turn_speed);
            DriveStraight(-steer, steer);
            /*checkOrientation();
            if (distance == Math.abs(distance)) {
                if (currentHeading > targetHeading + 1) {
                    robot.FrontRight.setPower(power * 0.9);
                    robot.FrontLeft.setPower(power * 1.1);
                    robot.RearRight.setPower(power * 0.9);
                    robot.RearLeft.setPower(power * 1.1);
                } else if (currentHeading < targetHeading - 1) {
                    robot.FrontRight.setPower(power * 1.1);
                    robot.FrontLeft.setPower(power * 0.9);
                    robot.RearRight.setPower(power * 1.1);
                    robot.RearLeft.setPower(power * 0.9);
                } else {
                    robot.FrontRight.setPower(power);
                    robot.FrontLeft.setPower(power);
                    robot.RearRight.setPower(power);
                    robot.RearLeft.setPower(power);
                }
            } else {
                if (currentHeading > targetHeading + 1) {
                    robot.FrontRight.setPower(power * 1.1);
                    robot.FrontLeft.setPower(power * 0.9);
                    robot.RearRight.setPower(power * 1.1);
                    robot.RearLeft.setPower(power * 0.9);
                } else if (currentHeading < targetHeading - 1) {
                    robot.FrontRight.setPower(power * 0.9);
                    robot.FrontLeft.setPower(power * 1.1);
                    robot.RearRight.setPower(power * 0.9);
                    robot.RearLeft.setPower(power * 1.1);
                } else {
                    robot.FrontRight.setPower(power);
                    robot.FrontLeft.setPower(power);
                    robot.RearRight.setPower(power);
                    robot.RearLeft.setPower(power);
                }
            }*/
        }
        /*if(half) {
            checkOrientation();
            offset -= currentHeading;
            targetHeading = 0;
        }*/
        StopDriving();
        sleep(10);
    }

    private void ShootPowershots(int a){
        int i = 0;
        int j = 2;
        if(a == 1){
            robot.Launcher.setPower(0.8);
            orient(1);
            robot.ServoElevate.setPower(-1);
            robot.Elevator.setPower(0.5);
            robot.Collector.setPower(1);
            sleep(3000);
        }
        else {
            robot.Launcher.setPower(0.7);
            orient(0.5);
            while (i <= j) {
                ringLoaded = robot.ringSensorColor.red() > 750 || robot.ringSensorColor.green() > 750;
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                robot.Collector.setPower(1);
                //if(ringLoaded || i == 0){
                /*while(ringLoaded){
                    telemetry.addLine("i = "+i);
                    telemetry.update();
                    if(robot.ringSensorColor.red() > 1000 || robot.ringSensorColor.green() > 1000){ringLoaded = true;}
                    else{ringLoaded = false;}
                    idle();
                }*/
                if (i == 0) {
                    sleep(1150);
                    Strafe(400);
                } else if (i == 1) {
                    sleep(250);
                    Strafe(775);
                } else if (i > 1) {
                    sleep(1000);
                }
                i++;
                //}
                //if(System.nanoTime()/1000000000 > time + 7){
                //  i = 3;
                //}
            }
        }
        robot.ServoElevate.setPower(0);
        robot.Elevator.setPower(0);
        robot.Collector.setPower(0);
        /*if(i > 1) {
            if (tZone == 1) {
                robot.Launcher.setPower(0);
            } else {
                robot.Launcher.setPower(0.8);
            }
        }*/
    }

    private void orient(double seconds){
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        for(int k = 0; k <= 100*seconds; k++){
            checkOrientation();
            turn_error = targetHeading - currentHeading;
            while(turn_error > 180){
                turn_error -= 360;
            }
            while(turn_error < -180){
                turn_error += 360;
            }
            steer = Range.clip(turn_k_p*turn_error, -1, 1);
            DriveStraight(steer, steer);
            sleep(10);
        }
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
    }

    private void checkOrientation() {
        // read the orientation of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.imu.getPosition();
        // and save the heading
        currentHeading = angles.firstAngle - offset;
    }
}
