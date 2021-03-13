package org.firstinspires.ftc.Season20and21.code.Ring.AutonomousRing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        Strafe(-150, 0.8);
        DriveStraightDistanceSquared(400, 0.6);
        Strafe(1550, 0.8);
        DriveStraightDistance(2000, 1, false);
        DriveStraightDistanceColor(800, 0.8);
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
                DriveStraightDistance(-500, 1, false);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistance(-300, 1, true);
                sleep(200);
                robot.Collector.setPower(0);

                Strafe(-200, 0.6);
                robot.WobbleRotate.setPosition(0.2);
                robot.WobbleServo.setPosition(0.2);
                DriveStraightDistance(-4000, 1, false);
                offset -= currentHeading;
                DriveStraightDistance(550, 1, false);
                Strafe(-1700, 0.6);
                Strafe(-100, 0.2);
                robot.WobbleServo.setPosition(0.7);
                sleep(500);
                robot.Wobble.setPower(1);
                sleep(300);
                robot.Wobble.setPower(0);
                Strafe(1600, 0.8);
                DriveStraightDistance(2600, 1, false);
                DriveStraightDistanceColor(400, 0.8);
                /*Strafe(500, 0.8);
                checkOrientation();
                offset -= currentHeading;*/
                Strafe(-50, 0.8);
                Turn(1450, 0.8, true);
                Strafe(-250, 1);
                robot.WobbleServo.setPosition(0.2);
                Strafe(-200, 1);
                Strafe(200, 1);
                robot.WobbleRotate.setPosition(0.7);
                robot.WobbleServo.setPosition(0.7);
                Strafe(300, 0.8);
                robot.Wobble.setPower(-1);
                sleep(300);
                robot.Wobble.setPower(0);
                DriveStraightDistance(-400, 1, false);

                Turn(1445, 0.8, true);
                Strafe(-1000, 0.8);
                offset -= currentHeading;
                Strafe(1590, 0.6);
                DriveStraightDistance(-600, 0.8, false);
                DriveStraightDistance(200, 0.8, false);
                ShootPowershots(true);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 180 && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 120cm - 180cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistanceColor(-1000, 1);
                break;
            case 2:
                telemetry.addLine("There is 1 ring in the stack");
                telemetry.update();
                DriveStraightDistance(800, 0.8, false);
                Strafe(-1600, 0.6);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistance(-300, 1, true);
                sleep(200);
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

                DriveStraightDistanceColor(-950, 0.8);
                DriveStraightDistance(-500, 0.8, false);
                Turn(1450, 0.8, true);
                Turn(1445, 0.8, true);
                Strafe(-2000, 0.8);
                offset -= currentHeading;
                Strafe(1590, 0.6);
                DriveStraightDistance(-600, 0.8, false);
                DriveStraightDistance(200, 0.8, false);
                ShootPowershots(true);
                Strafe(-700, 0.8);
                DriveStraightDistance(300, 0.8, false);
                robot.Collector.setPower(1);
                robot.Elevator.setPower(0.5);
                DriveStraightDistance(500, 0.2, false);
                sleep(1000);
                robot.Collector.setPower(0);
                robot.Elevator.setPower(0);
                DriveStraightDistance(-700, 0.8, false);
                Strafe(-1000, 0.6);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                robot.Collector.setPower(1);
                sleep(1000);
                DriveStraightDistanceColor(-1000, 1);
                break;
            case 3:
                telemetry.addLine("There are 4 rings in the stack");
                telemetry.update();
                DriveStraightDistance(2500, 1, false);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistance(-300, 1, true);
                sleep(200);
                robot.Collector.setPower(0);

                Strafe(-200, 0.6);
                robot.WobbleRotate.setPosition(0.2);
                robot.WobbleServo.setPosition(0.2);
                DriveStraightDistance(-6500, 1, false);
                offset -= currentHeading;
                DriveStraightDistance(550, 1, false);
                Strafe(-1700, 0.6);
                Strafe(-100, 0.3);
                robot.WobbleServo.setPosition(0.7);
                sleep(500);
                robot.Wobble.setPower(1);
                sleep(300);
                robot.Wobble.setPower(0);
                Strafe(1600, 0.8);
                DriveStraightDistance(2600, 1, false);
                DriveStraightDistanceColor(400, 0.8);
                DriveStraightDistance(2500,1,false);
                /*Strafe(500, 0.8);
                checkOrientation();
                offset -= currentHeading;*/
                Strafe(-50, 0.8);
                Turn(1450, 0.8, true);
                robot.WobbleServo.setPosition(0.2);
                Strafe(-100, 1);
                Strafe(100, 1);
                robot.WobbleRotate.setPosition(0.7);
                robot.WobbleServo.setPosition(0.7);
                Strafe(3200, 0.8);
                robot.Wobble.setPower(-1);
                sleep(300);
                robot.Wobble.setPower(0);
                DriveStraightDistance(-400, 0.8, false);

                //DriveStraightDistanceColor(-2700, 1);
                //DriveStraightDistance(-500, 1, false);
                //Strafe(-400, 0.8);
                Turn(1445, 0.8, true);
                Strafe(-600, 0.8);
                offset -= currentHeading;
                Strafe(1590, 0.8);
                DriveStraightDistance(-600, 0.8, false);
                DriveStraightDistance(200, 0.8, false);
                ShootPowershots(true);
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
                DriveStraightDistanceColor(-1000, 1);
        }
        checkOrientation();
        HeadingHolder.setHeading(currentHeading);
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

    private void DriveStraightDistance(int distance, double power, boolean fast) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            checkOrientation();
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
            }
        }
        StopDriving();
        if(!fast) {
            sleep(10);
        }
    }

    private void DriveStraightDistanceSquared(int distance, double power) {
        DriveStraightDistance(1400, 0.8, false);
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(power);
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

    private void Strafe(int distance, double power) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() - distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            checkOrientation();
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
            }
        }
        StopDriving();
        sleep(10);
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

    private void Turn(int distance, double power, boolean half) {
        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        if(half){
            targetHeading = 90;
        }

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            checkOrientation();
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
            }
        }
        if(half) {
            checkOrientation();
            offset -= currentHeading;
            targetHeading = 0;
        }
        StopDriving();
        sleep(10);
    }

    private void ShootPowershots(boolean three){
        int i = 0;
        int j = 1;
        if(three){
            j = 2;
        }
        robot.Launcher.setPower(0.7);
        sleep(1500);
        double time = System.nanoTime()/1000000000;
        while(i <= j) {
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
                if(i == 0) {
                    sleep(1150);
                    Strafe(400, 0.8);
                }
                else if(i == 1){
                    sleep(250);
                    Strafe(490, 0.8);
                }
                else if(i > 1){
                    sleep(1000);
                }
                i++;
            //}
            if(System.nanoTime()/1000000000 > time + 7){
                i = 3;
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

    private void checkOrientation() {
        // read the orientation of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.imu.getPosition();
        // and save the heading
        currentHeading = angles.firstAngle - offset;
    }
}
