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

    Orientation angles; //Safety Goggles are for the weak

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

        Strafe(-100, 0.6);
        DriveStraightDistanceSquared(400, 0.4);
        Strafe(1500, 0.6);
        DriveStraightDistance(2000, 0.8);
        DriveStraightDistanceColor(1000, 0.4); //was color
        Strafe(250, 0.6);
        Strafe(-50, 0.6);
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
                DriveStraightDistance(-300, 0.8);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistanceFast(-300, 1);
                sleep(200);
                robot.Collector.setPower(0);
                DriveStraightDistance(-500, 0.8);
                Strafe(-1450, 0.6);
                Turn(1445, 0.8, true);
                Turn(1445, 0.8, true);
                //Strafe(2000, 0.8);
                //Strafe(-1600, 0.8);
                //DriveStraightDistance(-1000, 0.8);
                //DriveStraightDistance(1000, 0.8);
                ShootPowershots(true);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 180 && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 120cm - 180cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistanceColor(-1000, 0.4);
                break;
            case 2:
                telemetry.addLine("There is 1 ring in the stack");
                telemetry.update();
                DriveStraightDistance(800, 0.8);
                Strafe(-1600, -0.6);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistanceFast(-300, 1);
                sleep(200);
                robot.Collector.setPower(0);
                DriveStraightDistanceColor(-950, 0.8);
                DriveStraightDistance(-500, 0.8);
                Strafe(150, 0.6);
                Turn(1445, 0.8, true);
                Turn(1445, 0.8, true);
                //Strafe(2000, 0.8);
                //Strafe(-1600, 0.8);
                //DriveStraightDistance(-1000, 0.8);
                //DriveStraightDistance(1000, 0.8);
                ShootPowershots(true);
                Strafe(-750, 0.8);
                DriveStraightDistance(1000, 0.8);
                robot.Collector.setPower(1);
                robot.Elevator.setPower(0.5);
                DriveStraightDistance(500, 0.2);
                robot.Collector.setPower(0);
                robot.Elevator.setPower(0);
                DriveStraightDistance(-1500, 0.8);
                Strafe(-1000, 0.6);
                robot.Launcher.setPower(0.8);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                robot.Collector.setPower(1);
                sleep(1000);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 120 && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 60cm - 120cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistanceColor(-1000, 0.4);
                break;
            case 3:
                telemetry.addLine("There are 4 rings in the stack");
                telemetry.update();
                DriveStraightDistance(2500, 0.8);
                robot.Collector.setPower(0.5);
                sleep(1000);
                DriveStraightDistanceFast(-300, 1);
                sleep(200);
                robot.Collector.setPower(0);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 60  && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 0cm - 60cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistanceColor(-2700, 0.8);
                DriveStraightDistance(-500, 0.8);
                Strafe(-1450, 0.6);
                Turn(1445, 0.8, true);
                Turn(1445, 0.8, true);
                //Strafe(2000, 0.8);
                //Strafe(-1600, 0.8);
                //DriveStraightDistance(-1000, 0.8);
                //DriveStraightDistance(1000, 0.8);
                ShootPowershots(true);
                Strafe(-750, 0.8);
                DriveStraightDistance(500, 0.8);
                robot.Collector.setPower(1);
                robot.Elevator.setPower(0.5);
                DriveStraightDistance(200, 0.2);
                robot.Collector.setPower(0);
                robot.Elevator.setPower(0);
                DriveStraightDistance(-700, 0.8);
                Strafe(-1000, 0.6);
                robot.Launcher.setPower(0.8);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                robot.Collector.setPower(1);
                sleep(3000);
                DriveStraightDistanceColor(-1000, 0.4);
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

    private void DriveStraightDistance(int distance, double power) {
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
        sleep(100);
    }

    private void DriveStraightDistanceFast(int distance, double power) {
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
    }

    private void DriveStraightDistanceSquared(int distance, double power) {
        DriveStraightDistance(1400, 0.8);
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
        sleep(100);
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
        sleep(100);
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
        sleep(100);
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
        sleep(100);
    }

    private void ShootPowershots(boolean three){
        int i = 0;
        int j = 1;
        if(three){
            j = 2;
        }
        robot.Launcher.setPower(0.7);
        sleep(1000);
        while(i < j) {
            if(robot.ringSensorColor.red() > 1000 || robot.ringSensorColor.green() > 1000){ringLoaded = true;}
            else{ringLoaded = false;}
            robot.ServoElevate.setPower(-1);
            robot.Elevator.setPower(0.5);
            robot.Collector.setPower(1);
            if(ringLoaded || i == 0){
                while(ringLoaded){
                    if(robot.ringSensorColor.red() > 1000 || robot.ringSensorColor.green() > 1000){ringLoaded = true;}
                    else{ringLoaded = false;}
                    idle();
                }
                if(i == 0) {
                    sleep(1000);
                }
                else if (i==1){
                    sleep(250);
                }
                else {

                    sleep(2000);
                }
                Strafe(750, 0.8);
                i++;
            }
            idle();
        }
        robot.ServoElevate.setPower(0);
        robot.Elevator.setPower(0);
        robot.Collector.setPower(0);
        robot.Launcher.setPower(0);
    }

    private void checkOrientation() {
        // read the orientation of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.imu.getPosition();
        // and save the heading
        currentHeading = angles.firstAngle - offset;
    }
}
