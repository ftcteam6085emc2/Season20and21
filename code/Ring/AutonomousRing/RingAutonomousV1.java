package org.firstinspires.ftc.Season20and21.code.Ring.AutonomousRing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.Season20and21.code.Ring.RingleaderHWMapSensorsColor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.Season20and21.code.HeadingHolder;

@Autonomous(name = "RingAutonomousV1", group = "Concept")
public class RingAutonomousV1 extends LinearOpMode {
    int tZone = 0;
    double targetHeading = 0;
    double currentHeading = 0;
    double offset = 0;
    int averageCount1 = 0;
    int averageCount2 = 0;
    int averageCount3 = 0;
    boolean whiteDetected = false;

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

        TurnIMU();
        stop();

        Strafe(-100, 0.6);
        DriveStraightDistance(1400, 0.8);
        DriveStraightDistanceSquared(400, 0.4);
        Strafe(1200, 0.6);
        DriveStraightDistance(2000, 0.8);
        DriveStraightDistanceColor(1000, 0.4); //was color
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
                robot.Collector.setPower(0);
                DriveStraightDistance(-500, 0.8);
                Strafe(-1600, 0.6);
                Turn(2900, 0.8, true);
                DriveStraightDistance(-1000, 0.8);
                DriveStraightDistance(1000, 0.8);
                robot.Launcher.setPower(0.7);
                sleep(1000);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(1000);
                robot.Launcher.setPower(0);
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
                robot.Collector.setPower(0);
                DriveStraightDistanceColor(-1250, 0.8);
                DriveStraightDistance(-250, 0.8);
                Turn(2900, 0.8, true);
                DriveStraightDistance(-1000, 0.8);
                DriveStraightDistance(1000, 0.8);
                robot.Launcher.setPower(0.7);
                sleep(1000);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(1000);
                robot.Launcher.setPower(0);
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
                robot.Collector.setPower(0);
                /*while (robot.sensorRangeTop.getDistance(DistanceUnit.CM) > 60  && robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 200) {   //Distance is 0cm - 60cm
                    DriveStraight(0.4);
                }
                StopDriving();*/
                DriveStraightDistanceColor(-3000, 0.8);
                DriveStraightDistance(-250, 0.8);
                Strafe(-1600, 0.6);
                Turn(2900, 0.8, true);
                DriveStraightDistance(-1000, 0.8);
                DriveStraightDistance(1000, 0.8);
                robot.Launcher.setPower(0.7);
                sleep(1000);
                robot.ServoElevate.setPower(-1);
                robot.Elevator.setPower(0.5);
                robot.Collector.setPower(1);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(1000);
                Strafe(500, 0.6);
                sleep(1000);
                robot.Launcher.setPower(0);
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

            if (robot.sensorRangeTop.getDistance(DistanceUnit.CM) < 14) {
                averageCount3++;
            } else if (robot.sensorRangeBottom.getDistance(DistanceUnit.CM) < 14) {
                averageCount2++;
            } else {
                averageCount1++;
            }
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
    }

    private void Turn(int distance, double power, boolean full) {
        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();

            if (full) {
                checkOrientation();
                if (currentHeading < -179 || currentHeading > 179) {
                    break;
                } else if (currentHeading < 0) {
                    robot.FrontRight.setPower(-0.1);
                    robot.FrontLeft.setPower(0.1);
                    robot.RearRight.setPower(-0.1);
                    robot.RearLeft.setPower(0.1);
                } else if (currentHeading < 90) {
                    robot.FrontRight.setPower(1);
                    robot.FrontLeft.setPower(-1);
                    robot.RearRight.setPower(1);
                    robot.RearLeft.setPower(-1);
                } else if (currentHeading < 135) {
                    robot.FrontRight.setPower(0.5);
                    robot.FrontLeft.setPower(-0.5);
                    robot.RearRight.setPower(0.5);
                    robot.RearLeft.setPower(-0.5);
                } else if (currentHeading < 165) {
                    robot.FrontRight.setPower(0.3);
                    robot.FrontLeft.setPower(-0.3);
                    robot.RearRight.setPower(0.3);
                    robot.RearLeft.setPower(-0.3);
                } else if (currentHeading < 180) {
                    robot.FrontRight.setPower(0.1);
                    robot.FrontLeft.setPower(-0.1);
                    robot.RearRight.setPower(0.1);
                    robot.RearLeft.setPower(-0.1);
                }
            }
        }

        StopDriving();
    }

    private void TurnIMU() {
        double ogtime = time;
        while (time < ogtime + 10) {
            checkOrientation();
            if (currentHeading < -179 || currentHeading > 179) {
                break;
            } else if (currentHeading < 0) {
                robot.FrontRight.setPower(-0.1);
                robot.FrontLeft.setPower(0.1);
                robot.RearRight.setPower(-0.1);
                robot.RearLeft.setPower(0.1);
            } else if (currentHeading < 90) {
                robot.FrontRight.setPower(1);
                robot.FrontLeft.setPower(-1);
                robot.RearRight.setPower(1);
                robot.RearLeft.setPower(-1);
            } else if (currentHeading < 135) {
                robot.FrontRight.setPower(0.5);
                robot.FrontLeft.setPower(-0.5);
                robot.RearRight.setPower(0.5);
                robot.RearLeft.setPower(-0.5);
            } else if (currentHeading < 165) {
                robot.FrontRight.setPower(0.3);
                robot.FrontLeft.setPower(-0.3);
                robot.RearRight.setPower(0.3);
                robot.RearLeft.setPower(-0.3);
            } else if (currentHeading < 180) {
                robot.FrontRight.setPower(0.1);
                robot.FrontLeft.setPower(-0.1);
                robot.RearRight.setPower(0.1);
                robot.RearLeft.setPower(-0.1);
            }
        }
    }

    private void checkOrientation() {
        // read the orientation of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.imu.getPosition();
        // and save the heading
        currentHeading = angles.firstAngle - offset;
    }
}
