package org.firstinspires.ftc.teamcode.Season20and21.code.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Season20and21.code.TeleOp.HWMap;

import java.util.Locale;

@Autonomous(name = "AutoTestingCleanIMU", group = "Concept")
public class AutoTestingCleanIMU extends LinearOpMode {
    boolean scanned = true;
    int tZone = 0;
    double targetHeading = 0;
    double currentHeading = 0;
    String heading = "0";

    Orientation angles;
    Acceleration gravity;

    HWMap robot = new HWMap();

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
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //heading = formatAngle(angles.angleUnit, angles.firstAngle);
        //telemetry.addLine(heading);

        DriveStraightDistance(1200, 0.4);
        sleep(1000);
        DriveStraightDistance(-1200, -0.4);
        sleep(3000);

        //"Detect" the ring stack height: 1 is 0 rings, 2 is 1 ring, and 3 is 3 rings
        tZone = (int) ((Math.random() * 3) + 1);
        switch (tZone) {
            case 1:
                DriveStraightDistance(-3600, 0.8);
                break;
            case 2:
                DriveStraightDistance(500, 0.8);
                Strafe(-300, -0.6);
                DriveStraightDistance(-100, 0.8);
                Strafe(300, 0.6);
                DriveStraightDistance(-5300, 0.8);
                break;
            case 3:
                DriveStraightDistance(2400, 0.8);
                DriveStraightDistance(-6000, 0.8);
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
            /*currentHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            if(currentHeading > targetHeading + 3){
                robot.FrontRight.setPower(power - 0.1);
                robot.FrontLeft.setPower(power + 0.1);
                robot.RearRight.setPower(power - 0.1);
                robot.RearLeft.setPower(power + 0.1);
            }
            else if  (currentHeading < targetHeading - 3){
                robot.FrontRight.setPower(power + 0.1);
                robot.FrontLeft.setPower(power - 0.1);
                robot.RearRight.setPower(power + 0.1);
                robot.RearLeft.setPower(power - 0.1);
            }
            else {
                robot.FrontRight.setPower(power);
                robot.FrontLeft.setPower(power);
                robot.RearRight.setPower(power);
                robot.RearLeft.setPower(power);
            }*/
        }

        StopDriving();
    }

    private void Strafe(int distance, double power) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() - distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
        }

        StopDriving();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
