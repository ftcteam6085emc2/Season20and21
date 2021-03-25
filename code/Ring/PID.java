package org.firstinspires.ftc.Season20and21.code.Ring;

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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "PID", group = "Autonomous")
public class PID extends LinearOpMode {
    double targetHeading = 0;
    double currentHeading = 0;
    double turn_error;
    double offset = 0;
    double current_time;
    double prior_time;
    double current_error;
    double prior_error;
    double k_p = 0.025;
    double k_i = 0;
    double k_d = 0.0002;
    double p;
    double i;
    double d;
    double turn_k_p = 0.1;
    double steer;
    double output;
    double i_max = 10;

    Orientation angles;

    RingleaderHWMapSensorsColor robot = new RingleaderHWMapSensorsColor();

    @Override
    public void runOpMode(){
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

        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        checkOrientation();
        offset = currentHeading;

        Turn(1450, 1, true);
    }

    private void DriveStraight(double leftPower, double rightPower) {
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

        /*robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);
        */
        double targetPosition = robot.FrontRight.getCurrentPosition()+distance;
        DriveStraight(-1, 1);
        while (opModeIsActive()) {
            checkOrientation();
            current_time = getRuntime();
            current_error = targetPosition - robot.FrontRight.getCurrentPosition();
            p = current_error;
            i += (current_error * (current_time - prior_time));
            if(i > i_max){
                i = i_max;
            }
            else if (i < -i_max){
                i = -i_max;
            }
            d = ((current_error - prior_error) / (current_time - prior_time));
            output = Range.clip((k_p * p) + (k_i * i) + (k_d * d), -1, 1);
            prior_time = current_time;
            prior_error = current_error;

            /*turn_error = targetHeading - currentHeading;
            while(turn_error > 180){
                turn_error -= 360;
            }
            while(turn_error < -180){
                turn_error += 360;
            }

            steer = Range.clip(turn_k_p*turn_error, -1, 1);*/
            steer = 0;

            DriveStraight(-(output - steer), output + steer);
            telemetry.addLine("Error: "+current_error);
            telemetry.addLine("Output: "+output);
            telemetry.addLine("k_p: "+k_p);
            telemetry.update();
            sleep((long)((current_time-prior_time)*1000));
        }
        StopDriving();
        if(!fast) {
            sleep(10);
        }
    }

    private void Turn(int distance, double power, boolean half) {
        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        if(half){
            targetHeading = 90;
        }

        DriveStraight(power, power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
            checkOrientation();
            p = robot.FrontRight.getTargetPosition() - robot.FrontRight.getCurrentPosition();
            turn_error = targetHeading - currentHeading;
            while(turn_error > 180){
                turn_error -= 360;
            }
            while(turn_error < -180){
                turn_error += 360;
            }
            output = Range.clip(k_p*p, -1, 1);
            steer = Range.clip(turn_k_p*turn_error, -2, 2);
            DriveStraight(output - steer, output + steer);
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

    private void checkOrientation() {
        // read the orientation of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.imu.getPosition();
        // and save the heading
        currentHeading = angles.firstAngle - offset;
    }
}
