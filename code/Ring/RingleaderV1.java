package org.firstinspires.ftc.teamcode.Season20and21.code.Ring;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Season20and21.code.HeadingHolder;

@TeleOp(name = "RingleaderV1", group = "Test")
public class RingleaderV1 extends OpMode {

    RingleaderHWMap robot = new RingleaderHWMap();
    double power = 0.5;
    boolean dualMode = true;
    boolean powerIncrement = true;
    boolean spinning = false;
    boolean leftCheck = true;
    boolean rightCheck = true;
    boolean backCheck = true;
    boolean expert = false;
    boolean expertCheck = true;
    //boolean checkingOrientation = false;
    //boolean rightStickReleased = true;
    double targetHeading = 0;
    double currentHeading = 0;
    int targetChanging = 90;
    double autoHeading = HeadingHolder.getHeading();

    Orientation angles;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {
        if(gamepad1.back && backCheck){
            dualMode = !dualMode;
            backCheck = false;
        }
        else if(!gamepad1.back){
            backCheck = true;
        }

        if(gamepad1.right_stick_button && expertCheck){
            expert = !expert;
            expertCheck = false;
        }
        else if(!gamepad1.right_stick_button){
            expertCheck = true;
        }
        telemetry.addLine("Dual Mode: "+dualMode);
        if(dualMode && expert){
            checkOrientation();
            /*if (checkingOrientation) {
                checkOrientation();
            }
            if (gamepad1.right_stick_button && rightStickReleased) {
                checkingOrientation = !checkingOrientation;
                rightStickReleased = false;
            } else if (!gamepad1.right_stick_button) {
                rightStickReleased = true;
            }*/
            telemetry.addLine("Power is at: " + power);
            telemetry.addLine("Target Heading: " + targetHeading);
            telemetry.addLine("Current Heading: " + currentHeading);
            telemetry.addLine("Target Changing: " + targetChanging);
            robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            robot.FrontRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
            robot.RearRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);

            if (gamepad2.right_bumper) {   //If I wanted to free up the bumpers, I could make x/y/a/b toggles with a bool or two
                robot.Collector.setPower(0);
            } else if (gamepad2.b) {
                robot.Collector.setPower(1);
            } else if (gamepad2.x) {
                robot.Collector.setPower(-1);
            }

            if (gamepad2.left_bumper) {
                robot.Launcher.setPower(0);
                spinning = false;
            } else if (gamepad2.y) {
                robot.Launcher.setPower(Math.abs(power));
                spinning = true;
            } else if (gamepad2.a) {
                robot.Launcher.setPower(-Math.abs(power));
                spinning = true;
            }
            if (spinning) {
                telemetry.addLine(6000 * power + " RPM!");
            }

            if (gamepad2.right_trigger > 0) {
                robot.Elevator.setPower(1);
            } else if (gamepad2.left_trigger > 0) {
                robot.Elevator.setPower(-1);
            } else {
                robot.Elevator.setPower(0);
            }

            if (gamepad2.dpad_down && powerIncrement) {   // 0.9 & 1.0 are too high, 0.7 & 0.8 go into the high goal,
                power -= 0.1;   // 0.6 & 0.7 (rebound) can knock down powershots, 0.5 & 0.6 go into middle goal,
                if (power < 0.1) {   // and 0.4 & 0.5 go into the low goal, all from right behind white line
                    power = 0.1;   // Also there's a 15-30 degree angle to the right that the rings will travel
                }
                powerIncrement = false;
            } else if (gamepad2.dpad_up && powerIncrement) {   //After more testing I will change all of these to +/- the difference between the goals
                power += 0.1;
                if (power > 1) {
                    power = 1;
                }
                powerIncrement = false;
            } else if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
                powerIncrement = true;
            }

            if (gamepad2.dpad_left && leftCheck) {
                if (targetChanging == 90) {
                    targetChanging /= 2;
                } else {
                    targetChanging = 90;
                }
                leftCheck = false;
            }
            else if (!gamepad2.dpad_left){
                leftCheck = true;
            }

            if (gamepad2.dpad_right && rightCheck) {
                targetHeading -= targetChanging;
                if (targetHeading == -360 || targetHeading == 360) {
                    targetHeading = 0;
                }
                rightCheck = false;
            }
            else if(!gamepad2.dpad_right){
                rightCheck = true;
            }
            if (gamepad1.start || gamepad2.start) {
                while (!(currentHeading < targetHeading + 1 && currentHeading > targetHeading - 1)) {
                    checkOrientation();
                    if (currentHeading < targetHeading + 1) {
                        robot.FrontRight.setPower(0.5);
                        robot.FrontLeft.setPower(0.5);
                        robot.RearRight.setPower(0.5);
                        robot.RearLeft.setPower(0.5);
                    } else if (currentHeading > targetHeading - 1) {
                        robot.FrontRight.setPower(-0.5);
                        robot.FrontLeft.setPower(-0.5);
                        robot.RearRight.setPower(-0.5);
                        robot.RearLeft.setPower(-0.5);
                    }
                }
                robot.FrontRight.setPower(0);
                robot.FrontLeft.setPower(0);
                robot.RearRight.setPower(0);
                robot.RearLeft.setPower(0);
            }

            if (gamepad2.left_stick_button) {
                autoHeading = 0;
            }

            if(gamepad1.right_bumper){
                Strafe(100, 1);
            }
            else if(gamepad1.left_bumper){
                Strafe(-100, 1);
            }
            else if(gamepad1.right_trigger > 0){
                Strafe(100, 0.5);
            }
            else if(gamepad1.left_trigger > 0){
                Strafe(-100, 0.5);
            }
        }
        else if(expert){
            checkOrientation();
            /*if (checkingOrientation) {
                checkOrientation();
            }
            if (gamepad1.right_stick_button && rightStickReleased) {
                checkingOrientation = !checkingOrientation;
                rightStickReleased = false;
            } else if (!gamepad1.right_stick_button) {
                rightStickReleased = true;
            }*/
            telemetry.addLine("Power is at: " + power);
            telemetry.addLine("Target Heading: " + targetHeading);
            telemetry.addLine("Current Heading: " + currentHeading);
            telemetry.addLine("Target Changing: " + targetChanging);
            robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            robot.FrontRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
            robot.RearRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);

            if (gamepad1.right_bumper) {   //If I wanted to free up the bumpers, I could make x/y/a/b toggles with a bool or two
                robot.Collector.setPower(0);
            } else if (gamepad1.b) {
                robot.Collector.setPower(1);
            } else if (gamepad1.x) {
                robot.Collector.setPower(-1);
            }

            if (gamepad1.left_bumper) {
                robot.Launcher.setPower(0);
                spinning = false;
            } else if (gamepad1.y) {
                robot.Launcher.setPower(Math.abs(power));
                spinning = true;
            } else if (gamepad1.a) {
                robot.Launcher.setPower(-Math.abs(power));
                spinning = true;
            }
            if (spinning) {
                telemetry.addLine(6000 * power + " RPM!");
            }

            if (gamepad1.right_trigger > 0) {
                robot.Elevator.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                robot.Elevator.setPower(-1);
            } else {
                robot.Elevator.setPower(0);
            }

            if (gamepad1.dpad_down && powerIncrement) {   // 0.9 & 1.0 are too high, 0.7 & 0.8 go into the high goal,
                power -= 0.1;   // 0.6 & 0.7 (rebound) can knock down powershots, 0.5 & 0.6 go into middle goal,
                if (power < 0.1) {   // and 0.4 & 0.5 go into the low goal, all from right behind white line
                    power = 0.1;   // Also there's a 15-30 degree angle to the right that the rings will travel
                }
                powerIncrement = false;
            } else if (gamepad1.dpad_up && powerIncrement) {
                power += 0.1;
                if (power > 1) {
                    power = 1;
                }
                powerIncrement = false;
            } else if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                powerIncrement = true;
            }

            if (gamepad1.dpad_left) {
                if (targetChanging == 90) {
                    targetChanging /= 2;
                } else {
                    targetChanging = 90;
                }
            }

            if (gamepad1.dpad_right) {
                targetHeading -= targetChanging;
                if (targetHeading == -360) {
                    targetHeading = 360;
                }
            }
            if (gamepad1.start) {
                while (!(currentHeading < targetHeading + 1 && currentHeading > targetHeading - 1)) {
                    checkOrientation();
                    if (currentHeading < targetHeading + 1) {
                        robot.FrontRight.setPower(0.5);
                        robot.FrontLeft.setPower(0.5);
                        robot.RearRight.setPower(0.5);
                        robot.RearLeft.setPower(0.5);
                    } else if (currentHeading > targetHeading - 1) {
                        robot.FrontRight.setPower(-0.5);
                        robot.FrontLeft.setPower(-0.5);
                        robot.RearRight.setPower(-0.5);
                        robot.RearLeft.setPower(-0.5);
                    }
                }
                robot.FrontRight.setPower(0);
                robot.FrontLeft.setPower(0);
                robot.RearRight.setPower(0);
                robot.RearLeft.setPower(0);
            }

            if (gamepad1.left_stick_button) {
                autoHeading = 0;
            }
        }
        else if(dualMode){
            telemetry.addLine("Power is at: " + power);
            telemetry.addLine("Current Heading: " + currentHeading);
            robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            robot.FrontRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
            robot.RearRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);

            if (gamepad2.right_bumper || gamepad2.x) {   //If I wanted to free up the bumpers, I could make x/y/a/b toggles with a bool or two
                robot.Collector.setPower(0);
            } else if (gamepad2.b) {
                robot.Collector.setPower(1);
            }

            if (gamepad2.left_bumper || gamepad2.a) {
                robot.Launcher.setPower(0);
                spinning = false;
            } else if (gamepad2.y) {
                robot.Launcher.setPower(Math.abs(power));
                spinning = true;
            }
            if (spinning) {
                telemetry.addLine(6000 * power + " RPM!");
            }

            if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                robot.Elevator.setPower(1);
            }
            else {
                robot.Elevator.setPower(0);
            }

            if (gamepad2.dpad_down && powerIncrement) {   // 0.9 & 1.0 are too high, 0.7 & 0.8 go into the high goal,
                power -= 0.1;   // 0.6 & 0.7 (rebound) can knock down powershots, 0.5 & 0.6 go into middle goal,
                if (power < 0.1) {   // and 0.4 & 0.5 go into the low goal, all from right behind white line
                    power = 0.1;   // Also there's a 15-30 degree angle to the right that the rings will travel
                }
                powerIncrement = false;
            } else if (gamepad2.dpad_up && powerIncrement) {
                power += 0.1;
                if (power > 1) {
                    power = 1;
                }
                powerIncrement = false;
            } else if (!gamepad2.dpad_down && !gamepad1.dpad_up) {
                powerIncrement = true;
            }
        }
        else {
            telemetry.addLine("Power is at: " + power);
            telemetry.addLine("Current Heading: " + currentHeading);
            robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            robot.FrontRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
            robot.RearRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);

            if (gamepad1.right_bumper || gamepad1.x) {   //If I wanted to free up the bumpers, I could make x/y/a/b toggles with a bool or two
                robot.Collector.setPower(0);
            } else if (gamepad1.b) {
                robot.Collector.setPower(1);
            }

            if (gamepad1.left_bumper || gamepad1.a) {
                robot.Launcher.setPower(0);
                spinning = false;
            } else if (gamepad1.y) {
                robot.Launcher.setPower(Math.abs(power));
                spinning = true;
            }
            if (spinning) {
                telemetry.addLine(6000 * power + " RPM!");
            }

            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                robot.Elevator.setPower(1);
            }
            else {
                robot.Elevator.setPower(0);
            }

            if (gamepad1.dpad_down && powerIncrement) {   // 0.9 & 1.0 are too high, 0.7 & 0.8 go into the high goal,
                power -= 0.1;   // 0.6 & 0.7 (rebound) can knock down powershots, 0.5 & 0.6 go into middle goal,
                if (power < 0.1) {   // and 0.4 & 0.5 go into the low goal, all from right behind white line
                    power = 0.1;   // Also there's a 15-30 degree angle to the right that the rings will travel
                }
                powerIncrement = false;
            } else if (gamepad1.dpad_up && powerIncrement) {
                power += 0.1;
                if (power > 1) {
                    power = 1;
                }
                powerIncrement = false;
            } else if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                powerIncrement = true;
            }
        }
    }

    private void checkOrientation() {
        // read the orientation of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.imu.getPosition();
        // and save the heading
        currentHeading = angles.firstAngle; //+ autoHeading;
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

    private void Strafe(int distance, double power) {
        telemetry.update();

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() - distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(power);
        while (robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) {}
        StopDriving();
    }
}