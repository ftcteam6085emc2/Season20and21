package org.firstinspires.ftc.Season20and21.code.Ring;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.Season20and21.code.Ring.HWMap.RingleaderHWMapSensorsColor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.Season20and21.code.HeadingHolder;

@TeleOp(name = "RingleaderV1", group = "Test")
public class RingleaderV1 extends OpMode {

    RingleaderHWMapSensorsColor robot = new RingleaderHWMapSensorsColor();
    double power = 0.5;
    double targetHeading = 0;
    double whiteTime = 0;
    boolean ringLoaded = false;
    boolean dualMode = true;
    boolean powerIncrement = true;
    //boolean leftCheck = true;
    //boolean rightCheck = true;
    //boolean backCheck = true;
    boolean expert = true;
    boolean expertCheck = true;
    boolean collectorCheck = true;
    boolean launcherCheck = true;
    boolean servoCheck = true;
    boolean powerUpdate = false;
    //boolean startCheck = true;
    //double targetHeading = 0;
    double currentHeading = 0;
    int a = 0;
    int b = 0;
    int x = 0;
    int y = 0;
    //int dividor = 3;
    boolean startCheck1 = true;
    int start1 = 0;
    int leftBumper = 0;
    int rightBumper = 0;
    //int targetChanging = 90;
    double autoHeading = HeadingHolder.getHeading();

    Orientation angles;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.Collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.Wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.Wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        checkOrientation();

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        robot.blinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    @Override
    public void loop() {
        if(whiteTime < getRuntime()) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
            robot.blinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        ringLoaded = robot.ringSensorColor.red() > 1000 || robot.ringSensorColor.green() > 1000;
        /*if(gamepad1.back && backCheck){
            dualMode = !dualMode;
            backCheck = false;
        }
        else if(!gamepad1.back){
            backCheck = true;
        }*/

        telemetry.addLine("Color Sensor Alpha is:" + robot.sensorColor.alpha()); //No one will ever use this, so be it
        telemetry.addLine("Ring loaded: "+ ringLoaded);
        if(robot.sensorColor.alpha() > 800){
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            robot.blinkinLedDriver2.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            whiteTime = getRuntime() + 0.1;
        }

        if(gamepad1.right_stick_button && expertCheck){   //Cringe, but it looks good in control award
            expert = !expert;
            expertCheck = false;
        }
        else if(!gamepad1.right_stick_button){
            expertCheck = true;
        }
        telemetry.addLine("Dual Mode: "+dualMode);
        if(dualMode && expert){
            checkOrientation();
            telemetry.addLine("Power is at: " + power);
            //telemetry.addLine("Target Heading: " + targetHeading);
            telemetry.addLine("Current Heading: " + currentHeading);
            //telemetry.addLine("Target Changing: " + targetChanging);
            robot.FrontLeft.setVelocity(2700*(gamepad1.left_stick_y + gamepad1.left_stick_x));
            robot.RearLeft.setVelocity(2700*(gamepad1.left_stick_y - gamepad1.left_stick_x));

            robot.FrontRight.setVelocity(2700*(-gamepad1.right_stick_y - gamepad1.right_stick_x));
            robot.RearRight.setVelocity(2700*(-gamepad1.right_stick_y + gamepad1.right_stick_x));

            /*if(gamepad1.dpad_up){
                dividor = 1;
            }
            else if(gamepad1.dpad_down){
                dividor = 3;
            }*/

            /*if(dividor == 1) {
                if (gamepad1.dpad_left && dpadLeftCheck) {
                    MoveWobble(switcher, true);
                    switcher = !switcher;
                    dpadLeftCheck = false;
                } else if (!gamepad1.dpad_left) {
                    dpadLeftCheck = true;
                }
            }
            else if (dividor == 3){
                if (gamepad1.dpad_left && dpadLeftCheck) {
                    MoveWobble(switcher, false);
                    switcher = !switcher;
                    dpadLeftCheck = false;
                } else if (!gamepad1.dpad_left) {
                    dpadLeftCheck = true;
                }
            }*/

            if(gamepad1.dpad_right){
                Strafe(700, 0.8);
            }

            /*if (gamepad1.dpad_left && leftCheck) {
                if (targetChanging == 90) {
                    targetChanging /= 2;
                } else {
                    targetChanging = 90;
                }
                leftCheck = false;
            }
            else if (!gamepad1.dpad_left){
                leftCheck = true;
            }

            if (gamepad1.dpad_right && rightCheck) {
                targetHeading += targetChanging;
                if(targetHeading == 270) {
                    targetHeading = -90;
                }
                rightCheck = false;
            }
            else if(!gamepad1.dpad_right){
                rightCheck = true;
            }
            if (gamepad1.left_stick_button) { //Not right now, this sucks. At 180 degrees everything breaks due to rollover
                while ((!(currentHeading < targetHeading + 0.25 && currentHeading > targetHeading - 0.25)) || (targetHeading == 180 && (currentHeading < -178 || currentHeading > 178))) {
                    checkOrientation();
                    if(currentHeading < -170 && targetHeading == 180){
                        robot.FrontRight.setPower(-0.1);
                        robot.FrontLeft.setPower(-0.1);
                        robot.RearRight.setPower(-0.1);
                        robot.RearLeft.setPower(-0.1);
                    }
                    else if (currentHeading < targetHeading - 45) {
                        robot.FrontRight.setPower(1);
                        robot.FrontLeft.setPower(1);
                        robot.RearRight.setPower(1);
                        robot.RearLeft.setPower(1);
                    } else if (currentHeading > targetHeading + 45) {
                        robot.FrontRight.setPower(-1);
                        robot.FrontLeft.setPower(-1);
                        robot.RearRight.setPower(-1);
                        robot.RearLeft.setPower(-1);
                    } else if (currentHeading < targetHeading - 15) {
                        robot.FrontRight.setPower(0.5);
                        robot.FrontLeft.setPower(0.5);
                        robot.RearRight.setPower(0.5);
                        robot.RearLeft.setPower(0.5);
                    } else if (currentHeading > targetHeading + 15) {
                        robot.FrontRight.setPower(-0.5);
                        robot.FrontLeft.setPower(-0.5);
                        robot.RearRight.setPower(-0.5);
                        robot.RearLeft.setPower(-0.5);
                    } else if (currentHeading < targetHeading - 5) {
                        robot.FrontRight.setPower(0.2);
                        robot.FrontLeft.setPower(0.2);
                        robot.RearRight.setPower(0.2);
                        robot.RearLeft.setPower(0.2);
                    } else if (currentHeading > targetHeading + 5) {
                        robot.FrontRight.setPower(-0.2);
                        robot.FrontLeft.setPower(-0.2);
                        robot.RearRight.setPower(-0.2);
                        robot.RearLeft.setPower(-0.2);
                    } else if (currentHeading < targetHeading) {
                        robot.FrontRight.setPower(0.1);
                        robot.FrontLeft.setPower(0.1);
                        robot.RearRight.setPower(0.1);
                        robot.RearLeft.setPower(0.1);
                    } else if (currentHeading > targetHeading) {
                        robot.FrontRight.setPower(-0.1);
                        robot.FrontLeft.setPower(-0.1);
                        robot.RearRight.setPower(-0.1);
                        robot.RearLeft.setPower(-0.1);
                    }
                }
                robot.FrontRight.setPower(0);
                robot.FrontLeft.setPower(0);
                robot.RearRight.setPower(0);
                robot.RearLeft.setPower(0);
            }

            if (gamepad2.left_stick_button && startCheck) {
                checkOrientation();
                autoHeading = currentHeading;
                startCheck = false;
            } else if (!gamepad2.left_stick_button){
                leftStickCheck = true;
            }

            if(gamepad1.left_stick_button && leftStickCheck1 && leftStick1 == 0){
                robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftStick1++;
                leftStickCheck1 = false;
            }
            else if(gamepad1.left_stick_button && leftStickCheck1 && leftStick1 == 1){
                robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftStick1--;
                leftStickCheck1 = false;
            }
            else if(!gamepad1.left_stick_button){
                leftStickCheck1 = true;
            }

            if(gamepad2.b){
                robot.Collector.setPower(1);
            }
            else if (gamepad2.x){
                robot.Collector.setPower(-1);
            }
            else {
                robot.Collector.setPower(0);
            }
            /*if (gamepad2.b && collectorCheck) {
                robot.Collector.setPower(1);
                b++;
                if(b > 1){
                    robot.Collector.setPower(0);
                    b = 0;
                }
                x = 0;
                collectorCheck = false;
            } else if (gamepad2.x && collectorCheck) {
                robot.Collector.setPower(-1);
                x++;
                if(x > 1){
                    robot.Collector.setPower(0);
                    x = 0;
                }
                b = 0;
                collectorCheck = false;
            } else if(!(gamepad2.b || gamepad2.x)){
                collectorCheck = true;
            }*/
            if(powerUpdate){
                robot.Launcher.setPower(Math.abs(power));
                powerUpdate = false;
            }
            if (gamepad2.y && launcherCheck) {
                robot.Launcher.setPower(Math.abs(power));
                y++;
                if(y > 1){
                    robot.Launcher.setPower(0);
                    y = 0;
                }
                a = 0;
                launcherCheck = false;
            } else if (gamepad2.a && launcherCheck) {
                robot.Launcher.setPower(-Math.abs(power)); // 1 sec for collector, 2 sec start up for launcher, 0.70 for elevator
                a++;
                if(a > 1){
                    robot.Launcher.setPower(0);
                    a = 0;
                }
                y = 0;
                launcherCheck = false;
            } else if (!(gamepad2.a || gamepad2.y)){
                launcherCheck = true;
            }
            if (a > 0 || y > 0) {
                telemetry.addLine(6000 * power + " RPM!");
            }

            if (gamepad2.right_trigger > 0) {
                robot.Elevator.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0) {
                robot.Elevator.setPower(-gamepad2.left_trigger);
            } else {
                robot.Elevator.setPower(0);
            }

            if (gamepad2.dpad_down && powerIncrement) {   // 0.9 & 1.0 are too high, 0.7 & 0.8 go into the high goal,
                power -= 0.1;   // 0.6 & 0.7 (rebound) can knock down powershots, 0.5 & 0.6 go into middle goal,
                if (power < 0.1) {   // and 0.4 & 0.5 go into the low goal, all from right behind white line
                    power = 0.1;   // Also there's a 15-30 degree angle to the right that the rings will travel
                }
                powerIncrement = false;
                if(robot.Launcher.getPower() > 0){
                    powerUpdate = true;
                }
            } else if (gamepad2.dpad_up && powerIncrement) {   //After more testing I will change all of these to +/- the difference between the goals
                power += 0.1;
                if (power > 1) {
                    power = 1;
                }
                powerIncrement = false;
                if(robot.Launcher.getPower() > 0){
                    powerUpdate = true;
                }
            } else if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
                powerIncrement = true;
            }

            if (gamepad2.left_bumper && servoCheck){
                robot.ServoElevate.setPower(1);
                leftBumper++;
                if(leftBumper > 1){
                    robot.ServoElevate.setPower(0);
                    leftBumper = 0;
                }
                rightBumper = 0;
                servoCheck = false;
            }
            else if(gamepad2.right_bumper && servoCheck){
                robot.ServoElevate.setPower(-1);
                rightBumper++;
                if(rightBumper > 1){
                    robot.ServoElevate.setPower(0);
                    rightBumper = 0;
                }
                leftBumper = 0;
                servoCheck = false;
            }

            else if (!(gamepad2.right_bumper || gamepad2.left_bumper)){
                servoCheck = true;
                robot.ServoElevate.setPower(gamepad2.left_stick_y);
            }

            if(gamepad1.right_trigger > 0){
                robot.Wobble.setPower(gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0){
                robot.Wobble.setPower(-gamepad1.left_trigger);
            }
            else {
                robot.Wobble.setPower(0);
            }

            if(gamepad1.right_bumper){
                robot.WobbleServo.setPosition(0.7);
            }
            else if (gamepad1.left_bumper){
                robot.WobbleServo.setPosition(0.2);
            }

            if(gamepad1.a){
                robot.WobbleRotate.setPosition(0.7);
            }
            else if (gamepad1.b){
                robot.WobbleRotate.setPosition(0.2);
            }
            /*if (gamepad2.right_bumper) {
                robot.ServoElevate.setPower(-1);
            } else if (gamepad2.left_bumper) {
                robot.ServoElevate.setPower(1);
            } else {
                robot.ServoElevate.setPower(0);
            }*/
            if(gamepad1.start && startCheck1 && start1 == 0){
                robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                start1++;
                startCheck1 = false;
            }
            else if(gamepad1.start && startCheck1 && start1 == 1){
                robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                start1--;
                startCheck1 = false;
            }
            else if(!gamepad1.start){
                startCheck1 = true;
            }
        }
        else if(expert){
            checkOrientation();
            telemetry.addLine("Power is at: " + power);
            telemetry.addLine("Current Heading: " + currentHeading);
            robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            robot.FrontRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);
            robot.RearRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);

            if(gamepad1.right_bumper && servoCheck){
                robot.ServoElevate.setPower(-1);
                rightBumper++;
                if(rightBumper > 1){
                    robot.ServoElevate.setPower(0);
                    rightBumper = 0;
                }
                leftBumper = 0;
                servoCheck = false;
            }
            else if (gamepad1.left_bumper && servoCheck){
                robot.ServoElevate.setPower(1);
                leftBumper++;
                if(leftBumper > 1){
                    robot.ServoElevate.setPower(0);
                    leftBumper = 0;
                }
                rightBumper = 0;
                servoCheck = false;
            }
            else if (!(gamepad1.right_bumper || gamepad1.left_bumper)) {
                servoCheck = true;
            }
            
            if (gamepad1.b && collectorCheck) {
                robot.Collector.setPower(1);
                b++;
                if(b > 1){
                    robot.Collector.setPower(0);
                    b = 0;
                }
                x = 0;
                collectorCheck = false;
            } else if (gamepad1.x && collectorCheck) {
                robot.Collector.setPower(-1);
                x++;
                if(x > 1){
                    robot.Collector.setPower(0);
                    x = 0;
                }
                b = 0;
                collectorCheck = false;
            } else if(!(gamepad1.b || gamepad1.x)){
                collectorCheck = true;
            }

            if (gamepad1.y && launcherCheck) {
                robot.Launcher.setPower(Math.abs(power));
                y++;
                if(y > 1){
                    robot.Launcher.setPower(0);
                    y = 0;
                }
                a = 0;
                launcherCheck = false;
            } else if (gamepad1.a && launcherCheck) {
                robot.Launcher.setPower(-Math.abs(power));
                a++;
                if(a > 1){
                    robot.Launcher.setPower(0);
                    a = 0;
                }
                y = 0;
                launcherCheck = false;
            } else if (!(gamepad1.a || gamepad1.y)){
                launcherCheck = true;
            }
            if (a > 0 || y > 0) {
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

            /*if (gamepad1.dpad_left) {
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
            }*/
        }
        else if(dualMode){
            telemetry.addLine("Power is at: " + power);
            telemetry.addLine("Current Heading: " + currentHeading);
            robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            robot.FrontRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);
            robot.RearRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);

            if ((gamepad2.right_bumper || gamepad2.left_bumper) && servoCheck) {
                robot.ServoElevate.setPower(-1);
                rightBumper++;
                if(rightBumper > 1){
                    robot.ServoElevate.setPower(0);
                    rightBumper = 0;
                }
                servoCheck = false;
            }
            else if (!(gamepad2.right_bumper || gamepad2.left_bumper)){
                servoCheck = true;
                robot.ServoElevate.setPower(-Math.abs(gamepad2.left_stick_y));
            }

            if ((gamepad2.b || gamepad2.x) && collectorCheck) {
                robot.Collector.setPower(1);
                b++;
                if(b > 1){
                    robot.Collector.setPower(0);
                    b = 0;
                }
                collectorCheck = false;
            }
            else if (!(gamepad2.b || gamepad2.x)){
                collectorCheck = true;
            }

            if ((gamepad2.y || gamepad2.a) && launcherCheck) {
                robot.Launcher.setPower(Math.abs(power));
                y++;
                if(y > 1){
                    robot.Launcher.setPower(0);
                    y = 0;
                }
                launcherCheck = false;
            }
            else if (!(gamepad2.y || gamepad2.a)){
                launcherCheck = true;
            }
            if (y == 1) {
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

            if(gamepad1.start && startCheck1 && start1 == 0){
                robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                start1++;
                startCheck1 = false;
            }
            else if(gamepad1.start && startCheck1 && start1 == 1){
                robot.FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                start1--;
                startCheck1 = false;
            }
            else if(!gamepad1.start){
                startCheck1 = true;
            }

            /*if(gamepad1.right_trigger > 0){
                robot.Wobble.setPower(gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger > 0){
                robot.Wobble.setPower(-gamepad1.left_trigger);
            }
            else {
                robot.Wobble.setPower(0);
            }

            if(gamepad1.right_bumper){
                robot.WobbleServo.setPosition(0.5);
            }
            else if (gamepad1.left_bumper){
                robot.WobbleServo.setPosition(0);
            }*/
        }
        else {
            telemetry.addLine("Power is at: " + power);
            telemetry.addLine("Current Heading: " + currentHeading);
            robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            robot.FrontRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);
            robot.RearRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);

            if ((gamepad2.right_bumper || gamepad2.left_bumper) && servoCheck) {
                robot.ServoElevate.setPower(-1);
                rightBumper++;
                if(rightBumper > 1){
                    robot.ServoElevate.setPower(0);
                    rightBumper = 0;
                }
                servoCheck = false;
            }
            else if (!(gamepad2.right_bumper || gamepad2.left_bumper)){
                servoCheck = true;
            }


            if ((gamepad2.b || gamepad2.x) && collectorCheck) {
                robot.Collector.setPower(1);
                b++;
                if(b > 1){
                    robot.Collector.setPower(0);
                    b = 0;
                }
                collectorCheck = false;
            }
            else if (!(gamepad2.b || gamepad2.x)){
                collectorCheck = true;
            }

            if ((gamepad2.y || gamepad2.a) && launcherCheck) {
                robot.Launcher.setPower(Math.abs(power));
                y++;
                if (y > 1) {
                    robot.Launcher.setPower(0);
                    y = 0;
                }
                launcherCheck = false;
            }
            else if (!(gamepad2.y || gamepad2.a)){
                launcherCheck = true;
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
        currentHeading = angles.firstAngle - autoHeading;
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

        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() - distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() - distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy())) {
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
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*private void MoveWobble(boolean direction, boolean holding) {
        if(holding) {
            if (direction) {
                for (double power = 1; power >= -0.2; power -= 0.1) {
                    double time = getRuntime();
                    while (getRuntime() < time + 0.05) {
                        robot.Wobble.setPower(power);
                    }
                }
            } else {
                for (double power = -0.9; power <= 0.2; power += 0.1) {
                    double time = getRuntime();
                    while (getRuntime() < time + 0.051) {
                        robot.Wobble.setPower(power);
                    }
                }
            }
        }
        else{
            if (direction) {
                for (double power = 0.7; power >= -0.2; power -= 0.1) {
                    double time = getRuntime();
                    while (getRuntime() < time + 0.05) {
                        robot.Wobble.setPower(power);
                    }
                }
            } else {
                for (double power = -0.6; power <= 0.2; power += 0.1) {
                    double time = getRuntime();
                    while (getRuntime() < time + 0.051) {
                        robot.Wobble.setPower(power);
                    }
                }
            }
        }
    }*/
}