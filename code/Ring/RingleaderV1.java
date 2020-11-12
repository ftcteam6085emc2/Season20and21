package org.firstinspires.ftc.teamcode.Season20and21.code.Ring;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    boolean powerIncrement = true;
    boolean spinning = false;
    boolean checkingOrientation = false;
    boolean rightStickReleased = true;
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
        if(checkingOrientation) {
            checkOrientation();
        }
        if(gamepad1.right_stick_button && rightStickReleased){
            checkingOrientation = !checkingOrientation;
            rightStickReleased = false;
        }
        else if(!gamepad1.right_stick_button){
            rightStickReleased = true;
        }
        telemetry.addLine("Power is at: "+power);
        telemetry.addLine("Target Heading: " + targetHeading);
        telemetry.addLine("Current Heading: " + currentHeading);
        telemetry.addLine("Target Changing: " + targetChanging);
        robot.FrontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        robot.RearLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

        robot.FrontRight.setPower(-gamepad1.right_stick_y + gamepad1.right_stick_x);
        robot.RearRight.setPower(-gamepad1.right_stick_y - gamepad1.right_stick_x);

        if(gamepad1.right_bumper){   //If I wanted to free up the bumpers, I could make x/y/a/b toggles with a bool or two
            robot.Collector.setPower(0);
        }
        else if(gamepad1.b){
            robot.Collector.setPower(1);
        }
        else if(gamepad1.x){
            robot.Collector.setPower(-1);
        }

        if(gamepad1.left_bumper){
            robot.Launcher.setPower(0);
            spinning = false;
        }
        else if(gamepad1.y){
            robot.Launcher.setPower(Math.abs(power));
            spinning = true;
        }
        else if(gamepad1.a){
            robot.Launcher.setPower(-Math.abs(power));
            spinning = true;
        }
        if(spinning){
            telemetry.addLine(6000*power + " RPM!");
        }

        if(gamepad1.dpad_down && powerIncrement){   // 0.9 & 1.0 are too high, 0.7 & 0.8 go into the high goal,
            power -= 0.1;   // 0.6 & 0.7 (rebound) can knock down powershots, 0.5 & 0.6 go into middle goal,
            if(power < 0.1){   // and 0.4 & 0.5 go into the low goal, all from right behind white line
                power = 0.1;   // Also there's a 15-30 degree angle to the right that the rings will travel
            }
            powerIncrement = false;
        }
        else if (gamepad1.dpad_up && powerIncrement){
            power += 0.1;
            if (power > 1){
                power = 1;
            }
            powerIncrement = false;
        }
        else if(!gamepad1.dpad_down && !gamepad1.dpad_up){
            powerIncrement = true;
        }

        if(gamepad1.dpad_left){
            if(targetChanging == 90){
                targetChanging /= 2;
            }
            else if(targetChanging > 15){
                targetChanging -= 15;
            }
            else{
                targetChanging = 90;
            }
        }

        if(gamepad1.dpad_right){
            targetHeading -= targetChanging;
            if(targetHeading == -360 || targetHeading == 360){
                targetHeading = 0;
            }
        }
        if(gamepad1.start){
            checkOrientation();
            while(!(currentHeading < targetHeading + 1 && currentHeading > targetHeading - 1)) {
                if (currentHeading < targetHeading + 1) {
                    robot.FrontRight.setPower(0.1);
                    robot.FrontLeft.setPower(0);
                    robot.RearRight.setPower(0.1);
                    robot.RearLeft.setPower(0);
                } else if (currentHeading > targetHeading - 1) {
                    robot.FrontRight.setPower(0);
                    robot.FrontLeft.setPower(0.1);
                    robot.RearRight.setPower(0);
                    robot.RearLeft.setPower(0.1);
                }
            }
            robot.FrontRight.setPower(0);
            robot.FrontLeft.setPower(0);
            robot.RearRight.setPower(0);
            robot.RearLeft.setPower(0);
        }

        if(gamepad1.left_stick_button){
            autoHeading = 0;
        }
    }

    private void checkOrientation() {
        // read the orientation of the robot
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.imu.getPosition();
        // and save the heading
        currentHeading = angles.firstAngle + autoHeading;
    }
}