
        package org.firstinspires.ftc.teamcode.Season20and21.code;

        import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

        import java.util.ArrayList;
        import java.util.List;

        import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
        import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * <p>
 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name = "VuforiaGarbage", group = "Concept")
public class AutoTesting extends LinearOpMode {

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static int tZone = 0;
    private static boolean init = true;
    private static int strafeCount = 0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "Ae0iKOD/////AAABmQYRSK8WbEkqlgc+d+ayLmZPGuJ4aI3gzdFZk1fEhXtOCPaQyf7BfzPeeI8wBtOlLwOz19W4RXfr5baCFk8BAhgledtrQwXl6dCEeOeH36tSQTNWDbQ+TYR4LIsROCbXeOD30n59xFS8tXupog/EjfWt9qmGT8t8XIwyTJ3h/X7edsCFvnY4xeBVvbqTqy9zpP87QIuoYmePsp+ce8/07KtTyTaZTp3HZzQY2MIAnu9wChFZphiToFtfsl+QElnKnaelqkS+hc5bT+f+ofWlqO9rVvrRpgHLiXuxsNP+kLAogh8YLoaVNivNRxxF86UEyWq+NS8WGkjywCLibBzlr1yWGEoalX3v+lEcYWu18YlA";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private boolean scanned = false;
    private float phoneXRotate = 180;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private int turnCount = 0;
    private boolean yea = false;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    HWMap robot = new HWMap();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        turnCount = 0;
        init = true;
        targetVisible = false;
        tZone = 0;
        strafeCount = 0;
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        targetsSkyStone.deactivate();
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 9f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 6.75f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();

        while (!isStopRequested()) {
            if (scanned) {
                switch (tZone){
                    case 1:
                        DriveStraightDistance(-3600, 0.8);
                        break;
                    case 2:
                        DriveStraightDistance(500, 0.8);
                        Strafe(-300, -0.6);
                        DriveStraightDistance(-100, 0.8);
                        Strafe (300, 0.6);
                        DriveStraightDistance(-5300, 0.8);
                        break;
                    case 3:
                        DriveStraightDistance(2400, 0.8);
                        DriveStraightDistance(-6000, 0.8);
                }
                    /*targetVisible = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }
                    yea = false;
                    why = 0;
                    VectorF translation = lastLocation.getTranslation();
                    telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                    int turnAmount = (int)(rotation.thirdAngle*(50/3));
                    if (rotation.thirdAngle > 5){
                        Turn(turnAmount+100, 0.8); //turn right
                        Strafe(100, 0.8); //strafe left
                    }
                    else if (rotation.thirdAngle < -5){
                        Turn(turnAmount-50, -0.8); //turn left
                        Strafe(-100, -0.8); //strafe right
                    }
                    else if (translation.get(1) / mmPerInch <= -3) {
                        robot.FrontRight.setPower(0.3);
                        robot.FrontLeft.setPower(0.3);
                        robot.RearRight.setPower(-0.3);
                        robot.RearLeft.setPower(-0.3);
                    } else if (translation.get(1) / mmPerInch >= 3) {
                        robot.FrontRight.setPower(-0.3);
                        robot.FrontLeft.setPower(-0.3);
                        robot.RearRight.setPower(0.3);
                        robot.RearLeft.setPower(0.3);
                    } else if (translation.get(0) / mmPerInch <= -1) {
                        robot.FrontRight.setPower(-0.3);
                        robot.FrontLeft.setPower(0.3);
                        robot.RearRight.setPower(-0.3);
                        robot.RearLeft.setPower(0.3);
                        why = 1;
                    } else if (translation.get(0) / mmPerInch > -1) {
                        why = 1;
                        targetVisible = false;
                    }
                }*/
                /*if (why == 1) {
                    DriveStraight(0.3);
                    sleep(50);
                    StopDriving();
                    while (robot.touchSensor.isPressed() == false) {
                        robot.FrontLeft.setPower(0.5);
                        robot.RearLeft.setPower(0.5);
                        robot.SpinRight.setPower(1.0);
                        robot.SpinLeft.setPower(-1.0);
                        turnCount++;
                        sleep(10);
                    }
                    robot.RaveShadowLegends.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                    robot.FrontLeft.setPower(0);
                    robot.RearLeft.setPower(0);
                    robot.SpinRight.setPower(0);
                    robot.SpinLeft.setPower(0);
                    runToPosition();
                    if (robot.touchSensor.isPressed()) {
                        robot.GrabRight.setPosition(-0.1);
                        robot.GrabLeft.setPosition(-0.1);
                        Strafe(-2500, -0.5);
                        Turn(3000, 0.8);
                        if(strafeCount == 2 || strafeCount == 3){
                            DriveStraightDistance(4000, 0.8);
                        }
                        else if (strafeCount >= 4 && strafeCount <= 6){
                            DriveStraightDistance(5000, 0.8);
                        }
                        else if (strafeCount < 2){
                            DriveStraightDistance(3500, 0.8);
                        }
                        else{
                            DriveStraightDistance(6000, 0.8);
                        }
                        robot.GrabRight.setPosition(0.1);
                        robot.GrabLeft.setPosition(0.2);
                        robot.SpinRight.setPower(-1.0);
                        robot.SpinLeft.setPower(1.0);
                        DriveStraightDistance(-1000, -0.8);
                        if(!robot.touchSensor.isPressed()){
                            robot.RaveShadowLegends.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                        robot.SpinRight.setPower(0);
                        robot.SpinLeft.setPower(0);



                        if(strafeCount == 2 || strafeCount == 3){
                            DriveStraightDistance(-9000, -0.8);
                        }
                        else if (strafeCount >= 4 && strafeCount <= 6){
                            DriveStraightDistance(-9000, -0.8);
                        }
                        else if (strafeCount < 2){
                            DriveStraightDistance(-6000, -0.8);
                        }
                        else{
                            DriveStraightDistance(-3000, -0.8);
                        }
                        Turn(1500, 0.8);
                        runUsingEncoder();
                        DriveStraight(-0.8);
                        sleep(1500);
                        StopDriving();
                        runToPosition();
                        DriveStraightDistance(3000, 0.5);
                        runUsingEncoder();
                        if(strafeCount >= 4 && strafeCount <= 6){
                            while (!robot.touchSensor.isPressed()) {
                                robot.FrontRight.setPower(-0.5);
                                robot.RearRight.setPower(-0.5);
                                robot.SpinRight.setPower(1.0);
                                robot.SpinLeft.setPower(-1.0);
                            }
                            robot.RaveShadowLegends.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                            robot.SpinRight.setPower(0);
                            robot.SpinLeft.setPower(0);
                        }
                        else {
                            while (!robot.touchSensor.isPressed()) {
                                robot.FrontLeft.setPower(0.5);
                                robot.RearLeft.setPower(0.5);
                                robot.SpinRight.setPower(1.0);
                                robot.SpinLeft.setPower(-1.0);
                            }
                            robot.RaveShadowLegends.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                            robot.SpinRight.setPower(0);
                            robot.SpinLeft.setPower(0);
                        }
                        runToPosition();
                        if(strafeCount >= 4 && strafeCount <= 6){
                            Turn(-200, -0.8);
                            Strafe(4000, 0.5);
                            DriveStraightDistance(5000, 0.8);
                        }
                        else {
                            Turn(200, 0.8);
                            Strafe(-4000, -0.5);
                            DriveStraightDistance(-8000, -0.8);
                            Turn(3000, 0.8);
                        }
                        robot.SpinRight.setPower(-1.0);
                        robot.SpinLeft.setPower(1.0);
                        DriveStraightDistance(-2000, -0.8);
                        if(!robot.touchSensor.isPressed()){
                            robot.RaveShadowLegends.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                        robot.SpinRight.setPower(0);
                        robot.SpinLeft.setPower(0);
                        break;
                    }
                }*/
            } else {
                telemetry.addData("Visible Target", "none");
                if (!init) {
                    while(!targetVisible){
                        SpecialStrafe(-700, -0.6);
                        strafeCount++;
                    }
                    scanned = true;
                    StopDriving();
                }
                if (!yea) {
                    StopDriving();
                }
            }
            if (init == true) {
                robot.FrontLeft.setPower(0);
                robot.RearLeft.setPower(0);
                //robot.SpinRight.setPower(0);
                //robot.SpinLeft.setPower(0);
                runToPosition();
                init = false;
                waitForStart();
                DriveStraightDistance(1200, 0.8);
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }

    private void runToPosition(){
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

    private void runUsingEncoder(){
        robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void DriveStraight(double power) {
        /*if(strafeCancel){
            robot.FrontRight.setPower(power - 0.2);
            robot.FrontLeft.setPower(-power);
            robot.RearRight.setPower(power);
            robot.RearLeft.setPower(-power - 0.2);
        }
        else {*/
        robot.FrontRight.setPower(-power);
        robot.FrontLeft.setPower(power);
        robot.RearRight.setPower(-power);
        robot.RearLeft.setPower(power);
        //}
    }

    private void StopDriving() {DriveStraight(0);}

    private void DriveStraightDistance(int distance, double power) {
        telemetry.addData("Driving", "Yes");

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
        }

        StopDriving();
    }

    private void Turn(int distance, double power) {
        telemetry.addData("Driving", "Yes");
        robot.FrontRight.setTargetPosition(distance);
        robot.FrontLeft.setTargetPosition(distance);
        robot.RearRight.setTargetPosition(distance);
        robot.RearLeft.setTargetPosition(distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()) {
            idle();
        }

        StopDriving();
    }

    private void SpecialTurn(int distance, double power) {
        telemetry.addData("Driving", "Yes");
        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() + distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() + distance);

        DriveStraight(power);
        while ((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive() && !targetVisible) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    robot.FrontRight.setPower(0);
                    robot.FrontLeft.setPower(0);
                    robot.RearRight.setPower(0);
                    robot.RearLeft.setPower(0);

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
        }
        StopDriving();
    }

    private void Strafe (int distance, double power){
        telemetry.addData("Driving", "Yes");
        robot.FrontRight.setPower(power);
        robot.FrontLeft.setPower(power);
        robot.RearRight.setPower(-power);
        robot.RearLeft.setPower(-power);

        while((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive()){
            idle();
        }

        StopDriving();
    }

    private void SpecialStrafe (int distance, double power){
        telemetry.addData("Driving", "Yes");
        robot.FrontRight.setTargetPosition(robot.FrontRight.getCurrentPosition() + distance);
        robot.FrontLeft.setTargetPosition(robot.FrontLeft.getCurrentPosition() + distance);
        robot.RearRight.setTargetPosition(robot.RearRight.getCurrentPosition() - distance);
        robot.RearLeft.setTargetPosition(robot.RearLeft.getCurrentPosition() - distance);

        robot.FrontRight.setPower(power);
        robot.FrontLeft.setPower(power);
        robot.RearRight.setPower(-power);
        robot.RearLeft.setPower(-power);

        while((robot.FrontRight.isBusy() && robot.RearLeft.isBusy() && robot.RearRight.isBusy() && robot.FrontLeft.isBusy()) && opModeIsActive() && !targetVisible){
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    robot.FrontRight.setPower(0);
                    robot.FrontLeft.setPower(0);
                    robot.RearRight.setPower(0);
                    robot.RearLeft.setPower(0);

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
        }
    }
}