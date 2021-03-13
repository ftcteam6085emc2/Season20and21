package org.firstinspires.ftc.Season20and21.code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HWMapAgain {
    public DcMotor FrontLeft = null;
    public DcMotor FrontRight  = null;
    public DcMotor RearLeft  = null;
    public DcMotor RearRight  = null;
    public DcMotor Collector  = null;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        FrontLeft   = hwMap.dcMotor.get("frontLeft");
        FrontRight  = hwMap.dcMotor.get("frontRight");
        RearLeft = hwMap.dcMotor.get("rearLeft");
        RearRight = hwMap.dcMotor.get("rearRight");
        Collector = hwMap.dcMotor.get("collector");

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
    }
}
