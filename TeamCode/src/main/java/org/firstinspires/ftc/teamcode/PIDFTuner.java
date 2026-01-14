package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "PIDFTuner")
public class PIDFTuner extends OpMode {
    private DcMotorEx motor0;

    @Override
    public void init() {
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor0.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 5));
    }

    @Override
    public void loop() {
        motor0.setVelocity(3000);
    }
}
