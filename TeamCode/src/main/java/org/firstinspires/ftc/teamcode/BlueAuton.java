/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "blue auton")
public class BlueAuton extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Servo leftBase;
    private Servo rightBase;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    VuforiaLocalizer vuforia;
    VuforiaTrackables trackables;
    VuforiaTrackable template;
    VuforiaTrackableDefaultListener mark;

    @Override
    public void runOpMode() {

//        leftBase = hardwareMap.get(Servo.class, "leftBase");
//        rightBase = hardwareMap.get(Servo.class, "rightBase");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        // TODO: 11/16/19 figure out range
//        leftBase.scaleRange();

//        rightBase.setDirection(Servo.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        runtime.reset();

        cartesianDrive(0, 0.4, 0);

        boolean done = false;
        while (!done && opModeIsActive()) {
            if (mark.isVisible()) {
                OpenGLMatrix pose = mark.getPose();

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                    telemetry.addData("rX", rX);
                    telemetry.addData("rY", rY);
                    telemetry.addData("rZ", rZ);

                    if (Math.abs(rY) < 1) {
                        done = true;
                    }
                }

                telemetry.update();
            }
        }


        forward(10, 0.3);

//        sleep(2000);
//        sideways(10, 0.3);
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "AeTSKkn/////AAABmXK540vJ4k8muhR6D7aoeZpbnFSenqf9a+poNXj4KY56UyTsbTrSeHqrNBi7hJweC+rEjfGiSPfJ813Az57QwdTyLzth/JgNKh3BfGz7OcgIaqCMLwDZf+BAEjFYuX2j5bKUNN/+kCrWp8AUvbPpcPHmGnnwJ7ABzmsazba+tMgSs3rcA3AvezaOGOMDwIiG71ouwN3mKOvybsDNf+2jzMCn1tywUqn3teDCGzKjV2ZeqJW9Qt2wjrvY2sI3MS176buUh/H04Da5FDj+6Dg3/fZtsIlsrVu2fAvepwWvgiCprGf6i9Q4oLBryNgCLDfpMx9EXBWAE4D5hzyBsoFITU0z6zUO+e8b6rJ0xQQr7dbV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("Skystone");
        template = trackables.get(0);

        mark = (VuforiaTrackableDefaultListener) template.getListener();
        trackables.activate();
    }

    private boolean markVisible() {
        return mark.isVisible();
    }

    private double markError() {
        OpenGLMatrix pose = mark.getPose();

        VectorF trans = pose.getTranslation();
        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double tX = trans.get(0);
        double tY = trans.get(1);
        double tZ = trans.get(2);

        double rX = rot.firstAngle;
        double rY = rot.secondAngle;
        double rZ = rot.thirdAngle;

        return tX;
    }

    private void baseServos(double pos) {
        leftBase.setPosition(pos);
        rightBase.setPosition(pos);
    }

    private boolean atTarget() {
        return !frontLeft.isBusy() && !frontRight.isBusy();
    }

    public void sideways(double inches, double power) {
        inches *= Math.sqrt(2);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + inchesToTicks(inches));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - inchesToTicks(inches));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - inchesToTicks(inches));
        backRight.setTargetPosition(backRight.getCurrentPosition() + inchesToTicks(inches));

        runToPosition(power);
    }

    public void forward(double inches, double power) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + inchesToTicks(inches));
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + inchesToTicks(inches));
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + inchesToTicks(inches));
        backRight.setTargetPosition(backRight.getCurrentPosition() + inchesToTicks(inches));

        runToPosition(power);
    }

    public void runToPosition(double power) {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (!atTarget() && opModeIsActive()) {
            idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private int inchesToTicks(double inches) {
        return (int) (inches / (4 * Math.PI) * 1440);
    }

    public void cartesianDrive(double forward, double sideways, double rotate) {
        double FLPower = Range.clip(forward + sideways + rotate, -1, 1);
        double FRPower = Range.clip(forward - sideways - rotate, -1, 1);
        double BLPower = Range.clip(forward - sideways + rotate, -1, 1);
        double BRPower = Range.clip(forward + sideways - rotate, -1, 1);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(FLPower);
        frontRight.setPower(FRPower);
        backLeft.setPower(BLPower);
        backRight.setPower(BRPower);
    }

}
