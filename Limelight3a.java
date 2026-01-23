

package org.firstinspires.ftc.mechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class Limelight3a extends OpMode {
        private Limelight3A limelight;
       @Override
      public void init() {
           limelight = hardwareMap.get(Limelight3A.class, "limelightCam");
           limelight.pipelineSwitch(0);
       }

       @Override
       public void start() {
          limelight.start();
       }
    @Override
       public void loop() {
          LLResult llResult = limelight.getLatestResult();
          if (llResult != null && llResult.isValid()) {
              Pose3D botPose = llResult.getBotpose();
              telemetry.addData("Tx", llResult.getTx());
              telemetry.addData("Tu", llResult.getTy());
              telemetry.addData("Ta", llResult.getTa());
          }
       }
}

