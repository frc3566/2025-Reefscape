package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private PhotonCamera apriltagCamera;
    private PhotonPoseEstimator poseEstimator;
    private Pose2d targetPose;

    public VisionSystemSim visionSim;
    public PhotonCameraSim cameraSim;

    public Vision() {
        apriltagCamera = new PhotonCamera(Constants.Vision.aprilTagCameraName);
        // poseEstimator = new PhotonPoseEstimator(
        //     Constants.Vision.aprilTagFieldLayout,
        //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //     apriltagCamera,
        //     Constants.Vision.robotToCamera
        // );


        initSimulation();
        

        // refreshTargetFiducialIds();
        // System.out.println("Targetting fiducial ids: " + targetFiducialIds);
    }

    /** 
     * Can be accessed at http://localhost:1182 by default.
     */
    public void initSimulation() {
        if (!Robot.isSimulation()) { return; } 

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        visionSim = new VisionSystemSim("Vision");
        visionSim.addAprilTags(fieldLayout);

        cameraSim = new PhotonCameraSim(apriltagCamera, Constants.Vision.cameraProperties);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, Constants.Vision.robotToCamera);

        System.out.println("Camera sim: " + cameraSim);

        SmartDashboard.putData("Field", visionSim.getDebugField());
    }

    public void updateSimulation(Swerve swerve) {
        if (!Robot.isSimulation()) { return; } 

        visionSim.update(swerve.getPose());
    }
}
