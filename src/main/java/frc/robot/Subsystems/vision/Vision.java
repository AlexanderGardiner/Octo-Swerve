package frc.robot.Subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.vision.Constants.TagCameraVisionConstants;

public class Vision extends SubsystemBase {
	private static Vision INSTANCE;

	public static Vision getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new Vision();
		}
		return INSTANCE;
	}

	// private final PhotonCamera tapeCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
	private final PhotonCamera tapeCamera = new PhotonCamera("tapeCam");

	private final PhotonCamera tagCameraFR = new PhotonCamera(TagCameraVisionConstants.cameraNameFR);
	private final PhotonCamera tagCameraFL = new PhotonCamera(TagCameraVisionConstants.cameraNameFL);
	private final PhotonCamera tagCameraBR = new PhotonCamera(TagCameraVisionConstants.cameraNameBR);
	private final PhotonCamera tagCameraBL = new PhotonCamera(TagCameraVisionConstants.cameraNameBL);
	// public boolean hasTarget;
	private PhotonPipelineResult tapeResult;
	private PhotonPipelineResult tagResultFR;
	private PhotonPipelineResult tagResultFL;
	private PhotonPipelineResult tagResultBR;
	private PhotonPipelineResult tagResultBL;

	@Override
	public void periodic() {
		// Query the latest result from PhotonVision
		if (tapeResult.hasTargets()) {
			this.tapeResult = tapeCamera.getLatestResult();
		}

		if (tagResultFR.hasTargets()) {
			this.tagResultFR = tagCameraFR.getLatestResult();
		}

		if (tagResultFL.hasTargets()) {
			this.tagResultFL = tagCameraFL.getLatestResult();
		}

		if (tagResultBR.hasTargets()) {
			this.tagResultBR = tagCameraBR.getLatestResult();
		}

		if (tagResultBL.hasTargets()) {
			this.tagResultBL = tagCameraBL.getLatestResult();
		}

	}

	public PhotonTrackedTarget getBestTarget() {
		if (tapeResult.hasTargets()) {
			return tapeResult.getBestTarget(); // Returns the best (closest) target
		} else {
			return null; // Otherwise, returns null if no targets are currently found
		}
	}

	public boolean getTapeCamHasTarget() {
		if (tapeResult == null) {
			return false;
		}
		return tapeResult.hasTargets();

		// Returns whether a target was found
	}

}
