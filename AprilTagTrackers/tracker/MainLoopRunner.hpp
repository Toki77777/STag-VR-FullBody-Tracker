#pragma once

#include "GUI.hpp"
#include "OpenVRClient.hpp"
#include "PlayspaceCalib.hpp"
#include "ReferenceMarkerCalib.hpp"
#include "RefPtr.hpp"
#include "StagWrapper.hpp"
#include "TrackerUnit.hpp"
#include "utils/Log.hpp"
#include "VideoCapture.hpp"
#include "VRDriver.hpp"

#include <opencv2/objdetect/aruco_detector.hpp>

#include <array>
#include <optional>

namespace tracker
{

inline std::optional<RodrPose> EstimateReferenceMarkerPose(
    const MarkerDetectionList& detections,
    const ArucoBoardSharedPtr& board,
    const cfg::CameraCalib& camera)
{
    auto [pose, numEstimated] = math::EstimatePoseTracker(
        detections.corners, detections.ids, board, camera);
    if (numEstimated <= 0) return std::nullopt;
    return pose;
}

class MainLoopRunner
{
    static inline const cv::Scalar COLOR_MASK{255, 0, 0}; /// red
    static inline const cv::Scalar COLOR_REFERENCE{0, 255, 255}; /// yellow

public:
    explicit MainLoopRunner(RefPtr<UserConfig> config,
                            RefPtr<CalibrationConfig> calibConfig,
                            RefPtr<PlayspaceCalib> playspace,
                            RefPtr<VRDriver> vrDriver,
                            OptRefPtr<const TrackerUnit> referenceMarker = {})
        : mConfig(config),
          mCalibConfig(calibConfig),
          camCalib(calibConfig->cameras[0]),
          videoStream(mConfig->videoStreams[0]),
          stagDetector(StagWrapper::ConvertLibrary(mConfig->markerLibrary), videoStream->quadDecimate),
          trackerNum(mConfig->trackerNum),
          mPlayspace(playspace),
          mVRDriver(vrDriver),
          mReferenceMarker(referenceMarker)
    {
        mPlayspace->Set(mConfig->manualCalib.GetAsReal());
        // calculate position of camera from calibration data and send its position to steamvr
        mVRDriver->UpdateStation(mPlayspace->GetStationPoseOVR());
        if (mReferenceMarker) InitReferenceCalibrator();
    }

    std::optional<RodrPose> GetReferenceMarkerPose() const
    {
        return mReferenceMarkerPose;
    }

    void Update(RefPtr<AwaitedFrame> cameraFrame,
                RefPtr<GUI> gui,
                RefPtr<std::vector<TrackerUnit>> trackerUnits,
                RefPtr<IVRClient> vrClient,
                RefPtr<const ITrackerControl> trackerCtrl)
    {
        cameraFrame->Get(frame);
        // shallow copy, gray will be cloned from image and used for detection,
        // so drawing can happen on color image without clone.
        drawImg = frame.image;
        StagWrapper::ConvertGrayscale(frame.image, grayImg);
        const bool previewIsVisible = gui->IsPreviewVisible();

        // The board is on the HMD, so its pose only means anything paired with the HMD pose
        // from the same frame. Read it before detection, so that the board can also be
        // predicted for the search mask below.
        mHmdPose.reset();
        if (mReferenceMarker && vrClient->IsInit()) mHmdPose = vrClient->GetHMDPose();

        const auto stampBeforeDetect = utils::SteadyTimer::Now();
        detectionTimer.Restart(stampBeforeDetect);

        bool circularWindow = videoStream->circularWindow;

        // if any tracker was lost for longer than 20 frames, mark circularWindow as false
        for (const auto& unit : *trackerUnits)
        {
            if (!unit.WasVisibleLastFrame())
            {
                ++framesSinceLastSeen;
                if (framesSinceLastSeen > framesToCheckAll) circularWindow = false;
                break;
            }
        }
        if (!circularWindow) framesSinceLastSeen = 0;

        // define our mask image. We want to create an image where everything but circles around predicted tracker positions will be black to speed up detection.
        if (GetMatSize(maskSearchImg) != GetMatSize(grayImg))
        {
            maskSearchImg.create(GetMatSize(grayImg), CV_8U);
        }
        maskSearchImg = cv::Scalar(0); // fill with empty pixels
        const int searchRadius = static_cast<int>(static_cast<double>(grayImg.rows) * videoStream->searchWindow);
        bool atleastOneTrackerVisible = false;

        const double frameTimeBeforeDetect = duration_cast<utils::FSeconds>(stampBeforeDetect - frame.timestamp).count();
        for (int i = 0; i < trackerNum; i++)
        {
            auto& unit = (*trackerUnits)[i];
            auto [pose, isValid] = mVRDriver->GetTracker(i, -frameTimeBeforeDetect - videoStream->latency);

            if (isValid)
                pose = mPlayspace->InvTransformFromOVR(pose);

            std::array<cv::Point2d, 2> projected;
            {
                const cv::Vec3d unusedRVec{}; // used to perform change of basis
                const cv::Vec3d unusedTVec{};
                const std::array<cv::Point3d, 2> points{pose.position, unit.GetEstimatedPose().position};
                cv::projectPoints(points, unusedRVec, unusedTVec, camCalib->cameraMatrix, camCalib->distortionCoeffs, projected);
            }
            const auto& [driverCenter, previousCenter] = projected;

            // project point from position of tracker in camera 3d space to 2d camera pixel space, and draw a dot there
            if (previewIsVisible) cv::circle(drawImg, driverCenter, 5, cv::Scalar(0, 0, 255), 2, 8, 0);

            unit.SetWasVisibleToDriverLastFrame(isValid);
            cv::Point2d maskCenter;
            if (isValid) // if the pose from steamvr was valid, save the predicted position and rotation
            {
                if (previewIsVisible) cv::drawFrameAxes(drawImg, camCalib->cameraMatrix, camCalib->distortionCoeffs, pose.rotation.toRotVec(), math::ToVec(pose.position), 0.10F);

                if (!unit.WasVisibleLastFrame()) // if tracker was found in previous frame, we use that position for masking. If not, we use position from driver for masking.
                {
                    maskCenter = driverCenter;
                }
                else
                {
                    maskCenter = previousCenter;
                }
                unit.SetWasVisibleLastFrame(true);
                unit.SetPoseFromDriver(RodrPose(pose));
            }
            else
            {
                if (unit.WasVisibleLastFrame())
                {
                    maskCenter = previousCenter; // if pose is not valid, set everything based on previous known position
                }
            }

            if (maskCenter.inside(cv::Rect2d(0, 0, frame.image.cols, frame.image.rows)))
            {
                atleastOneTrackerVisible = true;
                if (circularWindow) // if circular window is set mask a circle around the predicted tracker point
                {
                    cv::circle(maskSearchImg, maskCenter, searchRadius, cv::Scalar(255), -1, 8, 0);
                    if (previewIsVisible) cv::circle(drawImg, maskCenter, searchRadius, COLOR_MASK, 2, 8, 0);
                }
                else // if not, mask a vertical strip top to bottom. This happens every 20 frames if a tracker is lost.
                {
                    const int maskX = static_cast<int>(maskCenter.x);
                    const cv::Rect2i maskRect{cv::Point(maskX - searchRadius, 0), cv::Point2i(maskX + searchRadius, frame.image.rows)};
                    cv::rectangle(maskSearchImg, maskRect, cv::Scalar(255), -1);
                    if (previewIsVisible) cv::rectangle(drawImg, maskRect, COLOR_MASK, 3);
                }
            }
            else
            {
                unit.SetWasVisibleLastFrame(false); // if detected tracker is out of view of the camera, we mark it as not found, as either the prediction is wrong or we wont see it anyway
            }
        }

        // A reference board is not a tracker, so nothing above says where it is. Once the
        // camera pose is known it follows from the HMD pose, and the board gets a window in
        // the mask like everything else. Until then, or after the board has been missing long
        // enough for the prediction to be doubtful, the whole frame has to be searched.
        const bool referenceSearchIsMasked =
            !mReferenceMarker || TryMaskReferenceMarker(previewIsVisible, searchRadius);

        // using copyTo with masking creates the image where everything but the locations where trackers are predicted to be is black
        if (atleastOneTrackerVisible && referenceSearchIsMasked)
        {
            grayImg.copyTo(tempGrayMaskedImg, maskSearchImg);
            grayImg = tempGrayMaskedImg;
        }

        mCalibrator.Update(vrClient, mVRDriver, gui, mPlayspace, trackerCtrl->lockHeightCalib, trackerCtrl->manualRecalibrate);

        stagDetector.DetectMarkers(grayImg, dets);
        if (mReferenceMarker)
        {
            mReferenceMarkerPose = EstimateReferenceMarkerPose(
                dets, mReferenceMarker->GetArucoBoard(), *camCalib);
            if (mReferenceMarkerPose)
                mReferenceMissingFrames = 0;
            else
                ++mReferenceMissingFrames;
            UpdateReferenceCalibration(gui, trackerCtrl);
        }
        else
        {
            mReferenceMarkerPose.reset();
        }
        // frame time is how much time passed since frame was acquired.
        const double frameTimeAfterDetect = duration_cast<utils::FSeconds>(utils::SteadyTimer::Now() - frame.timestamp).count();
        for (int index = 0; index < trackerUnits->size(); ++index)
        {
            auto& unit = (*trackerUnits)[index];
            // estimate the pose of current board
            const RodrPose scaledPoseFromDriver{unit.GetPoseFromDriver().position / mPlayspace->GetScale(), unit.GetPoseFromDriver().rotation};
            // on rare occasions, detection crashes. Should be very rare and indicate something wrong with camera or tracker calibration
            auto [estimatedPose, numEstimated] = math::EstimatePoseTracker(
                dets.corners, dets.ids, unit.GetArucoBoard(), *camCalib,
                unit.WasVisibleLastFrame() && mConfig->usePredictive,
                scaledPoseFromDriver);
            estimatedPose.position *= mPlayspace->GetScale(); // unscale returned estimation;
            unit.SetEstimatedPose(estimatedPose);

            ATT_ASSERT(!std::isnan(estimatedPose.position[X]));

            if (numEstimated <= 0)
            {
                unit.SetWasVisibleLastFrame(false);
                continue;
            }
            unit.SetWasVisibleLastFrame(true);

            if (mConfig->depthSmoothing > 0 && unit.WasVisibleToDriverLastFrame() && !trackerCtrl->manualRecalibrate)
            {
                // depth estimation is noisy, so try to smooth it more, especialy if using multiple cameras
                // if position is close to the position predicted by the driver, take the depth of the driver.
                // if error is big, take the calculated depth
                // error threshold is defined in the params as depth smoothing
                RodrPose pose = unit.GetEstimatedPose();

                const double distDriver = Length(unit.GetPoseFromDriver().position);
                const double distPredict = Length(pose.position);

                const cv::Vec3d normPredict = pose.position / distPredict;

                double dist = std::abs(distPredict - distDriver);
                dist = (dist / static_cast<double>(mConfig->depthSmoothing)) + 0.1;
                dist = std::clamp(dist, 0.0, 1.0);

                const double distFinal = (dist * distPredict) + (1 - dist) * distDriver;

                pose.position = normPredict * distFinal;
                unit.SetEstimatedPose(pose);
            }

            {
                const cv::Point3d position = unit.GetEstimatedPose().position;

                // Reject detected positions that are behind the camera
                if (position.z < 0)
                {
                    unit.SetWasVisibleLastFrame(false);
                    continue;
                }

                // Figure out the camera aspect ratio, XZ and YZ ratio limits
                const double aspectRatio = GetMatSize(frame.image).aspectRatio();
                const double xzRatioLimit = 0.5 * static_cast<double>(frame.image.cols) / camCalib->cameraMatrix.at<double>(0, 0);
                const double yzRatioLimit = 0.5 * static_cast<double>(frame.image.rows) / camCalib->cameraMatrix.at<double>(1, 1);

                // Figure out whether X or Y dimension is most likely to go outside the camera field of view
                if (std::abs(position.x / position.y) > aspectRatio)
                {
                    // Reject detections when XZ coordinate ratio goes out of camera FOV
                    if (std::abs(position.x / position.z) > xzRatioLimit)
                    {
                        unit.SetWasVisibleLastFrame(false);
                        continue;
                    }
                }
                else
                {
                    // Reject detections when YZ coordinate ratio goes out of camera FOV
                    if (std::abs(position.y / position.z) > yzRatioLimit)
                    {
                        unit.SetWasVisibleLastFrame(false);
                        continue;
                    }
                }
            }

            if (trackerCtrl->multicamAutocalib && unit.WasVisibleToDriverLastFrame())
            {
                tracker::PlayspaceCalibrator::UpdateMulticam(gui, mPlayspace, unit);
                continue; // skip sending to driver
            }

            // transform boards position based on our calibration data
            Pose poseToSend = mPlayspace->TransformToOVR(Pose(unit.GetEstimatedPose()));

            // send all the values
            mVRDriver->UpdateTracker(index, poseToSend, -frameTimeAfterDetect - videoStream->latency, mConfig->smoothingFactor);
        }

        if (gui->IsPreviewVisible())
        {
            // Draw the detected board next to where the HMD pose says it should be. Seeing
            // the two apart is what tells a systematic calibration error from detection noise.
            if (mReferenceMarkerPose)
            {
                cv::drawFrameAxes(drawImg, camCalib->cameraMatrix, camCalib->distortionCoeffs,
                                  mReferenceMarkerPose->rotation.value, mReferenceMarkerPose->position, 0.15F);
            }
            if (mReferencePredictedCenter)
            {
                cv::circle(drawImg, *mReferencePredictedCenter, 6, COLOR_REFERENCE, -1, 8, 0);
            }
            // draw and display the detections
            if (!dets.ids.empty()) cv::aruco::drawDetectedMarkers(drawImg, dets.corners, dets.ids);
            const cv::Size2i drawSize = ConstrainSize(GetMatSize(frame.image), mConfig->previewImageSize);
            cv::resize(drawImg, outImg, drawSize);
            cv::putText(outImg, std::to_string(frameTimeAfterDetect).substr(0, 5), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
            gui->UpdatePreview(outImg);
        }
        // time of marker detection
    }

private:
    static const char* ReferenceStateName(ReferenceMarkerCalibrator::State state)
    {
        switch (state)
        {
        case ReferenceMarkerCalibrator::State::WaitingForData: return "waiting for the board and the HMD";
        case ReferenceMarkerCalibrator::State::CollectingSamples: return "measuring the HMD to board offset";
        case ReferenceMarkerCalibrator::State::Calibrated: return "calibrated";
        }
        return "unknown";
    }

    void InitReferenceCalibrator()
    {
        ReferenceMarkerCalibrator::Options options;
        options.continuous = mConfig->referenceMarker.continuousCalibration;
        mReferenceCalibrator = ReferenceMarkerCalibrator{options};

        const auto& stored = mCalibConfig->referenceMarkerOffset;
        if (!stored.calibrated || mConfig->referenceMarker.recalibrateHmdOffset)
        {
            ATT_LOG_INFO("reference marker: measuring where the board sits on the HMD, "
                         "keep it in view of the camera and turn your head about different axes");
            return;
        }
        mReferenceCalibrator.SetOffset(
            Pose{cv::Point3d(stored.position), cv::Quatd::createFromRvec(stored.rotation)},
            stored.scale);
        ATT_LOG_INFO("reference marker: reusing the stored HMD to board offset, camera scale ", stored.scale);
    }

    /// Give the reference board its window in the search mask.
    /// @return true when the board is accounted for and masked detection will not hide it
    bool TryMaskReferenceMarker(bool previewIsVisible, int searchRadius)
    {
        mReferencePredictedCenter.reset();
        if (mReferenceMissingFrames > framesToCheckAll) return false;
        if (!mHmdPose || !mReferenceCalibrator.IsCalibrated()) return false;

        const auto predicted = PredictMarkerPosInCamera(
            *mReferenceCalibrator.GetCalib(), *mReferenceCalibrator.GetOffset(), *mHmdPose);
        if (!predicted) return false;
        // behind the camera, so there is nothing to find and nothing to leave unmasked
        if ((*predicted)[Z] <= 0) return true;

        std::array<cv::Point2d, 1> projected{};
        {
            const cv::Vec3d unusedRVec{}; // used to perform change of basis
            const cv::Vec3d unusedTVec{};
            const std::array<cv::Point3d, 1> points{cv::Point3d(*predicted)};
            cv::projectPoints(points, unusedRVec, unusedTVec, camCalib->cameraMatrix, camCalib->distortionCoeffs, projected);
        }
        const cv::Point2d center = projected[0];
        if (!center.inside(cv::Rect2d(0, 0, frame.image.cols, frame.image.rows))) return true;

        mReferencePredictedCenter = center;
        cv::circle(maskSearchImg, center, searchRadius, cv::Scalar(255), -1, 8, 0);
        if (previewIsVisible) cv::circle(drawImg, center, searchRadius, COLOR_REFERENCE, 2, 8, 0);
        return true;
    }

    void UpdateReferenceCalibration(RefPtr<GUI> gui, RefPtr<const ITrackerControl> trackerCtrl)
    {
        // An explicit calibration request wins. The reference board is the source of truth
        // otherwise, so manual changes are not blended in: the automatic solve takes over
        // again once the user closes the manual calibration.
        if (trackerCtrl->manualRecalibrate || trackerCtrl->multicamAutocalib) return;

        std::optional<Pose> markerPose;
        if (mReferenceMarkerPose) markerPose = Pose(*mReferenceMarkerPose);
        const auto outcome = mReferenceCalibrator.Update(mHmdPose, markerPose);

        if (outcome.offsetSolved) SaveReferenceOffset();
        if (outcome.calib)
        {
            mPlayspace->Set(*outcome.calib);
            mVRDriver->UpdateStation(mPlayspace->GetStationPoseOVR());
            // the gui owns the persisted copy of these numbers, and throttles its own redraw
            gui->SetManualCalib(*outcome.calib);
        }
        LogReferenceDiagnostics(outcome);
    }

    void SaveReferenceOffset()
    {
        const auto& offset = mReferenceCalibrator.GetOffset();
        if (!offset) return;
        auto& stored = mCalibConfig->referenceMarkerOffset;
        stored.calibrated = true;
        stored.position = cv::Vec3d(offset->position);
        stored.rotation = offset->rotation.toRotVec(cv::QUAT_ASSUME_UNIT);
        stored.scale = mReferenceCalibrator.GetScale();
        // written once per solve, so the next session starts from the first frame the board
        // is seen in rather than asking for the head motion again
        mCalibConfig->Save();
        ATT_LOG_INFO("reference marker offset solved: position ",
                     stored.position[X], " ", stored.position[Y], " ", stored.position[Z],
                     " m, camera scale ", stored.scale);
    }

    void LogReferenceDiagnostics(const ReferenceMarkerCalibrator::UpdateOutcome& outcome)
    {
        if (outcome.state != mReferenceLoggedState)
        {
            mReferenceLoggedState = outcome.state;
            ATT_LOG_INFO("reference marker: ", ReferenceStateName(outcome.state));
        }
        constexpr auto logInterval = utils::Seconds(2);
        if (mReferenceLogTimer.Get() < logInterval) return;
        mReferenceLogTimer.Restart();

        if (!mReferenceCalibrator.IsCalibrated())
        {
            ATT_LOG_INFO("reference marker: ", ReferenceStateName(outcome.state),
                         ", ", outcome.sampleCount, " samples, board missing for ",
                         mReferenceMissingFrames, " frames");
            return;
        }

        const auto& calib = *mReferenceCalibrator.GetCalib();
        ATT_LOG_INFO("reference marker camera pose: position ",
                     calib.posOffset[X], " ", calib.posOffset[Y], " ", calib.posOffset[Z],
                     " m, angles ",
                     calib.angleOffset[X] * RAD_2_DEG, " ", calib.angleOffset[Y] * RAD_2_DEG, " ",
                     calib.angleOffset[Z] * RAD_2_DEG, " deg, scale ", calib.scale);
        if (outcome.positionError && outcome.rotationError)
        {
            // The gap between the HMD pose predicted from the board and the one SteamVR
            // reports. A steady offset means the calibration is off; noise that moves with
            // the board means detection is the limit.
            ATT_LOG_INFO("reference marker residual: ", *outcome.positionError * 100.0, " cm, ",
                         *outcome.rotationError * RAD_2_DEG, " deg",
                         outcome.outlier ? ", solution held back as an outlier" : "");
        }
        if (mReferenceMissingFrames > 0)
        {
            ATT_LOG_INFO("reference marker not detected for ", mReferenceMissingFrames, " frames");
        }
    }

    RefPtr<UserConfig> mConfig;
    RefPtr<CalibrationConfig> mCalibConfig;
    RefPtr<const cfg::CameraCalib> camCalib;
    RefPtr<const cfg::VideoStream> videoStream;
    StagWrapper stagDetector;
    Index trackerNum;
    RefPtr<PlayspaceCalib> mPlayspace;
    RefPtr<VRDriver> mVRDriver;
    OptRefPtr<const TrackerUnit> mReferenceMarker;
    std::optional<RodrPose> mReferenceMarkerPose = std::nullopt;
    /// HMD pose of the current frame, read before detection so it pairs with the board pose
    std::optional<Pose> mHmdPose = std::nullopt;
    ReferenceMarkerCalibrator mReferenceCalibrator{};
    ReferenceMarkerCalibrator::State mReferenceLoggedState = ReferenceMarkerCalibrator::State::WaitingForData;
    int mReferenceMissingFrames = 0;
    std::optional<cv::Point2d> mReferencePredictedCenter = std::nullopt;
    utils::SteadyTimer mReferenceLogTimer{};

    MarkerDetectionList dets{};

    tracker::CapturedFrame frame{};
    cv::Mat drawImg{};
    cv::Mat outImg{};
    cv::Mat grayImg{};
    cv::Mat maskSearchImg{};
    cv::Mat tempGrayMaskedImg{};

    int framesSinceLastSeen = 0;
    static constexpr int framesToCheckAll = 20;
    PlayspaceCalibrator mCalibrator{};

    utils::SteadyTimer detectionTimer{};
};

} // namespace tracker
