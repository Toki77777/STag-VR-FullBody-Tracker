#include "Tracker.hpp"

#include "config/TrackerUnit.hpp"
#include "Helpers.hpp"
#include "ImageDrawing.hpp"
#include "math/CVHelpers.hpp"
#include "StagWrapper.hpp"
#include "tracker/MainLoopRunner.hpp"
#include "tracker/TrackerUnit.hpp"
#include "utils/Assert.hpp"
#include "utils/Error.hpp"
#include "utils/LogBatch.hpp"
#include "utils/SteadyTimer.hpp"
#include "utils/Test.hpp"
#include "utils/Types.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/videoio.hpp>
#include <ps3eye/PSEyeVideoCapture.h>

#include <algorithm>
#include <array>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace
{

struct MarkerIdRange
{
    int begin;
    int end;

    bool Contains(int markerId) const
    {
        return markerId >= begin && markerId < end;
    }

    bool Overlaps(const MarkerIdRange& other) const
    {
        return begin < other.end && other.begin < end;
    }
};

class TrackerMarkerIdPartition
{
public:
    /// @param markerCount markers the selected STag library has, see StagWrapper::MarkerCount
    TrackerMarkerIdPartition(int trackerCount, int markersPerTracker, int markerCount,
                             const cfg::List<cfg::TrackerUnit>& trackerConfigs)
        : mTrackerCount(std::max(0, trackerCount)), mMarkersPerTracker(markersPerTracker),
          mMarkerCount(std::max(0, markerCount))
    {
        ResetToDefaults();
        if (!TryApplyConfiguredRanges(trackerConfigs))
        {
            ATT_LOG_ERROR("Invalid or overlapping tracker marker ID ranges; using markersPerTracker defaults.");
            ResetToDefaults();
        }
        LogRanges();
    }

    int MainMarkerId(int trackerIndex) const
    {
        return GetRange(trackerIndex).begin;
    }

    bool Contains(int trackerIndex, int markerId) const
    {
        return GetRange(trackerIndex).Contains(markerId);
    }

    bool IsMainMarker(int trackerIndex, int markerId) const
    {
        return markerId == MainMarkerId(trackerIndex);
    }

    bool Overlaps(const MarkerIdRange& range) const
    {
        return std::ranges::any_of(mRanges, [&](const MarkerIdRange& trackerRange)
                                   { return trackerRange.Overlaps(range); });
    }

    void EnsureContainsAll(int trackerIndex, const std::vector<int>& markerIds) const
    {
        for (const int markerId : markerIds)
        {
            if (!Contains(trackerIndex, markerId))
                throw utils::MakeError("marker ID ", markerId, " is outside tracker ", trackerIndex, " configured range");
        }
    }

private:
    /// Which IDs each tracker will look for, and which one has to be seen first. Without this
    /// a marker layout that does not match the config looks exactly like a detection failure.
    void LogRanges() const
    {
        std::string ranges;
        for (std::size_t index = 0; index < mRanges.size(); ++index)
        {
            if (index != 0) ranges += ", ";
            ranges += std::to_string(index) + ": [" + std::to_string(mRanges[index].begin) +
                      ", " + std::to_string(mRanges[index].end) + ") main " +
                      std::to_string(mRanges[index].begin);
        }
        ATT_LOG_INFO("tracker marker IDs - ", ranges);
    }

    const MarkerIdRange& GetRange(int trackerIndex) const
    {
        return mRanges.at(static_cast<std::size_t>(trackerIndex));
    }

    void ResetToDefaults()
    {
        // The library caps how many markers exist at all. Handing out IDs past its end gives
        // a tracker a main marker that cannot be printed or detected, and the calibration
        // then waits forever for a marker that does not exist, so fit the ranges instead.
        int perTracker = mMarkersPerTracker;
        if (mTrackerCount > 0 && mMarkerCount > 0)
        {
            const int fits = mMarkerCount / mTrackerCount;
            if (fits < perTracker)
            {
                ATT_LOG_ERROR("markersPerTracker ", mMarkersPerTracker, " does not fit ", mTrackerCount,
                              " trackers in the selected STag library, which has ", mMarkerCount,
                              " markers; using ", std::max(1, fits), " per tracker instead.");
                perTracker = std::max(1, fits);
            }
        }

        mRanges.clear();
        mRanges.reserve(static_cast<std::size_t>(mTrackerCount));
        for (int trackerIndex = 0; trackerIndex < mTrackerCount; ++trackerIndex)
        {
            mRanges.push_back({trackerIndex * perTracker, (trackerIndex + 1) * perTracker});
        }
    }

    bool TryApplyConfiguredRanges(const cfg::List<cfg::TrackerUnit>& trackerConfigs)
    {
        for (int trackerIndex = 0; trackerIndex < mTrackerCount && trackerIndex < trackerConfigs.GetSize(); ++trackerIndex)
        {
            const auto config = trackerConfigs[trackerIndex];
            const bool useDefault = config->markerIdBegin == -1 && config->markerIdEnd == -1;
            if (useDefault) continue;
            if (config->markerIdBegin < 0 || config->markerIdEnd <= config->markerIdBegin) return false;
            if (mMarkerCount > 0 && config->markerIdEnd > mMarkerCount)
            {
                ATT_LOG_ERROR("Tracker ", trackerIndex, " marker ID range [", config->markerIdBegin, ", ",
                              config->markerIdEnd, ") runs past the ", mMarkerCount,
                              " markers of the selected STag library.");
                return false;
            }
            mRanges[trackerIndex] = {config->markerIdBegin, config->markerIdEnd};
        }

        for (std::size_t lhs = 0; lhs < mRanges.size(); ++lhs)
        {
            for (std::size_t rhs = lhs + 1; rhs < mRanges.size(); ++rhs)
            {
                if (mRanges[lhs].Overlaps(mRanges[rhs])) return false;
            }
        }
        return true;
    }

    int mTrackerCount;
    int mMarkersPerTracker;
    int mMarkerCount;
    std::vector<MarkerIdRange> mRanges;
};

/// @param markerCount markers the selected STag library has, see StagWrapper::MarkerCount
std::optional<MarkerIdRange> ResolveReferenceMarkerRange(
    const cfg::ReferenceMarker& config,
    const TrackerMarkerIdPartition& trackerRanges,
    int markerCount)
{
    if (!config.enabled) return std::nullopt;

    const MarkerIdRange range{config.markerIdBegin, config.markerIdEnd};
    if (markerCount > 0 && range.end > markerCount)
    {
        // Named separately from the checks below: a range that is fine in itself but past the
        // end of a small library is the mistake that silently detects nothing at all.
        ATT_LOG_ERROR("Reference marker ID range [", range.begin, ", ", range.end,
                      ") runs past the ", markerCount,
                      " markers of the selected STag library; reference marker disabled.");
        return std::nullopt;
    }
    if (range.begin < 0 || range.end <= range.begin || trackerRanges.Overlaps(range))
    {
        ATT_LOG_ERROR("Invalid or overlapping reference marker ID range; reference marker disabled.");
        return std::nullopt;
    }
    return range;
}

} // namespace

Tracker::Tracker(UserConfig& _userConfig, CalibrationConfig& _calibConfig, const Localization& _lc)
    : mCapture(&_userConfig.videoStreams[0]->camera),
      user_config(_userConfig), calib_config(_calibConfig), lc(_lc)
{
    SetTrackerUnitsFromConfig();
    mPlayspace.Set(user_config.manualCalib.GetAsReal());
}

void Tracker::StartCamera(RefPtr<cfg::Camera> cam)
{
    if (cameraRunning)
    {
        cameraRunning = false;
        mainThreadRunning = false;
        cameraThread.join();
        return;
    }

    if (!mCapture.TryOpen())
    {
        gui->ShowPopup(lc.TRACKER_CAMERA_START_ERROR, PopupStyle::Error);
        return;
    }

    // ensure joined before creating new thread
    if (cameraThread.joinable())
    {
        cameraThread.join();
    }

    cameraRunning = true;
    cameraThread = std::thread(&Tracker::CameraLoop, this);
}

void Tracker::StartCamera()
{
    StartCamera(&user_config.videoStreams[0]->camera);
}

void Tracker::CameraLoop()
{
    const RefPtr<cfg::Camera> cam = &user_config.videoStreams[0]->camera;

    tracker::CapturedFrame frame;
    cv::Mat drawImg;

    utils::SteadyTimer fpsTimer{};
    int frameCount = 0;
    int fps = 0;

    utils::SteadyTimer previewTimer{};
    gui->SetStatus(true, StatusItem::Camera);

    while (cameraRunning)
    {
        const auto stampBeforeCap = utils::SteadyTimer::Now();
        const utils::NanoS frameTime = mFrameTimer.Get(stampBeforeCap);
        mFrameTimer.Restart(stampBeforeCap);

        if (!mCapture.TryReadFrame(frame))
        {
            gui->ShowPopup(lc.TRACKER_CAMERA_ERROR, PopupStyle::Error);
            cameraRunning = false;
            break;
        }
        const auto stampAfterCap = frame.timestamp;

        // framerate limiter
        // constexpr int minSafeFps = 300;
        // constexpr auto minSafeCaptureTime = duration_cast<utils::NanoS>(utils::PerSecond(utils::Seconds(minSafeFps)));
        // if (frameTime < minSafeCaptureTime)
        // {
        //     std::this_thread::sleep_for(utils::MilliS(10));
        // }

        // fps counter
        if (fpsTimer.Get(stampAfterCap) < utils::Seconds(1))
        {
            ++frameCount;
        }
        else
        {
            fpsTimer.Restart(stampAfterCap);
            fps = frameCount;
            frameCount = 0;
        }

        // fps = (0.95 * fps) + (0.05 * utils::PerSecond(frameTime).count());
        const utils::NanoS captureTime = mFrameTimer.Get(stampAfterCap);

        // Ensure that preview isnt shown more than 60 times per second.
        // In some cases opencv will return a solid color image without any blocking delay (unlike a camera locked to a framerate),
        // and we would just be drawing fps text on nothing.
        if ((previewTimer.Get(stampAfterCap) * 60) > utils::Seconds(1))
        {
            if (gui->IsPreviewVisible(PreviewId::Camera))
            {
                previewTimer.Restart(stampAfterCap);
                frame.image.copyTo(drawImg);
                cv::putText(drawImg, std::to_string(fps),
                            cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 2);
                const std::string resolution = std::to_string(frame.image.cols) + "x" + std::to_string(frame.image.rows);
                cv::putText(drawImg, resolution, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 2);
                if (previewCameraCalibration) drawCalibration(drawImg, *calib_config.cameras[0]);
                gui->UpdatePreview(drawImg, PreviewId::Camera);
            }
        }
        mCameraFrame.Set(frame);

        if (mVRClient && mVRClient->IsInit())
        {
            mVRClient->PollEvents();
        }
    }
    mCapture.Close();
    gui->SetStatus(false, StatusItem::Camera);
}

void Tracker::StartCameraCalib()
{
    if (mainThreadRunning)
    {
        mainThreadRunning = false;
        mainThread.join();
        return;
    }
    if (!cameraRunning)
    {
        gui->ShowPopup(lc.TRACKER_CAMERA_NOTRUNNING, PopupStyle::Error);
        mainThreadRunning = false;
        return;
    }

    // ensure joined before creating new thread
    if (mainThread.joinable())
    {
        mainThread.join();
    }

    mainThreadRunning = true;
    if (!user_config.chessboardCalib)
    {
        mainThread = std::thread(&Tracker::CalibrateCameraCharuco, this);
    }
    else
    {
        mainThread = std::thread(&Tracker::CalibrateCamera, this);
    }
}

/// function to calibrate our camera
void Tracker::CalibrateCameraCharuco()
{
    tracker::CapturedFrame frame;
    cv::Mat gray;
    cv::Mat drawImg;

    const cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;

    // set our detectors marker border bits to 1 since thats what charuco uses
    params.markerBorderBits = 1;

    // generate and show our charuco board that will be used for calibration
    cv::aruco::CharucoBoard board(cv::Size(8, 7), 0.04f, 0.02f, dictionary);
    // OpenCV 4.7 changed the charuco pattern generation for boards with an even row count,
    // keep detecting the boards printed for previous versions of this app
    board.setLegacyPattern(true);
    cv::Mat boardImage;

    const cv::aruco::ArucoDetector detector(dictionary, params);
    const cv::aruco::CharucoDetector charucoDetector(board);

    // int framesSinceLast = -2 * user_config.camFps;
    auto timeOfLast = std::chrono::steady_clock::now();

    bool promptSaveCalib = false;
    gui->ShowPrompt(lc.TRACKER_CAMERA_CALIBRATION_INSTRUCTIONS,
                    [&](bool pressedOk) {
                        promptSaveCalib = pressedOk;
                        mainThreadRunning = false;
                    });

    cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::Mat1d stdDeviationsIntrinsics, stdDeviationsExtrinsics;
    std::vector<double> perViewErrors;
    std::vector<std::vector<cv::Point2f>> allCharucoCorners;
    std::vector<std::vector<int>> allCharucoIds;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCorners;

    // cv::aruco::calibrateCameraCharuco was removed from the new aruco api,
    // match the collected charuco corners per view and calibrate directly
    const auto calibrateFromCharuco = [&](const cv::Size2i& imageSize) {
        std::vector<std::vector<cv::Point3f>> allObjPoints(allCharucoCorners.size());
        std::vector<std::vector<cv::Point2f>> allImgPoints(allCharucoCorners.size());
        for (std::size_t view = 0; view < allCharucoCorners.size(); ++view)
        {
            board.matchImagePoints(allCharucoCorners[view], allCharucoIds[view],
                                   allObjPoints[view], allImgPoints[view]);
        }
        cv::calibrateCamera(allObjPoints, allImgPoints, imageSize,
                            cameraMatrix, distCoeffs, R, T,
                            stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors,
                            cv::CALIB_USE_LU);
    };

    auto preview = gui->CreatePreviewControl();

    int picsTaken = 0;
    while (mainThreadRunning && cameraRunning)
    {
        mCameraFrame.Get(frame);
        frame.image.copyTo(drawImg);
        cv::putText(drawImg, std::to_string(picsTaken), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));

        drawCalibration(
            drawImg,
            cameraMatrix,
            distCoeffs,
            stdDeviationsIntrinsics,
            perViewErrors,
            allCharucoCorners,
            allCharucoIds);

        // check the highest per view error and remove it if its higher than 1px.

        if (perViewErrors.size() > 10)
        {
            double maxPerViewError = 0;
            int maxPerViewErrorIdx = 0;

            for (int i = 0; i < perViewErrors.size(); i++)
            {
                if (perViewErrors[i] > maxPerViewError)
                {
                    maxPerViewError = perViewErrors[i];
                    maxPerViewErrorIdx = i;
                }
            }

            if (maxPerViewError > 1)
            {
                perViewErrors.erase(perViewErrors.begin() + maxPerViewErrorIdx);
                allCharucoCorners.erase(allCharucoCorners.begin() + maxPerViewErrorIdx);
                allCharucoIds.erase(allCharucoIds.begin() + maxPerViewErrorIdx);

                // recalibrate camera without the problematic frame
                calibrateFromCharuco(math::GetMatSize(frame.image));

                picsTaken--;
            }
        }

        cvtColor(frame.image, gray, cv::COLOR_BGR2GRAY);
        detector.detectMarkers(gray, markerCorners, markerIds, rejectedCorners);

        // TODO: If markers are detected, the image gets updated, and then the calibration timer below
        // captures another image, in the time before the opencv loop updates the preview on screen,
        // then the masked out tags will still be visible, it probably won't effect much though.
        for (const auto& corners : markerCorners)
        {
            ATT_ASSERT(static_cast<int>(corners.size()) == 4, "A square has four corners.");
            const std::array<cv::Point, 4> points = {corners[0], corners[1], corners[2], corners[3]};
            // much faster than fillPoly, and we know they will be convex
            cv::fillConvexPoly(drawImg, points.data(), points.size(), cv::Scalar::all(255));
        }

        preview.Update(drawImg, user_config.previewImageSize);

        // if more than one second has passed since last calibration image, add current frame to calibration images
        // framesSinceLast++;
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - timeOfLast).count() > 1)
        {
            // framesSinceLast = 0;
            timeOfLast = std::chrono::steady_clock::now();
            // if any button was pressed

            // detect our markers
            detector.refineDetectedMarkers(gray, board, markerCorners, markerIds, rejectedCorners);

            if (markerIds.size() > 0)
            {
                // if markers were found, try to add calibration data
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                // using data from aruco detection we refine the search of chessboard corners for higher accuracy
                charucoDetector.detectBoard(gray, charucoCorners, charucoIds, markerCorners, markerIds);
                if (charucoIds.size() > 15)
                {
                    // if corners were found, we draw them
                    // cv::aruco::drawDetectedCornersCharuco(drawImg, charucoCorners, charucoIds);
                    // we then add our corners to the array
                    allCharucoCorners.push_back(charucoCorners);
                    allCharucoIds.push_back(charucoIds);
                    picsTaken++;

                    if (picsTaken >= 3)
                    {
                        try
                        {
                            // Calibrate camera using our data
                            calibrateFromCharuco(math::GetMatSize(frame.image));
                        }
                        catch (const cv::Exception& e)
                        {
                            ATT_LOG_ERROR(e.what());
                        }

                        std::size_t curI = perViewErrors.size();
                    }
                }
            }
        }
    }

    mainThreadRunning = false;
    if (promptSaveCalib)
    {
        if (cameraMatrix.empty())
        {
            gui->ShowPopup(lc.TRACKER_CAMERA_CALIBRATION_NOTDONE, PopupStyle::Warning);
        }
        else
        {

            // some checks of the camera calibration values. The thresholds should be adjusted to prevent any false  negatives
            //
            // this was before bad frames were being removed, and should no longer be necessary and is commented out since it caused too many false negatives
            /*
            double avgPerViewError = 0;
            double maxPerViewError = 0;

            for (int i = 0; i < perViewErrors.size(); i++)
            {
                avgPerViewError += perViewErrors[i];
                if (perViewErrors[i] > maxPerViewError)
                    maxPerViewError = perViewErrors[i];
            }

            avgPerViewError /= perViewErrors.size();


            if (avgPerViewError > 0.5)          //a big reprojection error indicates that calibration wasnt done properly
            {
                wxMessageDialog dial(NULL, wxT("WARNING:\nThe avarage reprojection error is over 0.5 pixel. This usualy indicates a bad calibration."), wxT("Warning"), wxOK | wxICON_ERROR);
                dial.ShowModal();
            }
            if (maxPerViewError > 10)           //having one reprojection error very high indicates that one frame had missdetections
            {
                wxMessageDialog dial(NULL, wxT("WARNING:\nOne or more reprojection errors are over 10 pixels. This usualy indicates something went wrong during calibration."), wxT("Warning"), wxOK | wxICON_ERROR);
                dial.ShowModal();
            }

            volatile double test = stdDeviationsIntrinsics.at<double>(0);
            test = stdDeviationsIntrinsics.at<double>(1);
            test = stdDeviationsIntrinsics.at<double>(2);
            test = stdDeviationsIntrinsics.at<double>(3);

            if (stdDeviationsIntrinsics.at<double>(0) > 5 || stdDeviationsIntrinsics.at<double>(1) > 5)         //high uncertiancy is bad
            {
                wxMessageDialog dial(NULL, wxT("WARNING:\nThe calibration grid doesnt seem very stable. This usualy indicates a bad calibration."), wxT("Warning"), wxOK | wxICON_ERROR);
                dial.ShowModal();
            }
            */

            // Hand the result over to be stored. Saving happens there, because a calibration
            // that would replace a better saved one has to be confirmed by the user first.
            cfg::CameraCalib fresh;
            fresh.cameraMatrix = cameraMatrix;
            fresh.distortionCoeffs = distCoeffs;
            fresh.stdDeviationsIntrinsics = stdDeviationsIntrinsics;
            fresh.perViewErrors = perViewErrors;
            fresh.allCharucoCorners = allCharucoCorners;
            fresh.allCharucoIds = allCharucoIds;
            SaveCameraCalib(fresh);
        }
    }
}

void Tracker::CalibrateCamera()
{
    // old calibration function, only still here for legacy reasons.

    int CHECKERBOARD[2]{7, 7};

    int blockSize = 125;
    int imageSizeX = blockSize * (CHECKERBOARD[0] + 1);
    int imageSizeY = blockSize * (CHECKERBOARD[1] + 1);
    cv::Mat chessBoard(imageSizeX, imageSizeY, CV_8UC3, cv::Scalar::all(0));
    unsigned char color = 0;

    for (int i = 0; i < imageSizeX - 1; i = i + blockSize)
    {
        if (CHECKERBOARD[1] % 2 == 1)
            color = ~color;
        for (int j = 0; j < imageSizeY - 1; j = j + blockSize)
        {
            cv::Mat ROI = chessBoard(cv::Rect(j, i, blockSize, blockSize));
            ROI.setTo(cv::Scalar::all(color));
            color = ~color;
        }
    }
    // cv::namedWindow("Chessboard", cv::WINDOW_KEEPRATIO);
    // imshow("Chessboard", chessBoard);
    // cv::imwrite("chessboard.png", chessBoard);

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;
    std::vector<cv::Point3f> objp;

    for (int i{0}; i < CHECKERBOARD[0]; i++)
    {
        for (int j{0}; j < CHECKERBOARD[1]; j++)
        {
            objp.push_back(cv::Point3f(static_cast<float>(j), static_cast<float>(i), 0));
        }
    }

    std::vector<cv::Point2f> corner_pts;
    bool success;

    tracker::CapturedFrame frame;
    cv::Mat outImg;

    int i = 0;
    int framesSinceLast = -100;

    int picNum = user_config.cameraCalibSamples;

    cv::Size2i imageSize;

    while (i < picNum)
    {
        if (!mainThreadRunning || !cameraRunning)
        {
            return;
        }
        mCameraFrame.Get(frame);
        cv::Mat& image = frame.image;
        cv::putText(image, std::to_string(i) + "/" + std::to_string(picNum), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        const cv::Size2i drawSize = math::ConstrainSize(math::GetMatSize(image), user_config.previewImageSize);

        framesSinceLast++;
        if (framesSinceLast > 50)
        {
            framesSinceLast = 0;
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

            success = findChessboardCorners(image, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts);

            if (success)
            {
                i++;
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

                cornerSubPix(image, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                drawChessboardCorners(image, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        cv::resize(image, outImg, drawSize);
        gui->UpdatePreview(outImg);

        imageSize = math::GetMatSize(frame.image);
    }

    cv::Mat cameraMatrix, distCoeffs, R, T;

    calibrateCamera(objpoints, imgpoints, imageSize, cameraMatrix, distCoeffs, R, T);

    // The legacy path measures no per view errors, so the saved calibration is protected by
    // asking rather than by comparing, see ShouldConfirmCameraCalibReplacement.
    cfg::CameraCalib fresh;
    fresh.cameraMatrix = cameraMatrix;
    fresh.distortionCoeffs = distCoeffs;
    SaveCameraCalib(fresh);
    mainThreadRunning = false;
}

void Tracker::StartTrackerCalib()
{
    // check that no other process is running on main thread, check that camera is running and calibrated
    if (mainThreadRunning)
    {
        mainThreadRunning = false;
        mainThread.join();
        return;
    }
    if (!cameraRunning)
    {
        gui->ShowPopup(lc.TRACKER_CAMERA_NOTRUNNING, PopupStyle::Error);
        mainThreadRunning = false;
        return;
    }
    if (calib_config.cameras[0]->cameraMatrix.empty())
    {
        gui->ShowPopup(lc.TRACKER_CAMERA_NOTCALIBRATED, PopupStyle::Error);
        mainThreadRunning = false;
        return;
    }

    // ensure joined before creating new thread
    if (mainThread.joinable())
    {
        mainThread.join();
    }
    // start tracker calibration on another thread
    mainThreadRunning = true;
    mainThread = std::thread(&Tracker::CalibrateTracker, this);
}

void Tracker::StartConnection()
{
    if (mVRDriver && mVRClient && mVRClient->IsInit())
    {
        gui->ShowPopup(lc.CONNECT_ALREADYCONNECTED, PopupStyle::Info);
        return;
    }

    if (!TryCreateVRDriver()) return;
    if (!TryInitializeVRClient()) return;
    gui->SetStatus(true, StatusItem::Driver);
}

bool Tracker::TryCreateVRDriver()
{
    try
    {
        mVRDriver = tracker::VRDriver{user_config.trackers};
    }
    catch (const tracker::DriverVersionMismatch& e)
    {
        ATT_LOG_ERROR(e.what());
        gui->ShowPopup(lc.CONNECT_DRIVER_MISSMATCH_1 + e.found.ToString() + lc.CONNECT_DRIVER_MISSMATCH_2 + e.expected.ToString(), PopupStyle::Error);
        mVRDriver.reset();
        gui->SetStatus(false, StatusItem::Driver);
        return false;
    }
    catch (const std::system_error& e)
    {
        ATT_LOG_ERROR(e.what());
        gui->ShowPopup(lc.CONNECT_DRIVER_ERROR + std::to_string(e.code().value()), PopupStyle::Error);
        mVRDriver.reset();
        gui->SetStatus(false, StatusItem::Driver);
        return false;
    }
    catch (const std::exception& e)
    {
        ATT_LOG_ERROR(e.what());
        gui->ShowPopup(lc.CONNECT_SOMETHINGWRONG + (std::string(" ") + e.what()), PopupStyle::Error);
        mVRDriver.reset();
        gui->SetStatus(false, StatusItem::Driver);
        return false;
    }

    return true;
}

bool Tracker::TryInitializeVRClient()
{
    if (!user_config.disableOpenVrApi)
    {
        mVRClient = std::make_unique<tracker::OpenVRClient>();
    }
    else
    {
        mVRClient = std::make_unique<tracker::MockOpenVRClient>();
    }
    if (!mVRClient->CanInit())
    {
        gui->ShowPopup("Unable to initialize steamvr client, is your hmd connected?", PopupStyle::Error);
        mVRClient.reset();
        mVRDriver.reset();
        gui->SetStatus(false, StatusItem::Driver);
        return false;
    }
    try
    {
        mVRClient->Init();
    }
    catch (const std::exception& e)
    {
        ATT_LOG_ERROR(e.what());
        gui->ShowPopup(lc.CONNECT_CLIENT_ERROR + std::string(e.what()), PopupStyle::Error);
        mVRClient.reset();
        mVRDriver.reset();
        gui->SetStatus(false, StatusItem::Driver);
        return false;
    }

    return true;
}

void Tracker::Start()
{
    // check that no other process is running on main thread, check that camera is running and calibrated, check that trackers are calibrated
    if (mainThreadRunning)
    {
        mainThreadRunning = false;
        gui->SetStatus(false, StatusItem::Tracker);
        mainThread.join();
        return;
    }
    if (!cameraRunning)
    {
        gui->ShowPopup(lc.TRACKER_CAMERA_NOTRUNNING, PopupStyle::Error);
        mainThreadRunning = false;
        mainThread.join();
        return;
    }
    if (calib_config.cameras[0]->cameraMatrix.empty())
    {
        gui->ShowPopup(lc.TRACKER_CAMERA_NOTCALIBRATED, PopupStyle::Error);
        mainThreadRunning = false;
        mainThread.join();
        return;
    }
    if (!IsTrackerUnitsCalibrated())
    {
        gui->ShowPopup(lc.TRACKER_TRACKER_NOTCALIBRATED, PopupStyle::Error);
        mainThreadRunning = false;
        mainThread.join();
        return;
    }
    if (!mVRClient->IsInit() || !mVRDriver)
    {
        gui->ShowPopup(lc.TRACKER_STEAMVR_NOTCONNECTED, PopupStyle::Error);
        mainThreadRunning = false;
        mainThread.join();
        return;
    }

    gui->SetStatus(true, StatusItem::Tracker);

    // ensure joined before creating new thread
    if (mainThread.joinable())
    {
        mainThread.join();
    }

    // start detection on another thread
    mainThreadRunning = true;
    mainThread = std::thread(&Tracker::MainLoop, this);
}

void Tracker::Stop()
{
    mainThreadRunning = false;
    cameraRunning = false;

    if (cameraThread.joinable()) cameraThread.join();
    if (mainThread.joinable()) mainThread.join();
}

void Tracker::UpdateConfig()
{
    if (mVRDriver)
    {
        mVRDriver->SetSmoothing(user_config.smoothingFactor, user_config.additionalSmoothing);
    }
}

void Tracker::CalibrateTracker()
{
    utils::RegisterThisThreadName("Calibrate Tracker");

    bool promptSaveCalib = false;
    gui->ShowPrompt(lc.TRACKER_TRACKER_CALIBRATION_INSTRUCTIONS,
                    [&](bool pressedOk) {
                        promptSaveCalib = pressedOk;
                        mainThreadRunning = false;
                    });

    // initialize all parameters needed for tracker calibration
    std::vector<tracker::TrackerUnit> trackerUnits;

    StagWrapper stagDetector{StagWrapper::ConvertLibrary(user_config.markerLibrary), user_config.videoStreams[0]->quadDecimate};
    MarkerDetectionList dets{};

    const Index trackerNum = user_config.trackerNum;
    const int markerCount = StagWrapper::MarkerCount(user_config.markerLibrary);
    const TrackerMarkerIdPartition markerIds{static_cast<int>(trackerNum), user_config.markersPerTracker, markerCount, user_config.trackers};
    const auto referenceMarkerRange = ResolveReferenceMarkerRange(user_config.referenceMarker, markerIds, markerCount);
    const double markerSize = user_config.markerSize * 0.01; // centimeters to meters

    const MarkerCorners3f modelMarker = tracker::TrackerUnit::CreateModelMarker(markerSize);
    constexpr int numCachedToAdd = 50;
    /// maps marker id to many copies of its corners
    std::unordered_map<int, std::vector<MarkerCorners3f>> markersCache;

    // add main marker for every tracker
    for (int i = 0; i < trackerNum; i++)
    {
        tracker::TrackerUnit unit;
        // TODO: dynamically pick the main marker, based on the first seen? need some gui to help as multiple marker tend to get detected in the background while calibrating.
        // might be helpful to draw the id of the marker on each detected, and then some gui to select which detected marker is the main, and which should be added to this one.
        // it should be easy to detect if two markers are moving together, and separate from one not moving in the background
        const int id = markerIds.MainMarkerId(i);
        unit.AddMarker(id, modelMarker);
        trackerUnits.push_back(std::move(unit));
    }
    // The reference board uses the same calibration target and loop as trackers,
    // but its explicitly configured range is never added to the tracker partition.
    if (referenceMarkerRange)
    {
        tracker::TrackerUnit unit;
        unit.AddMarker(referenceMarkerRange->begin, modelMarker);
        trackerUnits.push_back(std::move(unit));
    }

    tracker::CapturedFrame frame;
    cv::Mat grayImage;

    math::EstimatePoseSingleMarkersResult markerPoses;
    const RefPtr<cfg::CameraCalib> camCalib = calib_config.cameras[0];
    auto preview = gui->CreatePreviewControl();

    // TODO: temporary make code easier by allowing returns and handling exceptions properly within loop
    // will be refactored to another class, but easier than pulling out to another function due to amount of state
    const auto doStep = [&] {
        mCameraFrame.Get(frame);
        // detect and draw all markers on image
        StagWrapper::ConvertGrayscale(frame.image, grayImage);
        stagDetector.DetectMarkers(grayImage, dets);
        // draw all markers blue. We will overwrite this with other colors for markers that are part of any of the trackers that we use
        cv::aruco::drawDetectedMarkers(frame.image, dets.corners, dets.ids, COLOR_MARKER_DETECTED);

        math::EstimatePoseSingleMarkers(dets.corners, markerSize, *camCalib, markerPoses);
        ATT_ASSERT(markerPoses.positions.size() == dets.ids.size());
        ATT_ASSERT(markerPoses.rotations.size() == dets.ids.size());
        const double maxDist = user_config.trackerCalibDistance;

        // Tracker roles come from config and are independent of this marker-ID partition.
        // An optional final target is the reference board and follows this exact same path.
        for (Index targetIndex = 0; targetIndex < static_cast<Index>(trackerUnits.size()); ++targetIndex)
        {
            const bool isReferenceMarker = targetIndex == trackerNum;
            auto& unit = trackerUnits[targetIndex];
            // on weird images or calibrations, throws exception. This should usually only happen on bad camera calibrations, or in very rare cases
            auto [boardPose, estimated] = math::EstimatePoseTracker(dets.corners, dets.ids, unit.GetArucoBoard(), *camCalib);
            if (estimated == 0) continue; // no existing markers in this tracker were detected, can't add new ones to it

            cv::drawFrameAxes(frame.image, camCalib->cameraMatrix, camCalib->distortionCoeffs, boardPose.rotation.value, boardPose.position, 0.1F);

            bool foundMarkerToCalibrate = false;
            for (Index detIndex = 0; detIndex < static_cast<Index>(dets.ids.size()); ++detIndex)
            {
                const int detId = dets.ids[detIndex];
                const MarkerCorners2f& detCorners = dets.corners[detIndex];
                const RodrPose detMarkerPose{markerPoses.positions[detIndex], math::RodriguesVec3d(markerPoses.rotations[detIndex])};

                // If the marker is not part of the current tracker, continue to the next detection.
                const bool isInTargetRange = isReferenceMarker
                                                 ? referenceMarkerRange->Contains(detId)
                                                 : markerIds.Contains(static_cast<int>(targetIndex), detId);
                if (!isInTargetRange)
                {
                    continue;
                }

                // the main markers are already added above
                if (unit.HasMarkerId(detId)) // already added to a tracker, draw it green and continue to next detection
                {
                    DrawMarker(frame.image, detCorners, COLOR_MARKER_ADDED);
                    continue;
                }
                const bool isMainMarker = isReferenceMarker
                                              ? detId == referenceMarkerRange->begin
                                              : markerIds.IsMainMarker(static_cast<int>(targetIndex), detId);
                ATT_ASSERT(!isMainMarker, "main marker already added");

                // if marker is too far away from camera, paint it purple, as adding it could have too much error
                if (Length(detMarkerPose.position) > maxDist)
                {
                    DrawMarker(frame.image, detCorners, COLOR_MARKER_FAR);
                    continue;
                }

                DrawMarker(frame.image, detCorners, COLOR_MARKER_ADDING);

                if (foundMarkerToCalibrate) continue;
                foundMarkerToCalibrate = true;

                // append this detection to cornersList for this id
                auto& cornersList = markersCache[detId]; // add or get
                MarkerCorners3f& corners = cornersList.emplace_back(math::NUM_CORNERS);
                TransformMarkerSpace(modelMarker, boardPose, detMarkerPose, corners);

                if (cornersList.size() >= numCachedToAdd)
                {
                    MarkerCorners3f outMedianMarker;
                    FindMedianMarker(cornersList, outMedianMarker);
                    unit.AddMarker(detId, std::move(outMedianMarker));
                }
            }
        }

        if (preview.IsVisible()) preview.Update(frame.image, user_config.previewImageSize);
    };

    // run loop until we stop it
    while (cameraRunning && mainThreadRunning)
    {
        try
        {
            doStep();
        }
        catch (const std::exception& e)
        {
            ATT_LOG_ERROR(e.what());
            gui->ShowPopup(lc.TRACKER_CALIBRATION_SOMETHINGWRONG, PopupStyle::Error);
            mainThreadRunning = false;
            return;
        }
    }
    mainThreadRunning = false;

    if (promptSaveCalib)
    {
        if (referenceMarkerRange)
        {
            const auto& referenceMarker = trackerUnits.at(static_cast<std::size_t>(trackerNum));
            calib_config.referenceMarker.ids = referenceMarker.GetIds();
            calib_config.referenceMarker.corners = referenceMarker.GetMarkers();
        }
        SaveTrackerUnitsToCalib(trackerUnits, trackerNum);
        SetTrackerUnitsFromConfig();
        // the saved marker calibration is what later sessions start from, so show it landed
        RefreshCalibrationStatus();
    }
}

void Tracker::MainLoop()
{
    OptRefPtr<const tracker::TrackerUnit> referenceMarker;
    if (mReferenceMarkerUnit) referenceMarker = &*mReferenceMarkerUnit;
    tracker::MainLoopRunner runner(
        &user_config, &calib_config, &mPlayspace, &mVRDriver.value(), referenceMarker);

    // run detection until camera is stopped or the start/stop button is pressed again
    while (mainThreadRunning && cameraRunning)
    {
        try
        {
            runner.Update(&mCameraFrame, gui, &mTrackerUnits, mVRClient.get(), this);
        }
        catch (const std::exception& e)
        {
            ATT_LOG_ERROR(e.what());
            mainThreadRunning = false;
            gui->ShowPopup(lc.TRACKER_DETECTION_SOMETHINGWRONG, PopupStyle::Error);
        }
    }
    mainThreadRunning = false;
}

namespace
{

void EnsureTrackersConfigSize(UserConfig& userConfig, CalibrationConfig& calibConfig)
{
    const Index expected = std::max(userConfig.trackers.GetSize(), calibConfig.trackers.GetSize());
    userConfig.trackers.Resize(expected);
    calibConfig.trackers.Resize(expected);
}

} // namespace

void Tracker::SetTrackerUnitsFromConfig()
{
    EnsureTrackersConfigSize(user_config, calib_config);
    const int markerCount = StagWrapper::MarkerCount(user_config.markerLibrary);
    const TrackerMarkerIdPartition markerIds{static_cast<int>(user_config.trackers.GetSize()), user_config.markersPerTracker, markerCount, user_config.trackers};
    mTrackerUnits.resize(user_config.trackers.GetSize());
    for (Index i = 0; i < static_cast<Index>(mTrackerUnits.size()); ++i)
    {
        const auto config = user_config.trackers[i];
        const auto calib = calib_config.trackers[i];
        auto& unit = mTrackerUnits[i];
        unit = tracker::TrackerUnit{};
        unit.SetRole(config->role);
        try
        {
            markerIds.EnsureContainsAll(static_cast<int>(i), calib->ids);
            unit.SetMarkers(calib->ids, calib->corners);
            if (user_config.trackerCalibCenters) unit.RecenterMarkers();
        }
        catch (const std::exception& e)
        {
            // A stored calibration can hold marker IDs that the current library or ID ranges
            // no longer cover, which is what switching STag library does to every tracker.
            // Leave that one uncalibrated and say so; refusing to start at all would leave
            // the user with a config they cannot reach the calibration buttons to fix.
            ATT_LOG_ERROR("Ignoring the stored calibration of tracker ", i,
                          ", it has to be calibrated again: ", e.what());
        }
    }

    mReferenceMarkerUnit.reset();
    const auto referenceMarkerRange = ResolveReferenceMarkerRange(user_config.referenceMarker, markerIds, markerCount);
    if (!referenceMarkerRange) return;

    try
    {
        if (calib_config.referenceMarker.ids.size() < 2)
            throw utils::MakeError("reference marker board requires at least two calibrated markers");
        for (const int markerId : calib_config.referenceMarker.ids)
        {
            if (!referenceMarkerRange->Contains(markerId))
                throw utils::MakeError("marker ID ", markerId, " is outside configured reference marker range");
        }
        tracker::TrackerUnit referenceMarker;
        referenceMarker.SetMarkers(calib_config.referenceMarker.ids, calib_config.referenceMarker.corners);
        mReferenceMarkerUnit = std::move(referenceMarker);
    }
    catch (const std::exception& e)
    {
        ATT_LOG_ERROR("Invalid reference marker calibration; reference marker disabled: ", e.what());
        mReferenceMarkerUnit.reset();
    }
}

void Tracker::SaveTrackerUnitsToCalib(const std::vector<tracker::TrackerUnit>& trackerUnits, Index trackerCount)
{
    calib_config.trackers.Resize(trackerCount);
    for (Index i = 0; i < trackerCount; ++i)
    {
        calib_config.trackers[i]->ids = trackerUnits[i].GetIds();
        calib_config.trackers[i]->corners = trackerUnits[i].GetMarkers();
    }
    calib_config.Save();
}

tracker::CalibrationStatus Tracker::RefreshCalibrationStatus()
{
    const auto status = tracker::GetCalibrationStatus(user_config, calib_config);
    gui->SetCalibrationStatus(status.camera, status.calibratedTrackers, status.trackerCount);
    return status;
}

void Tracker::ReportCalibrationStatus()
{
    const auto status = RefreshCalibrationStatus();
    switch (tracker::GetNextCalibrationStep(status))
    {
    case tracker::CalibrationStep::Camera:
        ATT_LOG_INFO("no camera calibration stored, guiding the user through it");
        gui->ShowPopup(lc.CALIBRATION_FIRSTRUN_CAMERA, PopupStyle::Info);
        break;
    case tracker::CalibrationStep::Trackers:
        ATT_LOG_INFO("camera calibration loaded, ", status.calibratedTrackers, " of ",
                     status.trackerCount, " trackers calibrated");
        gui->ShowPopup(lc.CALIBRATION_FIRSTRUN_TRACKERS, PopupStyle::Info);
        break;
    case tracker::CalibrationStep::Done:
        // Nothing to ask for. Say so in the log, the status bar carries it on screen.
        ATT_LOG_INFO("calibration loaded from a previous session: camera and ",
                     status.calibratedTrackers, " trackers");
        break;
    }
}

namespace
{

/// Compact, language neutral summary of a calibration, for the replace prompt.
std::string DescribeCameraCalib(std::string_view label, const tracker::CameraCalibScore& score)
{
    std::ostringstream out;
    out << label << ": ";
    if (!score.present)
    {
        out << "unusable";
    }
    else if (!score.HasErrorData())
    {
        out << "no error data";
    }
    else
    {
        out << score.views << " views, " << std::fixed << std::setprecision(2)
            << score.meanError << " px mean, " << score.maxError << " px max";
    }
    return out.str();
}

} // namespace

void Tracker::SaveCameraCalib(const cfg::CameraCalib& fresh)
{
    const auto stored = tracker::ScoreCameraCalib(*calib_config.cameras[0]);
    const auto stillFresh = tracker::ScoreCameraCalib(fresh);
    if (!tracker::ShouldConfirmCameraCalibReplacement(stored, stillFresh))
    {
        StoreCameraCalib(fresh);
        return;
    }

    // A saved calibration is work the user already did and overwriting it cannot be undone,
    // so show both sets of numbers and let them decide.
    ATT_LOG_INFO("new camera calibration looks worse than the saved one, asking before replacing");
    const U8String message = lc.TRACKER_CAMERA_CALIBRATION_REPLACE +
                             (std::string("\n\n") + DescribeCameraCalib("saved", stored) +
                              "\n" + DescribeCameraCalib("new", stillFresh));
    // Runs on the ui thread after this calibration thread is gone, so capture by value.
    gui->ShowPrompt(message, [this, fresh](bool pressedOk)
        {
            if (!pressedOk)
            {
                ATT_LOG_INFO("kept the saved camera calibration");
                gui->ShowPopup(lc.TRACKER_CAMERA_CALIBRATION_KEPT, PopupStyle::Info);
                return;
            }
            StoreCameraCalib(fresh);
        });
}

void Tracker::StoreCameraCalib(const cfg::CameraCalib& fresh)
{
    *calib_config.cameras[0] = fresh;
    calib_config.Save();
    const auto status = RefreshCalibrationStatus();
    ATT_LOG_INFO("saved camera calibration");

    U8String message = lc.TRACKER_CAMERA_CALIBRATION_COMPLETE;
    if (tracker::GetNextCalibrationStep(status) == tracker::CalibrationStep::Trackers)
    {
        // straight on to the step that is still missing, rather than leaving the user to find it
        message = message + std::string("\n\n") + lc.CALIBRATION_FIRSTRUN_TRACKERS;
    }
    gui->ShowPopup(message, PopupStyle::Info);
}

TEST_CASE("PlayspaceCalib applies a known translation")
{
    tracker::PlayspaceCalib playspace;
    playspace.Set(cv::Vec3d{1, 2, 3}, cv::Vec3d::all(0), 2.0);

    const cv::Point3d transformed = playspace.Transform(cv::Point3d{4, 5, 6});
    CHECK(transformed.x == doctest::Approx(5.0));
    CHECK(transformed.y == doctest::Approx(7.0));
    CHECK(transformed.z == doctest::Approx(9.0));
    CHECK(playspace.GetScale() == doctest::Approx(2.0));
}

TEST_CASE("Tracker marker IDs are partitioned into unchanged contiguous ranges")
{
    const cfg::List<cfg::TrackerUnit> trackerConfigs{3};
    const TrackerMarkerIdPartition markerIds{3, 45, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};

    CHECK(markerIds.MainMarkerId(0) == 0);
    CHECK(markerIds.MainMarkerId(1) == 45);
    CHECK(markerIds.MainMarkerId(2) == 90);

    CHECK(markerIds.Contains(0, 0));
    CHECK(markerIds.Contains(0, 44));
    CHECK(!markerIds.Contains(0, 45));
    CHECK(!markerIds.Contains(1, 44));
    CHECK(markerIds.Contains(1, 45));
    CHECK(markerIds.Contains(1, 89));
    CHECK(!markerIds.Contains(1, 90));

    CHECK(markerIds.IsMainMarker(0, 0));
    CHECK(markerIds.IsMainMarker(1, 45));
    CHECK(markerIds.IsMainMarker(2, 90));
    CHECK(!markerIds.IsMainMarker(0, 1));
    CHECK(!markerIds.IsMainMarker(0, 44));
    CHECK(!markerIds.IsMainMarker(1, 46));
}

TEST_CASE("Tracker marker ID ranges support per-tracker overrides")
{
    cfg::List<cfg::TrackerUnit> trackerConfigs{3};
    trackerConfigs[1]->markerIdBegin = 100;
    trackerConfigs[1]->markerIdEnd = 110;
    trackerConfigs[2]->markerIdBegin = 200;
    trackerConfigs[2]->markerIdEnd = 220;
    const TrackerMarkerIdPartition markerIds{3, 45, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};

    CHECK(markerIds.MainMarkerId(0) == 0);
    CHECK(markerIds.MainMarkerId(1) == 100);
    CHECK(markerIds.MainMarkerId(2) == 200);
    CHECK(markerIds.Contains(1, 109));
    CHECK(!markerIds.Contains(1, 110));
    CHECK(markerIds.IsMainMarker(2, 200));
}

TEST_CASE("Invalid tracker marker ID ranges fall back to all defaults")
{
    // empty range
    {
        cfg::List<cfg::TrackerUnit> trackerConfigs{3};
        trackerConfigs[1]->markerIdBegin = 100;
        trackerConfigs[1]->markerIdEnd = 100;
        const TrackerMarkerIdPartition markerIds{3, 45, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};
        CHECK(markerIds.MainMarkerId(1) == 45);
        CHECK(markerIds.Contains(1, 89));
    }

    // overlap with a default range
    {
        cfg::List<cfg::TrackerUnit> trackerConfigs{3};
        trackerConfigs[1]->markerIdBegin = 44;
        trackerConfigs[1]->markerIdEnd = 60;
        const TrackerMarkerIdPartition markerIds{3, 45, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};
        CHECK(markerIds.MainMarkerId(1) == 45);
        CHECK(markerIds.MainMarkerId(2) == 90);
    }

    // only one endpoint configured
    {
        cfg::List<cfg::TrackerUnit> trackerConfigs{3};
        trackerConfigs[1]->markerIdBegin = 100;
        const TrackerMarkerIdPartition markerIds{3, 45, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};
        CHECK(markerIds.MainMarkerId(1) == 45);
    }
}

TEST_CASE("Serialized tracker calibration IDs must stay inside their configured range")
{
    const cfg::List<cfg::TrackerUnit> trackerConfigs{2};
    const TrackerMarkerIdPartition markerIds{2, 45, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};

    DOCTEST_CHECK_NOTHROW(markerIds.EnsureContainsAll(0, {0, 44}));
    DOCTEST_CHECK_NOTHROW(markerIds.EnsureContainsAll(1, {45, 89}));
    DOCTEST_CHECK_THROWS_AS(markerIds.EnsureContainsAll(0, {45}), utils::Error);
    DOCTEST_CHECK_THROWS_AS(markerIds.EnsureContainsAll(1, {-1}), utils::Error);
}

TEST_CASE("Serialized tracker calibration rejects malformed IDs and corners")
{
    tracker::TrackerUnit unit;
    const MarkerCorners3f marker = tracker::TrackerUnit::CreateModelMarker(0.1);

    DOCTEST_CHECK_NOTHROW(unit.SetMarkers({}, {}));
    DOCTEST_CHECK_NOTHROW(unit.SetMarkers({0}, {marker}));
    DOCTEST_CHECK_THROWS_AS(unit.SetMarkers({-1}, {marker}), utils::Error);
    DOCTEST_CHECK_THROWS_AS(unit.SetMarkers({0, 0}, {marker, marker}), utils::Error);

    MarkerCorners3f nonFiniteMarker = marker;
    nonFiniteMarker[0].x = std::numeric_limits<float>::quiet_NaN();
    DOCTEST_CHECK_THROWS_AS(unit.SetMarkers({0}, {nonFiniteMarker}), utils::Error);

    const MarkerCorners3f zeroMarker(4, cv::Point3f{});
    DOCTEST_CHECK_THROWS_AS(unit.SetMarkers({0}, {zeroMarker}), utils::Error);
}

TEST_CASE("Existing tracker config without marker ID ranges uses defaults")
{
    // an older config selected HD11, whose 22309 markers a large partition fits inside
    const std::string yaml = "%YAML:1.0\n---\nmarkerLibrary: 0\nmarkersPerTracker: 17\ntrackers:\n  - { role: Waist }\n  - { role: LeftFoot }\n  - { role: RightFoot }\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    UserConfig config;
    serial::FileStorageReader reader{storage.root()};
    reader.Read(config);

    CHECK(config.trackers[0]->markerIdBegin == -1);
    CHECK(config.trackers[0]->markerIdEnd == -1);
    const TrackerMarkerIdPartition markerIds{3, config.markersPerTracker, StagWrapper::MarkerCount(config.markerLibrary), config.trackers};
    CHECK(markerIds.MainMarkerId(0) == 0);
    CHECK(markerIds.MainMarkerId(1) == 17);
    CHECK(markerIds.MainMarkerId(2) == 34);
    CHECK(markerIds.Contains(0, 0));
    CHECK(markerIds.Contains(0, 16));
    CHECK(!markerIds.Contains(0, 17));
    CHECK(markerIds.Contains(1, 17));
    CHECK(markerIds.Contains(1, 33));
    CHECK(!markerIds.Contains(1, 34));
    CHECK(markerIds.Contains(2, 34));
    CHECK(markerIds.Contains(2, 50));
    CHECK(!markerIds.Contains(2, 51));
}

TEST_CASE("Existing user config without a reference marker keeps it disabled")
{
    const std::string yaml = "%YAML:1.0\n---\ntrackerNum: 3\nmarkersPerTracker: 17\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    UserConfig config;
    serial::FileStorageReader reader{storage.root()};
    reader.Read(config);

    CHECK_NOT(config.referenceMarker.enabled);
    CHECK(config.referenceMarker.markerIdBegin == -1);
    CHECK(config.referenceMarker.markerIdEnd == -1);
    // the calibration settings added later must not change what an old config means
    CHECK(config.referenceMarker.continuousCalibration);
    CHECK_NOT(config.referenceMarker.recalibrateHmdOffset);
}

TEST_CASE("A stored calibration from another marker library does not stop the app starting")
{
    // Switching STag library invalidates every stored marker ID. That must cost the user a
    // recalibration, not the ability to launch the app and press the calibration button.
    UserConfig userConfig;
    CalibrationConfig calibConfig;
    const MarkerCorners3f marker = tracker::TrackerUnit::CreateModelMarker(0.093);
    // IDs from the old HD11 default partition, outside every range the HD19 default gives out
    calibConfig.trackers[0]->ids = {0};
    calibConfig.trackers[0]->corners = {marker};
    calibConfig.trackers[1]->ids = {45};
    calibConfig.trackers[1]->corners = {marker};
    calibConfig.trackers[2]->ids = {90};
    calibConfig.trackers[2]->corners = {marker};
    const Localization lc;

    const Tracker tracker{userConfig, calibConfig, lc};

    // the trackers that can be kept are kept, and the status reports what is left to redo
    const auto status = tracker::GetCalibrationStatus(userConfig, calibConfig);
    CHECK(status.trackerCount == 3);
    CHECK(status.calibratedTrackers == 3); // the stored file is untouched, nothing is thrown away
}

TEST_CASE("Marker ID ranges are fitted to the markers the selected library actually has")
{
    // HD19 has 38 markers, so the ranges a large markersPerTracker asks for would give
    // trackers a main marker that does not exist. Nothing would ever be detected for them,
    // and the calibration would wait for a marker that cannot be printed.
    constexpr int hd19 = 4;
    constexpr int hd19Count = 38;
    CHECK(STAG_LIBRARY_HDS[hd19] == 19);
    CHECK(StagWrapper::MarkerCount(hd19) == hd19Count);

    const cfg::List<cfg::TrackerUnit> trackerConfigs{3};
    const TrackerMarkerIdPartition fitted{3, 45, hd19Count, trackerConfigs};

    CHECK(fitted.MainMarkerId(0) == 0);
    CHECK(fitted.MainMarkerId(1) == 12);
    CHECK(fitted.MainMarkerId(2) == 24);
    CHECK(fitted.Contains(0, 11));
    CHECK(!fitted.Contains(0, 12));
    CHECK(fitted.Contains(2, 35));
    // every ID handed out has to be one the library can produce
    for (int tracker = 0; tracker < 3; ++tracker)
    {
        CHECK(fitted.MainMarkerId(tracker) < hd19Count);
        CHECK(!fitted.Contains(tracker, hd19Count));
    }
}

TEST_CASE("A default config asks for marker IDs the default library has")
{
    // The shipped defaults have to agree with each other and with the sheets in
    // images-to-print, or a first run cannot detect anything.
    const UserConfig config;
    CHECK(config.markerLibrary == 4); // HD19
    CHECK(config.markersPerTracker == 12);
    CHECK(config.trackerNum == 3);
    const int markerCount = StagWrapper::MarkerCount(config.markerLibrary);
    const TrackerMarkerIdPartition markerIds{
        static_cast<int>(config.trackerNum), config.markersPerTracker, markerCount, config.trackers};

    CHECK(config.trackerNum * config.markersPerTracker <= markerCount);
    for (int tracker = 0; tracker < config.trackerNum; ++tracker)
    {
        CAPTURE(tracker);
        CHECK(markerIds.MainMarkerId(tracker) < markerCount);
    }
}

TEST_CASE("Marker ID ranges past the end of the library are refused")
{
    constexpr int hd19Count = 38;
    cfg::List<cfg::TrackerUnit> trackerConfigs{3};
    trackerConfigs[0]->markerIdBegin = 0;
    trackerConfigs[0]->markerIdEnd = 12;
    trackerConfigs[1]->markerIdBegin = 100;
    trackerConfigs[1]->markerIdEnd = 112;

    // an explicit range outside the library is a config mistake, not something to rewrite
    // silently, so the whole partition falls back to fitted defaults
    const TrackerMarkerIdPartition markerIds{3, 12, hd19Count, trackerConfigs};
    CHECK(markerIds.MainMarkerId(0) == 0);
    CHECK(markerIds.MainMarkerId(1) == 12);
    CHECK(markerIds.MainMarkerId(2) == 24);

    // the same range is fine on a library that reaches that far
    const TrackerMarkerIdPartition onHd11{3, 12, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};
    CHECK(onHd11.MainMarkerId(1) == 100);
}

TEST_CASE("A reference marker range past the end of the library disables it")
{
    constexpr int hd19Count = 38;
    const cfg::List<cfg::TrackerUnit> trackerConfigs{3};
    const TrackerMarkerIdPartition markerIds{3, 12, hd19Count, trackerConfigs};

    cfg::ReferenceMarker referenceConfig;
    referenceConfig.enabled = true;
    referenceConfig.markerIdBegin = 200;
    referenceConfig.markerIdEnd = 210;
    CHECK_NOT(ResolveReferenceMarkerRange(referenceConfig, markerIds, hd19Count).has_value());

    // what is left over after the fitted tracker ranges is what it has to use instead
    referenceConfig.markerIdBegin = 36;
    referenceConfig.markerIdEnd = 38;
    const auto accepted = ResolveReferenceMarkerRange(referenceConfig, markerIds, hd19Count);
    REQUIRE(accepted.has_value());
    CHECK(accepted->begin == 36);
    CHECK(accepted->end == 38);
}

TEST_CASE("Camera and tracker calibration survive being written to a file and read back")
{
    // Both calibrations are meant to be done once and then reused on every later launch, so
    // the round trip through calib.yaml is the property that has to hold. A cv::Mat or a
    // corner list that does not survive it would send the user back to recalibrating.
    const auto path = std::filesystem::temp_directory_path() / "att-calib-roundtrip-test.yaml";
    std::filesystem::remove(path);

    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 612.5, 0, 319.5, 0, 611.25, 239.5, 0, 0, 1);
    const cv::Mat distortionCoeffs = (cv::Mat_<double>(1, 5) << 0.1, -0.25, 0.001, -0.002, 0.05);
    const MarkerCorners3f firstMarker = tracker::TrackerUnit::CreateModelMarker(0.093);
    MarkerCorners3f secondMarker = firstMarker;
    for (auto& corner : secondMarker) corner.z += 0.02F;

    {
        CalibrationConfig written;
        written.SetPath(path);
        written.cameras[0]->cameraMatrix = cameraMatrix;
        written.cameras[0]->distortionCoeffs = distortionCoeffs;
        written.cameras[0]->perViewErrors = {0.21, 0.34};
        written.trackers[0]->ids = {0, 1};
        written.trackers[0]->corners = {firstMarker, secondMarker};
        written.referenceMarkerOffset.calibrated = true;
        written.referenceMarkerOffset.position = {0.02, 0.06, -0.11};
        written.referenceMarkerOffset.rotation = {0.1, -0.2, 0.3};
        written.referenceMarkerOffset.scale = 1.02;
        REQUIRE(written.Save());
    }

    CalibrationConfig read;
    read.SetPath(path);
    REQUIRE(read.Load());

    const auto readCamera = read.cameras[0];
    REQUIRE_NOT(readCamera->cameraMatrix.empty());
    CHECK(readCamera->cameraMatrix.rows == 3);
    CHECK(readCamera->cameraMatrix.cols == 3);
    CHECK(cv::norm(readCamera->cameraMatrix - cameraMatrix) == doctest::Approx(0.0));
    CHECK(cv::norm(readCamera->distortionCoeffs - distortionCoeffs) == doctest::Approx(0.0));
    REQUIRE(readCamera->perViewErrors.size() == 2);
    CHECK(readCamera->perViewErrors[1] == doctest::Approx(0.34));

    const auto readTracker = read.trackers[0];
    REQUIRE(readTracker->ids.size() == 2);
    CHECK(readTracker->ids[1] == 1);
    REQUIRE(readTracker->corners.size() == 2);
    REQUIRE(readTracker->corners[1].size() == 4);
    CHECK(readTracker->corners[1][2].z == doctest::Approx(secondMarker[2].z));
    CHECK(readTracker->corners[0][0].x == doctest::Approx(firstMarker[0].x));

    // a stored calibration must also be usable, not merely present
    tracker::TrackerUnit unit;
    unit.SetMarkers(readTracker->ids, readTracker->corners);
    CHECK(unit.IsCalibrated());

    CHECK(read.referenceMarkerOffset.calibrated);
    CHECK(read.referenceMarkerOffset.scale == doctest::Approx(1.02));
    CHECK(read.referenceMarkerOffset.position[Z] == doctest::Approx(-0.11));

    std::filesystem::remove(path);
}

TEST_CASE("Existing calibration config without a stored board offset asks for it to be measured")
{
    const std::string yaml = "%YAML:1.0\n---\ncameras:\n  - { }\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    CalibrationConfig config;
    serial::FileStorageReader reader{storage.root()};
    reader.Read(config);

    CHECK_NOT(config.referenceMarkerOffset.calibrated);
    CHECK(config.referenceMarkerOffset.scale == doctest::Approx(1.0));
    CHECK(cv::norm(config.referenceMarkerOffset.position) == doctest::Approx(0.0));
    CHECK(cv::norm(config.referenceMarkerOffset.rotation) == doctest::Approx(0.0));
}

TEST_CASE("A stored board offset is read back as the pose it was solved as")
{
    const std::string yaml =
        "%YAML:1.0\n---\n"
        "referenceMarkerOffset: { calibrated: 1, position: [ 2.0000000000000001e-02, "
        "6.0000000000000002e-02, -1.1e-01 ], rotation: [ 1.0e-01, -2.0e-01, 3.0e-01 ], "
        "scale: 1.0200000000000000e+00 }\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    CalibrationConfig config;
    serial::FileStorageReader reader{storage.root()};
    reader.Read(config);

    const auto& stored = config.referenceMarkerOffset;
    REQUIRE(stored.calibrated);
    CHECK(stored.position[X] == doctest::Approx(0.02));
    CHECK(stored.position[Y] == doctest::Approx(0.06));
    CHECK(stored.position[Z] == doctest::Approx(-0.11));
    CHECK(stored.scale == doctest::Approx(1.02));

    // this is how MainLoopRunner restores it, so an angle axis that does not survive the
    // trip would silently move the whole playspace on the next session
    const cv::Vec3d rotation{0.1, -0.2, 0.3};
    const Pose offset{cv::Point3d(stored.position), cv::Quatd::createFromRvec(stored.rotation)};
    const double rotationError = tracker::RotationAngleBetween(
        offset.rotation, cv::Quatd::createFromRvec(rotation));
    CAPTURE(rotationError);
    CHECK(rotationError < 1e-9);
}

TEST_CASE("Valid explicit reference marker range is read without changing tracker ranges")
{
    const std::string yaml =
        "%YAML:1.0\n---\nmarkerLibrary: 0\nmarkersPerTracker: 17\n"
        "referenceMarker: { enabled: 1, markerIdBegin: 100, markerIdEnd: 110 }\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    UserConfig config;
    serial::FileStorageReader reader{storage.root()};
    reader.Read(config);

    const TrackerMarkerIdPartition markerIds{3, config.markersPerTracker, StagWrapper::MarkerCount(config.markerLibrary), config.trackers};
    const auto referenceRange = ResolveReferenceMarkerRange(config.referenceMarker, markerIds, StagWrapper::MarkerCount(config.markerLibrary));

    REQUIRE(referenceRange.has_value());
    CHECK(referenceRange->begin == 100);
    CHECK(referenceRange->end == 110);
    CHECK(markerIds.MainMarkerId(0) == 0);
    CHECK(markerIds.MainMarkerId(1) == 17);
    CHECK(markerIds.MainMarkerId(2) == 34);
}

TEST_CASE("Invalid reference marker ranges disable only the reference marker")
{
    const cfg::List<cfg::TrackerUnit> trackerConfigs{3};
    const TrackerMarkerIdPartition markerIds{3, 45, STAG_LIBRARY_MARKER_COUNTS[0], trackerConfigs};
    cfg::ReferenceMarker referenceConfig;
    referenceConfig.enabled = true;

    const auto isAccepted = [&](int begin, int end)
    {
        referenceConfig.markerIdBegin = begin;
        referenceConfig.markerIdEnd = end;
        return ResolveReferenceMarkerRange(referenceConfig, markerIds, STAG_LIBRARY_MARKER_COUNTS[0]).has_value();
    };

    CHECK_NOT(isAccepted(-1, 10));
    CHECK_NOT(isAccepted(200, 200));
    CHECK_NOT(isAccepted(201, 200));
    CHECK_NOT(isAccepted(44, 50));

    // Reference validation is atomic and never rewrites the tracker partition.
    CHECK(markerIds.MainMarkerId(0) == 0);
    CHECK(markerIds.MainMarkerId(1) == 45);
    CHECK(markerIds.MainMarkerId(2) == 90);
    CHECK(markerIds.Contains(0, 44));
    CHECK(markerIds.Contains(1, 45));
}

TEST_CASE("Missing reference marker detections return no pose")
{
    tracker::TrackerUnit referenceMarker;
    const MarkerCorners3f firstMarker = tracker::TrackerUnit::CreateModelMarker(0.1);
    MarkerCorners3f secondMarker = firstMarker;
    for (auto& corner : secondMarker) corner.x += 0.2F;
    referenceMarker.SetMarkers({200, 201}, {firstMarker, secondMarker});
    const MarkerDetectionList detections;
    const cfg::CameraCalib camera;

    const auto pose = tracker::EstimateReferenceMarkerPose(
        detections, referenceMarker.GetArucoBoard(), camera);

    CHECK_NOT(pose.has_value());
}

TEST_CASE("Existing user config keeps the default preview image size")
{
    const std::string yaml = "%YAML:1.0\n---\ntrackerNum: 3\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    UserConfig config;
    serial::FileStorageReader reader{storage.root()};
    reader.Read(config);

    CHECK(config.previewImageSize == 480);
}

TEST_CASE("Existing user config ignores the removed ignoreTracker0 key")
{
    const std::string yaml = "%YAML:1.0\n---\nignoreTracker0: 1\ntrackerNum: 7\nmarkerSize: 12.5\nmarkersPerTracker: 17\npreviewImageSize: 360\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    UserConfig config;
    serial::FileStorageReader reader{storage.root()};
    DOCTEST_CHECK_NOTHROW(reader.Read(config));

    CHECK(config.trackerNum == 7);
    CHECK(config.markerSize.Get() == doctest::Approx(12.5));
    CHECK(config.markersPerTracker == 17);
    CHECK(config.previewImageSize == 360);
}

TEST_CASE("markersPerTracker restores its fallback without changing valid values")
{
    // The fallback follows the default, which HD19 and its 38 markers set to 12.
    const auto readMarkersPerTracker = [](int value)
    {
        const std::string yaml = "%YAML:1.0\n---\nmarkersPerTracker: " + std::to_string(value) + "\n";
        cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
        UserConfig config;
        serial::FileStorageReader reader{storage.root()};
        reader.Read(config);
        return config.markersPerTracker.Get();
    };

    CHECK(readMarkersPerTracker(-1) == 12);
    CHECK(readMarkersPerTracker(0) == 12);
    CHECK(readMarkersPerTracker(1) == 1);
    CHECK(readMarkersPerTracker(45) == 45);
    CHECK(readMarkersPerTracker(123) == 123);
}

TEST_CASE("PlayspaceCalib pose transforms round trip")
{
    tracker::PlayspaceCalib playspace;
    playspace.Set(cv::Vec3d{1.5, -2.0, 0.75}, cv::Vec3d{0.2, -0.4, 0.1}, 1.25);
    const Pose original{
        cv::Point3d{4.0, -3.0, 2.0},
        cv::Quatd::createFromRvec(cv::Vec3d{0.1, 0.3, -0.2})};

    const Pose restored = playspace.InvTransform(playspace.Transform(original));
    CHECK(restored.position.x == doctest::Approx(original.position.x).epsilon(1e-12));
    CHECK(restored.position.y == doctest::Approx(original.position.y).epsilon(1e-12));
    CHECK(restored.position.z == doctest::Approx(original.position.z).epsilon(1e-12));
    CHECK(restored.rotation.w == doctest::Approx(original.rotation.w).epsilon(1e-12));
    CHECK(restored.rotation.x == doctest::Approx(original.rotation.x).epsilon(1e-12));
    CHECK(restored.rotation.y == doctest::Approx(original.rotation.y).epsilon(1e-12));
    CHECK(restored.rotation.z == doctest::Approx(original.rotation.z).epsilon(1e-12));

    const Pose restoredFromOVR = playspace.InvTransformFromOVR(playspace.TransformToOVR(original));
    CHECK(restoredFromOVR.position.x == doctest::Approx(original.position.x).epsilon(1e-12));
    CHECK(restoredFromOVR.position.y == doctest::Approx(original.position.y).epsilon(1e-12));
    CHECK(restoredFromOVR.position.z == doctest::Approx(original.position.z).epsilon(1e-12));
    CHECK(restoredFromOVR.rotation.w == doctest::Approx(original.rotation.w).epsilon(1e-12));
    CHECK(restoredFromOVR.rotation.x == doctest::Approx(original.rotation.x).epsilon(1e-12));
    CHECK(restoredFromOVR.rotation.y == doctest::Approx(original.rotation.y).epsilon(1e-12));
    CHECK(restoredFromOVR.rotation.z == doctest::Approx(original.rotation.z).epsilon(1e-12));
}
