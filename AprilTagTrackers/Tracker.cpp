#include "Tracker.hpp"

#include "config/TrackerUnit.hpp"
#include "Helpers.hpp"
#include "ImageDrawing.hpp"
#include "math/CVHelpers.hpp"
#include "StagWrapper.hpp"
#include "tracker/MainLoopRunner.hpp"
#include "tracker/TrackerUnit.hpp"
#include "utils/Assert.hpp"
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
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace
{

class TrackerMarkerIdPartition
{
    struct Range
    {
        int begin;
        int end;

        bool Contains(int markerId) const
        {
            return markerId >= begin && markerId < end;
        }

        bool Overlaps(const Range& other) const
        {
            return begin < other.end && other.begin < end;
        }
    };

public:
    TrackerMarkerIdPartition(int trackerCount, int markersPerTracker, const cfg::List<cfg::TrackerUnit>& trackerConfigs)
        : mTrackerCount(std::max(0, trackerCount)), mMarkersPerTracker(markersPerTracker)
    {
        ResetToDefaults();
        if (!TryApplyConfiguredRanges(trackerConfigs))
        {
            ATT_LOG_ERROR("Invalid or overlapping tracker marker ID ranges; using markersPerTracker defaults.");
            ResetToDefaults();
        }
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

private:
    const Range& GetRange(int trackerIndex) const
    {
        return mRanges.at(static_cast<std::size_t>(trackerIndex));
    }

    void ResetToDefaults()
    {
        mRanges.clear();
        mRanges.reserve(static_cast<std::size_t>(mTrackerCount));
        for (int trackerIndex = 0; trackerIndex < mTrackerCount; ++trackerIndex)
        {
            mRanges.push_back({trackerIndex * mMarkersPerTracker, (trackerIndex + 1) * mMarkersPerTracker});
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
    std::vector<Range> mRanges;
};

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

        preview.Update(drawImg, DRAW_IMG_SIZE);

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

            // Save calibration to our global params cameraMatrix and distCoeffs
            RefPtr<cfg::CameraCalib> camCalib = calib_config.cameras[0];
            camCalib->cameraMatrix = cameraMatrix;
            camCalib->distortionCoeffs = distCoeffs;
            camCalib->stdDeviationsIntrinsics = stdDeviationsIntrinsics;
            camCalib->perViewErrors = perViewErrors;
            camCalib->allCharucoCorners = allCharucoCorners;
            camCalib->allCharucoIds = allCharucoIds;
            calib_config.Save();
            gui->ShowPopup(lc.TRACKER_CAMERA_CALIBRATION_COMPLETE, PopupStyle::Info);
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
        const cv::Size2i drawSize = math::ConstrainSize(math::GetMatSize(image), DRAW_IMG_SIZE);

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

    RefPtr<cfg::CameraCalib> camCalib = calib_config.cameras[0];

    camCalib->cameraMatrix = cameraMatrix;
    camCalib->distortionCoeffs = distCoeffs;
    calib_config.Save();
    mainThreadRunning = false;
    gui->ShowPopup("Calibration complete.", PopupStyle::Info);
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
    const TrackerMarkerIdPartition markerIds{static_cast<int>(trackerNum), user_config.markersPerTracker, user_config.trackers};
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
        for (int trackerIndex = 0; trackerIndex < trackerNum; ++trackerIndex)
        {
            auto& unit = trackerUnits[trackerIndex];
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
                if (!markerIds.Contains(trackerIndex, detId))
                {
                    continue;
                }

                // the main markers are already added above
                if (unit.HasMarkerId(detId)) // already added to a tracker, draw it green and continue to next detection
                {
                    DrawMarker(frame.image, detCorners, COLOR_MARKER_ADDED);
                    continue;
                }
                ATT_ASSERT(!markerIds.IsMainMarker(trackerIndex, detId), "main marker already added");

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

        if (preview.IsVisible()) preview.Update(frame.image, DRAW_IMG_SIZE);
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
        SaveTrackerUnitsToCalib(trackerUnits);
        SetTrackerUnitsFromConfig();
    }
}

void Tracker::MainLoop()
{
    tracker::MainLoopRunner runner(&user_config, &calib_config, &mPlayspace, &mVRDriver.value());

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
    if (user_config.trackers.GetSize() == 0) return; // not calibrated yet

    mTrackerUnits.resize(user_config.trackers.GetSize());
    for (Index i = 0; i < static_cast<Index>(mTrackerUnits.size()); ++i)
    {
        const auto config = user_config.trackers[i];
        const auto calib = calib_config.trackers[i];
        auto& unit = mTrackerUnits[i];
        unit.SetMarkers(calib->ids, calib->corners);
        if (user_config.trackerCalibCenters) unit.RecenterMarkers();
        unit.SetRole(config->role);
    }
}

void Tracker::SaveTrackerUnitsToCalib(const std::vector<tracker::TrackerUnit>& trackerUnits)
{
    calib_config.trackers.Resize(trackerUnits.size());
    for (Index i = 0; i < static_cast<Index>(trackerUnits.size()); ++i)
    {
        calib_config.trackers[i]->ids = trackerUnits[i].GetIds();
        calib_config.trackers[i]->corners = trackerUnits[i].GetMarkers();
    }
    calib_config.Save();
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
    const TrackerMarkerIdPartition markerIds{3, 45, trackerConfigs};

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
    const TrackerMarkerIdPartition markerIds{3, 45, trackerConfigs};

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
        const TrackerMarkerIdPartition markerIds{3, 45, trackerConfigs};
        CHECK(markerIds.MainMarkerId(1) == 45);
        CHECK(markerIds.Contains(1, 89));
    }

    // overlap with a default range
    {
        cfg::List<cfg::TrackerUnit> trackerConfigs{3};
        trackerConfigs[1]->markerIdBegin = 44;
        trackerConfigs[1]->markerIdEnd = 60;
        const TrackerMarkerIdPartition markerIds{3, 45, trackerConfigs};
        CHECK(markerIds.MainMarkerId(1) == 45);
        CHECK(markerIds.MainMarkerId(2) == 90);
    }

    // only one endpoint configured
    {
        cfg::List<cfg::TrackerUnit> trackerConfigs{3};
        trackerConfigs[1]->markerIdBegin = 100;
        const TrackerMarkerIdPartition markerIds{3, 45, trackerConfigs};
        CHECK(markerIds.MainMarkerId(1) == 45);
    }
}

TEST_CASE("Existing tracker config without marker ID ranges uses defaults")
{
    const std::string yaml = "%YAML:1.0\n---\nmarkersPerTracker: 45\ntrackers:\n  - { role: Waist }\n  - { role: LeftFoot }\n  - { role: RightFoot }\n";
    cv::FileStorage storage{yaml, cv::FileStorage::READ | cv::FileStorage::MEMORY};
    UserConfig config;
    serial::FileStorageReader reader{storage.root()};
    reader.Read(config);

    CHECK(config.trackers[0]->markerIdBegin == -1);
    CHECK(config.trackers[0]->markerIdEnd == -1);
    const TrackerMarkerIdPartition markerIds{3, config.markersPerTracker, config.trackers};
    CHECK(markerIds.MainMarkerId(0) == 0);
    CHECK(markerIds.MainMarkerId(1) == 45);
    CHECK(markerIds.MainMarkerId(2) == 90);
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
