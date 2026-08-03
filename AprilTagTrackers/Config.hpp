#pragma once

#include "config/List.hpp"
#include "config/ManualCalib.hpp"
#include "config/TrackerUnit.hpp"
#include "config/Validated.hpp"
#include "config/VideoStream.hpp"
#include "serial/Comment.hpp"
#include "serial/Serializable.hpp"
#include "utils/Env.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

// Create definitions for each config file below

// Non-user editable calibration data, long lists of numbers.
// Potentially store this in a file.yaml.gz to reduce size,
//  and show that it is not user editable. FileStorage has this ability built in
class CalibrationConfig : public serial::Serializable<CalibrationConfig>
{
public:
    CalibrationConfig() : Serializable(utils::GetConfigDir() / "calib.yaml") {}

    REFLECTABLE_BEGIN;
    REFLECTABLE_FIELD(cfg::List<cfg::CameraCalib>, cameras){1};
    REFLECTABLE_FIELD(cfg::List<cfg::TrackerUnitCalib>, trackers){3};
    REFLECTABLE_FIELD(cfg::TrackerUnitCalib, referenceMarker){};
    REFLECTABLE_FIELD(cfg::ReferenceMarkerOffset, referenceMarkerOffset){};
    REFLECTABLE_END;
};

// user editable storage
class UserConfig : public serial::Serializable<UserConfig>
{
public:
    UserConfig() : Serializable(utils::GetConfigDir() / "config.yaml") {}

    REFLECTABLE_BEGIN;
    REFLECTABLE_FIELD(std::string, windowTitle);
    // Keep synced with Localization::LANG_CODE_MAP
    ATT_SERIAL_COMMENT("en, ja, ru, zh-cn");
    REFLECTABLE_FIELD(std::string, langCode) = "en";
    REFLECTABLE_FIELD(int, trackerNum) = 3;
    REFLECTABLE_FIELD(cfg::Validated<double>, markerSize){9.3, cfg::GreaterEqual(0.01)};
    // Legacy compatibility: ebd8c5e disabled its median filter and f77d85f removed the reader; retained pending a future decision.
    REFLECTABLE_FIELD(int, numOfPrevValues) = 5;
    REFLECTABLE_FIELD(bool, usePredictive) = true;
    // Legacy compatibility: e79c6c5 intentionally folded this choice into markerLibrary; retained pending a future decision.
    REFLECTABLE_FIELD(bool, coloredMarkers) = true;
    REFLECTABLE_FIELD(cfg::ManualCalib, manualCalib){};
    REFLECTABLE_FIELD(bool, chessboardCalib) = false;
    REFLECTABLE_FIELD(cfg::Validated<double>, smoothingFactor){0.5, cfg::Clamp(0.0, 1.0)};
    // Legacy compatibility: e79c6c5 intentionally folded this choice into markerLibrary; retained pending a future decision.
    REFLECTABLE_FIELD(bool, circularMarkers) = false;
    REFLECTABLE_FIELD(cfg::Validated<double>, trackerCalibDistance){0.5, cfg::GreaterEqual(0.5)};
    /// TODO: change to not validated, gets set during calibration, to indicate if the user has done calibration
    REFLECTABLE_FIELD(cfg::Validated<int>, cameraCalibSamples){15, cfg::GreaterEqual(15)};
    REFLECTABLE_FIELD(bool, trackerCalibCenters) = false;
    REFLECTABLE_FIELD(cfg::Validated<double>, depthSmoothing){0, cfg::Clamp(0.0, 1.0)};
    REFLECTABLE_FIELD(float, additionalSmoothing) = 0;
    ATT_SERIAL_COMMENT("Maximum preview image dimension in pixels");
    REFLECTABLE_FIELD(cfg::Validated<int>, previewImageSize){480, cfg::GreaterEqual(1)};
    ATT_SERIAL_COMMENT("STag marker library: 0=HD11, 1=HD13, 2=HD15, 3=HD17, 4=HD19, 5=HD21, 6=HD23");
    ATT_SERIAL_COMMENT("markers available: 22309, 2884, 766, 157, 38, 12, 6 - marker IDs must stay below this");
    REFLECTABLE_FIELD(int, markerLibrary) = 4;
    ATT_SERIAL_COMMENT("ID range width per tracker; trackerNum * this must fit the marker library");
    REFLECTABLE_FIELD(cfg::Validated<int>, markersPerTracker){12, [](int& value)
                                                              {
                                                                  if (value <= 0) value = 12;
                                                              }};
    REFLECTABLE_FIELD(bool, disableOpenVrApi) = false;
    REFLECTABLE_FIELD(cfg::List<cfg::VideoStream>, videoStreams){1};
    REFLECTABLE_FIELD(cfg::List<cfg::TrackerUnit>, trackers){3};
    REFLECTABLE_FIELD(cfg::ReferenceMarker, referenceMarker){};
    REFLECTABLE_END;

    CalibrationConfig calib{};
};
