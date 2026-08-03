#pragma once

#include "math/CVTypes.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <vector>

/// STag marker library HD (hamming distance) values, indexed by UserConfig::markerLibrary.
/// Larger HD means fewer but more robust markers.
constexpr std::array<int, 7> STAG_LIBRARY_HDS = {11, 13, 15, 17, 19, 21, 23};

/// Markers each library contains, indexed like STAG_LIBRARY_HDS. The valid marker IDs of a
/// library are [0, count), and these match the line counts of utilities/stag-codebooks/HD*.txt,
/// which is where the detector's codebooks come from.
/// An ID at or past the count of the library in use can never be detected, so the ID ranges
/// handed out to trackers have to stay inside it. HD19 in particular only has 38 markers,
/// far fewer than the ID ranges a large markersPerTracker would ask for.
constexpr std::array<int, 7> STAG_LIBRARY_MARKER_COUNTS = {22309, 2884, 766, 157, 38, 12, 6};

struct MarkerDetectionList
{
    std::vector<int> ids{};
    std::vector<MarkerCorners2f> corners{}; /// 4 corners in CW order (TL, TR, BR, BL), aruco compatible
};

class StagWrapper
{
public:
    /// @param libraryHD one of STAG_LIBRARY_HDS
    /// @param decimate detection downscale factor >= 1,
    ///  detection runs on the downscaled image while corners are refined at full resolution
    StagWrapper(int libraryHD, double decimate);

    /// convert UserConfig::markerLibrary choice index to a library HD value
    static int ConvertLibrary(int libraryIndex)
    {
        if (libraryIndex < 0 || libraryIndex >= static_cast<int>(STAG_LIBRARY_HDS.size()))
        {
            return STAG_LIBRARY_HDS[0];
        }
        return STAG_LIBRARY_HDS[libraryIndex];
    }

    /// markers available in the library selected by a UserConfig::markerLibrary choice index
    static int MarkerCount(int libraryIndex)
    {
        if (libraryIndex < 0 || libraryIndex >= static_cast<int>(STAG_LIBRARY_MARKER_COUNTS.size()))
        {
            return STAG_LIBRARY_MARKER_COUNTS[0];
        }
        return STAG_LIBRARY_MARKER_COUNTS[libraryIndex];
    }

    /// convert BGR image to single channel grayscale
    static void ConvertGrayscale(const cv::Mat& image, cv::Mat& outImage)
    {
        cv::cvtColor(image, outImage, cv::COLOR_BGR2GRAY);
    }

    void DetectMarkers(const cv::Mat& frame, MarkerDetectionList& outList);

private:
    int mLibraryHD;
    double mDecimate;
    cv::Mat mDecimatedImage{}; /// reused decimation buffer
};
