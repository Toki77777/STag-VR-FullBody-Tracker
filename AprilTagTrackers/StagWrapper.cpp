#include "StagWrapper.hpp"

#include "utils/Assert.hpp"
#include "utils/Test.hpp"

#include <stag/Stag.h>

#include <algorithm>
#include <cmath>
#include <limits>

StagWrapper::StagWrapper(int libraryHD, double decimate)
    : mLibraryHD(libraryHD), mDecimate(std::max(decimate, 1.0))
{
    ATT_ASSERT(std::find(STAG_LIBRARY_HDS.begin(), STAG_LIBRARY_HDS.end(), libraryHD) != STAG_LIBRARY_HDS.end());
}

namespace
{

/// refine corners detected on a decimated image using the full resolution image,
/// skips markers with corners too close to the image border for the search window
void RefineCornersFullRes(const cv::Mat& frame, std::vector<MarkerCorners2f>& cornersList, double decimate)
{
    // search radius should cover the position error introduced by decimation
    const int winSize = std::clamp(static_cast<int>(std::lround(decimate)) + 1, 3, 5);
    const int margin = winSize + 2;
    const cv::Rect2f safeArea(
        static_cast<float>(margin), static_cast<float>(margin),
        static_cast<float>(frame.cols - 2 * margin), static_cast<float>(frame.rows - 2 * margin));
    const cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 20, 0.05);

    for (MarkerCorners2f& corners : cornersList)
    {
        const bool inside = std::all_of(corners.begin(), corners.end(),
            [&](const cv::Point2f& corner) { return safeArea.contains(corner); });
        if (!inside) continue;
        cv::cornerSubPix(frame, corners, cv::Size(winSize, winSize), cv::Size(-1, -1), criteria);
    }
}

TEST_CASE("StagWrapper::ConvertLibrary maps every supported library")
{
    constexpr std::array<int, 7> expected{11, 13, 15, 17, 19, 21, 23};
    for (int index = 0; index < static_cast<int>(expected.size()); ++index)
    {
        CAPTURE(index);
        CHECK(StagWrapper::ConvertLibrary(index) == expected[index]);
    }
}

TEST_CASE("StagWrapper::ConvertLibrary falls back for invalid indices")
{
    CHECK(StagWrapper::ConvertLibrary(-1) == STAG_LIBRARY_HDS[0]);
    CHECK(StagWrapper::ConvertLibrary(static_cast<int>(STAG_LIBRARY_HDS.size())) == STAG_LIBRARY_HDS[0]);
    CHECK(StagWrapper::ConvertLibrary(std::numeric_limits<int>::min()) == STAG_LIBRARY_HDS[0]);
    CHECK(StagWrapper::ConvertLibrary(std::numeric_limits<int>::max()) == STAG_LIBRARY_HDS[0]);
}

} // namespace

void StagWrapper::DetectMarkers(const cv::Mat& frame, MarkerDetectionList& outList)
{
    ATT_ASSERT(frame.type() == CV_8U);
    outList.ids.clear();
    outList.corners.clear();

    if (mDecimate > 1.0)
    {
        const double invDecimate = 1.0 / mDecimate;
        cv::resize(frame, mDecimatedImage, cv::Size(), invDecimate, invDecimate, cv::INTER_AREA);
        stag::detectMarkers(mDecimatedImage, mLibraryHD, outList.corners, outList.ids);
        if (outList.corners.empty()) return;

        const double scaleX = static_cast<double>(frame.cols) / static_cast<double>(mDecimatedImage.cols);
        const double scaleY = static_cast<double>(frame.rows) / static_cast<double>(mDecimatedImage.rows);
        for (MarkerCorners2f& corners : outList.corners)
        {
            for (cv::Point2f& corner : corners)
            {
                corner.x = static_cast<float>(corner.x * scaleX);
                corner.y = static_cast<float>(corner.y * scaleY);
            }
        }
        RefineCornersFullRes(frame, outList.corners, mDecimate);
    }
    else
    {
        stag::detectMarkers(frame, mLibraryHD, outList.corners, outList.ids);
    }
}
