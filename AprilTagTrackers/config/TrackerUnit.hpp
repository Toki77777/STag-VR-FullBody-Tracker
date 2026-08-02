#pragma once

#include "math/CVTypes.hpp"
#include "serial/Comment.hpp"
#include "serial/Serial.hpp"
#include "utils/Enum.hpp"
#include "utils/Reflectable.hpp"

#include <opencv2/core/types.hpp>

#include <array>
#include <vector>

namespace cfg
{

ATT_ENUM(TrackerRole,
         Disabled,
         Waist,
         RightFoot,
         LeftFoot)

struct TrackerUnit
{
    REFLECTABLE_BEGIN;
    REFLECTABLE_FIELD(TrackerRole, role);
    ATT_SERIAL_COMMENT("Marker ID range [begin, end); keep both at -1 to use markersPerTracker defaults");
    REFLECTABLE_FIELD(int, markerIdBegin) = -1;
    REFLECTABLE_FIELD(int, markerIdEnd) = -1;
    REFLECTABLE_END;
};

struct TrackerUnitCalib
{
    REFLECTABLE_BEGIN;
    REFLECTABLE_FIELD(std::vector<std::vector<cv::Point3f>>, corners);
    REFLECTABLE_FIELD(std::vector<int>, ids);
    REFLECTABLE_END;
};

struct ReferenceMarker
{
    REFLECTABLE_BEGIN;
    REFLECTABLE_FIELD(bool, enabled) = false;
    ATT_SERIAL_COMMENT("Required marker ID range [begin, end) when enabled; no implicit range is assigned");
    REFLECTABLE_FIELD(int, markerIdBegin) = -1;
    REFLECTABLE_FIELD(int, markerIdEnd) = -1;
    REFLECTABLE_END;
};

} // namespace cfg

template <>
struct serial::Serial<cfg::TrackerRole>
{
    static void Parse(auto& ctx, cfg::TrackerRole& outValue)
    {
        std::string roleStr;
        ctx.Read(roleStr);
        outValue = utils::renum::FromString<cfg::TrackerRole>(roleStr)
                       .value_or(cfg::TrackerRole::Disabled);
    }
    static void Format(auto& ctx, cfg::TrackerRole value)
    {
        std::string_view roleStr = utils::renum::ToString(value);
        ctx.WriteComment(utils::renum::Stringized<cfg::TrackerRole>);
        ctx.Write(roleStr);
    }
};
