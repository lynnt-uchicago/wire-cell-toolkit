#include "WireCellUtil/PointCloudCoordinates.h"
#include "WireCellUtil/PointCloudDataset.h"
#include "WireCellUtil/Logging.h"
#include "WireCellUtil/Type.h"
#include "WireCellUtil/doctest.h"

#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/range.hpp>

#include <vector>

using spdlog::debug;
using namespace WireCell;
using namespace WireCell::PointCloud;

// Test coordinate_array in mutable and const version
template<typename DS>
void test_ca(DS& ds)
{
    auto sel = ds.selection({"x","y","z"});
    coordinate_array ca(sel);

    REQUIRE(ca.size() == 3);
    for (size_t ind=0; ind<3; ++ind) {
        REQUIRE(ca[ind].size() == 3);
    }
    {
        const auto& p = ca[0];
        CHECK(p[0] == 1.0);
        CHECK(p[1] == 2.0);
        CHECK(p[2] == 1.0);
        CHECK(p.at(0) == 1.0);
        CHECK(p.at(1) == 2.0);
        CHECK(p.at(2) == 1.0);
        auto pit = ca.begin() + 0;
        CHECK(pit->at(0) == 1.0);
        CHECK(pit->at(1) == 2.0);
        CHECK(pit->at(2) == 1.0);
    }
    {
        const auto& p = ca[1];
        CHECK(p[0] == 1.0);
        CHECK(p[1] == 1.0);
        CHECK(p[2] == 4.0);
        CHECK(p.at(0) == 1.0);
        CHECK(p.at(1) == 1.0);
        CHECK(p.at(2) == 4.0);
        auto pit = ca.begin();
        ++pit;
        CHECK(pit->at(0) == 1.0);
        CHECK(pit->at(1) == 1.0);
        CHECK(pit->at(2) == 4.0);
    }
    {
        const auto& p = ca[2];
        CHECK(p[0] == 1.0);
        CHECK(p[1] == 3.0);
        CHECK(p[2] == 1.0);
        CHECK(p.at(0) == 1.0);
        CHECK(p.at(1) == 3.0);
        CHECK(p.at(2) == 1.0);
        auto pit = ca.end();
        --pit;
        CHECK(pit->at(0) == 1.0);
        CHECK(pit->at(1) == 3.0);
        CHECK(pit->at(2) == 1.0);
    }
    for (const auto& p : ca) {
        debug("point cloud coordinate array: p=({},{},{})",
              p[0], p[1], p[2]);
    }
}

TEST_CASE("point cloud coordinate array")
{
    Dataset ds({
            {"x", Array({1.0, 1.0, 1.0})},
            {"y", Array({2.0, 1.0, 3.0})},
            {"z", Array({1.0, 4.0, 1.0})},
            {"one", Array({1  ,2  ,3  })},
            {"two", Array({1.1,2.2,3.3})}});

    SUBCASE("mutable point index") {
        test_ca(ds);
    }
    SUBCASE("const point index") {
        test_ca<Dataset const>(ds);
    }

}
