#ifndef WIRECELLUTIL_TESTING
#define WIRECELLUTIL_TESTING

#include "WireCellUtil/Configuration.h"

#define BOOST_ENABLE_ASSERT_HANDLER 1
#include <boost/assert.hpp>

#define Assert BOOST_ASSERT
#define AssertMsg BOOST_ASSERT_MSG

#include <string>
#include <vector>

namespace boost {
    void assertion_failed(char const* expr, char const* function, char const* file, long line);
    void assertion_failed_msg(char const* expr, char const* msg, char const* function, char const* file, long line);
}  // namespace boost

namespace WireCell::Testing {

    // Add a stderr and file log sink based on the argv[0] name.
    // Bare calls, eg, spdlog::debug() may then be issued.
    void log(const char* argv0);

    // Load plugins.  If empty, load "core" plugins.
    void load_plugins(std::vector<std::string> list = {});

    // Return the known detectors object from a given source.
    Configuration detectors(const std::string& source="detectors.jsonnet");

}  // namespace WireCell

#endif
