#include "WireCellUtil/Logging.h"
#include "WireCellUtil/doctest.h"

#include <json/json.h>

#include <iostream> 
#include <vector>
#include <string>

using spdlog::debug;

//int main() {  
TEST_CASE("bee dead area") {
    std::vector<std::string> v = {
        "[[10, 10],[10, 100],[100, 100],[110, 60],[100,10]]",
        "[[48.9, 122.2],[49.1, 122.5],[53.5, 115.0],[53.3, 114.7]]"
    };

    Json::Value jsonArray(Json::arrayValue); // Create a JSON array

    for (auto& s : v) {
        Json::Value one;
        Json::Reader reader;
        reader.parse(s, one);
        jsonArray.append(one); // Add each parsed JSON object to the array
    }

    // Print the JSON array
    std::cout << jsonArray.toStyledString() << std::endl;
}
