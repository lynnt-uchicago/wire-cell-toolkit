
#include "WireCellImg/DeadLiveMerging.h"
#include "WireCellUtil/NamedFactory.h"

WIRECELL_FACTORY(DeadLiveMerging, WireCell::Img::DeadLiveMerging,
                 WireCell::INamed,
                 WireCell::IClusterFanin,
                 WireCell::IConfigurable)

using namespace WireCell;
using namespace WireCell::Img;

DeadLiveMerging::DeadLiveMerging()
    : Aux::Logger("DeadLiveMerging", "img")
{
}

void DeadLiveMerging::configure(const WireCell::Configuration& cfg)
{
    int m = get<int>(cfg, "multiplicity", (int) m_multiplicity);
    if (m <= 0) {
        raise<ValueError>("DeadLiveMerging multiplicity must be > 0");
    }
    m_multiplicity = m;

    m_tags.resize(m);

    // Tag entire input frame worth of traces in the output frame.
    auto jtags = cfg["tags"];
    for (int ind = 0; ind < m; ++ind) {
        m_tags[ind] = convert<std::string>(jtags[ind], "");
    }
}

WireCell::Configuration DeadLiveMerging::default_configuration() const
{
    Configuration cfg;
    return cfg;
}

std::vector<std::string> WireCell::Img::DeadLiveMerging::input_types()
{
    const std::string tname = std::string(typeid(input_type).name());
    std::vector<std::string> ret(m_multiplicity, tname);
    return ret;
}

bool DeadLiveMerging::operator()(const input_vector& in, output_pointer& out)
{
    out = nullptr;
    if (in.empty()) {
        return true;
    }
    // dummy implementation
    out = in.front();
    return true;
}