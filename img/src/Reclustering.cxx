#include "WireCellImg/Reclustering.h"

#include "WireCellAux/SimpleCluster.h"

#include "WireCellUtil/NamedFactory.h"
#include "WireCellUtil/GraphTools.h"

WIRECELL_FACTORY(Reclustering, WireCell::Img::Reclustering,
                 WireCell::INamed,
                 WireCell::IClusterFilter)

using namespace WireCell;
using namespace WireCell::GraphTools;
Img::Reclustering::Reclustering()
    : Aux::Logger("Reclustering", "img")
{
}

bool Img::Reclustering::operator()(const input_pointer& in, output_pointer& out)
{
    return true;
}

Img::Reclustering::~Reclustering()
{
}
