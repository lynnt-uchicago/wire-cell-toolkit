#include "WireCellAux/TensorDMcommon.h"
#include "WireCellAux/SimpleTensorSet.h"
#include "WireCellAux/SimpleTensor.h"
#include "WireCellUtil/Exceptions.h"
#include <regex>

using namespace WireCell;
using namespace WireCell::Aux;

    // Helper to access a set of ITensors
TensorDM::TensorIndex::TensorIndex()
{
}
TensorDM::TensorIndex::TensorIndex(const ITensor::vector& tens)
{
    add(tens);
}
void TensorDM::TensorIndex::add(const ITensor::vector& tens)
{
    for (const auto& ten : tens) {
        add(ten);
    }
}
void TensorDM::TensorIndex::add(const ITensor::pointer& ten)
{
    auto md = ten->metadata();
    const std::string dp = md["datapath"].asString();
    const std::string dt = md["datatype"].asString();

    const size_t ind = m_tens.size();
    m_path2ind[dp] = ind;
    if (m_type2ind.find(dt) == m_type2ind.end()) {
        m_type2ind[dt] = ind;
    }
    m_tens.push_back(ten);
}
ITensor::pointer TensorDM::TensorIndex::at_of(const std::string& datatype) const
{
    auto it = m_type2ind.find(datatype);
    if (it == m_type2ind.end()) {
        raise<KeyError>("no tensor of datatype \"%s\"", datatype);
    }
    const size_t ind = it->second;
    return m_tens.at(ind);
}

ITensor::pointer TensorDM::TensorIndex::at(const std::string& datapath) const
{
    auto it = m_path2ind.find(datapath);
    if (it == m_path2ind.end()) {
        raise<KeyError>("no tensor at datapath \"%s\"", datapath);
    }
    const size_t ind = it->second;
    return m_tens.at(ind);
}

ITensor::pointer TensorDM::TensorIndex::at(const std::string& datapath, const std::string& datatype) const
{
    if (datapath.empty()) {
        return at_of(datatype);
    }
    auto ten = at(datapath);
    if (ten->metadata()["datatype"].asString() != datatype) {
        raise<KeyError>("tensor at datapath \"%s\" is not of datatype \"%s\"", datapath, datatype);
    }
    return ten;
}

ITensor::pointer TensorDM::TensorIndex::get_of(const std::string& datatype, ITensor::pointer def) const
{
    auto it = m_type2ind.find(datatype);
    if (it == m_type2ind.end()) {
        return def;
    }
    const size_t ind = it->second;
    return m_tens.at(ind);
}
        
ITensor::pointer TensorDM::TensorIndex::get(const std::string& datapath, ITensor::pointer def) const
{
    auto it = m_path2ind.find(datapath);
    if (it == m_path2ind.end()) {
        return def;
    }
    const size_t ind = it->second;
    return m_tens.at(ind);
}

ITensor::pointer TensorDM::TensorIndex::get(const std::string& datapath, const std::string& datatype,
                                            ITensor::pointer def) const
{
    if (datapath.empty()) {
        return get_of(datatype, def);
    }

    auto ten = get(datapath, def);
    if (!ten) return def;
    if (ten->metadata()["datatype"].asString() != datatype) {
        return def;
    }
    return ten;
}




ITensor::pointer WireCell::Aux::TensorDM::make_metadata_tensor(
    const std::string& datatype,
    const std::string& datapath,
    Configuration metadata)
{
    metadata["datatype"] = datatype;
    metadata["datapath"] = datapath;
    return std::make_shared<Aux::SimpleTensor>(metadata);
}



ITensorSet::pointer
WireCell::Aux::TensorDM::as_tensorset(const ITensor::vector& tens,
                                      int ident,
                                      const Configuration& tsetmd)
{
    auto sv = std::make_shared<ITensor::vector>(tens.begin(), tens.end());
    return std::make_shared<SimpleTensorSet>(ident, tsetmd, sv);
}

