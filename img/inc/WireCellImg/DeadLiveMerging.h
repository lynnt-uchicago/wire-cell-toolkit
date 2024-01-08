#pragma once

#include "WireCellIface/IClusterFanin.h"
#include "WireCellIface/IConfigurable.h"
#include "WireCellAux/Logger.h"

namespace WireCell {
    namespace Img {
        class DeadLiveMerging : public IClusterFanin, public IConfigurable, public Aux::Logger
        {
        public:
            DeadLiveMerging();
            virtual ~DeadLiveMerging() = default;

            // IConfigurable
            virtual void configure(const WireCell::Configuration& cfg);
            virtual WireCell::Configuration default_configuration() const;

            virtual std::vector<std::string> input_types();

            // IClusterFanin
            virtual bool operator()(const input_vector& in, output_pointer& out);
        private:
            size_t m_multiplicity{2};
            std::vector<std::string> m_tags;

            int m_count{0};
        };
    }
}