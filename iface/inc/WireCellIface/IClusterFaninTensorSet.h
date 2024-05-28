#ifndef WIRECELL_IFACE_ICLUSTERFANINTENSORSET
#define WIRECELL_IFACE_ICLUSTERFANINTENSORSET

#include "WireCellIface/IFaninNode.h"
#include "WireCellIface/ICluster.h"
#include "WireCellIface/ITensorSet.h"

#include <string>

namespace WireCell {
    /*
     * Fanin IClusters, output ITensorSet
     */
    class IClusterFaninTensorSet : public IFaninNode<ICluster, ITensorSet, 0> {
       public:
        virtual ~IClusterFaninTensorSet();

        virtual std::string signature() { return typeid(IClusterFaninTensorSet).name(); }

        // Subclass must implement:
        virtual std::vector<std::string> input_types() = 0;
        // and the already abstract:
        // virtual bool operator()(const input_pointer& in, output_vector& outv);
    };
}  // namespace WireCell

#endif