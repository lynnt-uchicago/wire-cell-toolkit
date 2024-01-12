/** Sample blobs to make point cloud tree and output as tensors.
 * Same as PointTreeBuilding but use ICluster as input.
*/
#ifndef WIRECELL_IMG_POINTTREEBUILDING
#define WIRECELL_IMG_POINTTREEBUILDING

#include "WireCellIface/IClusterFaninTensorSet.h"
#include "WireCellIface/IBlobSampler.h"
#include "WireCellIface/IConfigurable.h"
#include "WireCellAux/Logger.h"


namespace WireCell::Img {

    class PointTreeBuilding : public Aux::Logger, public IClusterFaninTensorSet, public IConfigurable
    {
      public:
        PointTreeBuilding();
        virtual ~PointTreeBuilding();

        // INode, override because we get multiplicity at run time.
        virtual std::vector<std::string> input_types();

        // IConfigurable
        virtual void configure(const WireCell::Configuration& cfg);
        virtual WireCell::Configuration default_configuration() const;

        virtual bool operator()(const input_vector& invec, output_pointer& tensorset);

      private:
        size_t m_multiplicity {2};
        std::vector<std::string> m_tags;
        size_t m_count{0};
        
        /** Configuration: "samplers"

            An object with attributes providing names of
            IBlobSamplers.  The attribute names will be used to name
            the point cloud produced by the samplers.
        */
        std::map<std::string, IBlobSampler::pointer> m_samplers;

        /** Config: "datapath"

            Set the datapath for the tensor representing the point
            cloud tree.  If a %d format code is found it wil be
            interpolated with the IBlobSet::ident() value.
         */
        std::string m_datapath = "pointtrees/%d";

    };
}

#endif
