/** Sample blobs to make point cloud datasets as tensor sets.

 The heaving lifting is done via an IBlobSamper.
*/
#ifndef WIRECELL_IMG_BLOBSAMPLING
#define WIRECELL_IMG_BLOBSAMPLING

#include "WireCellIface/IBlobSampling.h"
#include "WireCellIface/IBlobSampler.h"
#include "WireCellIface/IConfigurable.h"
#include "WireCellAux/Logger.h"


namespace WireCell::Img {

    class BlobSampling : public Aux::Logger, public IBlobSampling, public IConfigurable
    {
      public:
        BlobSampling();
        virtual ~BlobSampling();

        // IConfigurable
        virtual void configure(const WireCell::Configuration& cfg);
        virtual WireCell::Configuration default_configuration() const;

        virtual bool operator()(const input_pointer& blobset, output_pointer& tensorset);

      private:
        
        IBlobSampler::pointer m_sampler;
        size_t m_count{0};
    };
}

#endif