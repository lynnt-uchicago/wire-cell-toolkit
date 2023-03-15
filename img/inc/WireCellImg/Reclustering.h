/** Recluster

    This component will consume an ICluster and produce an ICluster
    which represents a "reclustering" of the input.

    The input ICluster graph is assumed to composed of vertices
    potentially produced by disparate means.  For example, multiple
    BlobSets may have been produced independently, each sent through
    BlobClustering, subsequently independent cluster processing and
    finally brought together with a ClusterFanin.

    The result will have the following operations applied.

    - Where a channel or a wire is represented by multiple vertices,
      those vertices will be replaced with a single vertex and any
      edges to the removed vertices remade to this new single vertex.

    - New blob-blob edges are constructed in a manner similar to
      BlobClustering with the exception that the slices of blobs are
      not assumed to be shared nor of common span.  Blob pairs are
      considered connected if the same transverse condition applied in
      BlobClustering holds and if the pairs exist in different slices
      with overlapping or touching time spans.

*/
#ifndef WIRECELLIMG_RECLUSTERING
#define WIRECELLIMG_RECLUSTERING

#include "WireCellIface/IClusterFilter.h"
#include "WireCellAux/Logger.h"

namespace WireCell::Img {

     class Reclustering : public Aux::Logger, public IClusterFilter {
      public:
        Reclustering();
        virtual ~Reclustering();

        virtual bool operator()(const input_pointer& in, output_pointer& out);

      private:
        int m_count{0};
    };
}

#endif
