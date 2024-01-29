/**
 *
 */

#ifndef WIRECELLIMG_POINTCLOUDFACADE
#define WIRECELLIMG_POINTCLOUDFACADE

#include "WireCellUtil/PointCloudDataset.h"
#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/Point.h"

namespace WireCell::PointCloud::Facade {
    using node_t = WireCell::PointCloud::Tree::Points::node_t;
    using node_ptr = std::unique_ptr<node_t>;
    using geo_point_t = WireCell::Point;
    using float_t = double;

    class Blob {
       public:
        Blob(const node_ptr& n);
        geo_point_t center_pos() const;

        /// FIXME: cache all scalers?
        float_t charge {0};
        float_t center_x {0};
        float_t center_y {0};
        float_t center_z {0};
        int slice_index {0};
        // [u, v, w], [min, max]
        int wire_index_ranges[3][2];

       private:
        node_t* m_node;  /// do not own
    };

    class Cluster {
       public:
        Cluster(const node_ptr& n);
        geo_point_t calc_ave_pos(const geo_point_t& origin, const double dis, const int alg = 0) const;

       private:
        node_t* m_node;  /// do not own
        std::unordered_multimap<int, std::shared_ptr<Blob>> m_time_blob_map;
        std::vector<std::shared_ptr<Blob>> m_blobs;
    };
}  // namespace WireCell::PointCloud

#endif