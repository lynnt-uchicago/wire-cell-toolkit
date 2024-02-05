/**
 *
 */

#ifndef WIRECELLIMG_POINTCLOUDFACADE
#define WIRECELLIMG_POINTCLOUDFACADE

#include "WireCellIface/IData.h"
#include "WireCellUtil/PointCloudDataset.h"
#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/Point.h"

namespace WireCell::PointCloud::Facade {
    using node_t = WireCell::PointCloud::Tree::Points::node_t;
    using node_ptr = std::unique_ptr<node_t>;
    using geo_point_t = WireCell::Point;
    using float_t = double;
    using int_t = int;

    class Blob : public IData<Blob> {
       public:
        Blob(const node_ptr& n);
        geo_point_t center_pos() const;
        bool overlap_fast(const Blob& b, const int offset) const;

        /// FIXME: cache all scalers?
        float_t charge {0};
        float_t center_x {0};
        float_t center_y {0};
        float_t center_z {0};
        int_t slice_index {0};

        int_t u_wire_index_min {0};
        int_t u_wire_index_max {0};
        int_t v_wire_index_min {0};
        int_t v_wire_index_max {0};
        int_t w_wire_index_min {0};
        int_t w_wire_index_max {0};

       private:
        node_t* m_node;  /// do not own
    };

    class Cluster : public IData<Cluster> {
       public:
        Cluster(const node_ptr& n);
        geo_point_t calc_ave_pos(const geo_point_t& origin, const double dis, const int alg = 0) const;
        Blob::vector is_connected(const Cluster& c, const int offset) const;

       private:
        node_t* m_node;  /// do not own
        std::unordered_multimap<int, Blob::pointer> m_time_blob_map;
        Blob::vector m_blobs;
    };
}  // namespace WireCell::PointCloud

#endif