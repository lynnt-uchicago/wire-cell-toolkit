/**
 *
 */

#ifndef WIRECELLIMG_POINTCLOUDFACADE
#define WIRECELLIMG_POINTCLOUDFACADE

#include "WireCellIface/IData.h"
#include "WireCellUtil/PointCloudDataset.h"
#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/Point.h"
#include "WireCellUtil/Units.h"

using namespace WireCell;

namespace WireCell::PointCloud::Facade {
    using node_t = WireCell::PointCloud::Tree::Points::node_t;
    using node_ptr = std::unique_ptr<node_t>;
    using geo_point_t = WireCell::Point;
    using float_t = double;
    using int_t = int;

    struct TPCParams {
        float_t pitch_u {3*units::mm};
        float_t pitch_v {3*units::mm};
        float_t pitch_w {3*units::mm};
        // float_t angle_u {60};
        // float_t angle_v {60};
        // float_t angle_w {90};
        float_t ts_width {3.2*units::mm}; // time slice width 2 us * 1.6 mm/us ~ 3.2 mm
    };

    class Blob : public IData<Blob> {
       public:
        Blob(const node_ptr& n);
        node_t* m_node;  /// do not own

        geo_point_t center_pos() const;
        bool overlap_fast(const Blob& b, const int offset) const;

        /// FIXME: cache all scalers?
        float_t charge {0};
        float_t center_x {0};
        float_t center_y {0};
        float_t center_z {0};
        int_t slice_index_min {0};
        int_t slice_index_max {0};

        int_t u_wire_index_min {0};
        int_t u_wire_index_max {0};
        int_t v_wire_index_min {0};
        int_t v_wire_index_max {0};
        int_t w_wire_index_min {0};
        int_t w_wire_index_max {0};


       private:
    };

    class Cluster : public IData<Cluster> {
       public:
        Cluster(const node_ptr& n);
        node_t* m_node;  /// do not own
        Blob::vector m_blobs;

        geo_point_t calc_ave_pos(const geo_point_t& origin, const double dis, const int alg = 0) const;
        Blob::vector is_connected(const Cluster& c, const int offset) const;
        // alg 0: cos(theta), 1: theta
        std::pair<double, double> hough_transform(const geo_point_t& origin, const double dis, const int alg = 0) const;
        geo_point_t vhough_transform(const geo_point_t& origin, const double dis, const int alg = 0) const;

        // get the number of unique uvwt bins
        std::tuple<int, int, int, int> get_uvwt_range() const;
        double get_length(const TPCParams& tp) const;

       private:
        std::unordered_multimap<int, Blob::pointer> m_time_blob_map;
    };
}  // namespace WireCell::PointCloud

#endif