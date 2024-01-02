/**
 *
 */

#ifndef WIRECELLIMG_POINTCLOUDFACADE
#define WIRECELLIMG_POINTCLOUDFACADE

#include "WireCellUtil/PointCloudDataset.h"
#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/Point.h"

namespace WireCell::PointCloud {
    using node_t = WireCell::PointCloud::Tree::Points::node_t;
    using node_ptr = std::unique_ptr<node_t>;
    using Point = WireCell::Point;

    class Cluster {
       public:
        Cluster(const node_ptr& n)
          : m_node(n.get())
        {
        }
        WireCell::PointCloud::Point calc_ave_pos(const Point& origin, const double dis) const;

       private:
        node_t* m_node;  /// do not own
    };

    class Blob {
       public:
        Blob(const node_ptr& n)
          : m_node(n.get())
        {
        }
        double center_pos() const;

       private:
        node_t* m_node;  /// do not own
    };
}  // namespace WireCell::PointCloud

#endif