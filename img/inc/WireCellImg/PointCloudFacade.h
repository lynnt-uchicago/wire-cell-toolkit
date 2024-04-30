/** A facade over a PC tree giving semantics to otherwise nodes.
 *
 */

#ifndef WIRECELLIMG_POINTCLOUDFACADE
#define WIRECELLIMG_POINTCLOUDFACADE

#include "WireCellUtil/PointCloudDataset.h"
#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/Point.h"
#include "WireCellUtil/Units.h"

// using namespace WireCell;  NO!  do not open up namespaces in header files!

namespace WireCell::PointCloud::Facade {
    using points_t = Tree::Points;
    using node_t = Tree::Points::node_t;
    using node_ptr = std::unique_ptr<node_t>;
    using geo_point_t = WireCell::Point;
    using geo_vector_t = WireCell::Vector;

    // FIXME: why define these out?
    using float_t = double;
    using int_t = int;

    // FIXME: refactor to vector<pitch>, etc?  or vector<TPCPlane> with ::pitch/::angle?
    struct TPCParams {
        float_t pitch_u {3*units::mm};
        float_t pitch_v {3*units::mm};
        float_t pitch_w {3*units::mm};
        float_t angle_u {1.0472};  // 60 degrees    uboone geometry ...
        float_t angle_v {-1.0472};  //-60 degrees   uboone geometry ...
        float_t angle_w {0};        // 0 degrees    uboone geometry ...
        float_t tick_drift {0.5*1.101*units::mm}; // tick * speed
    };


    /// Give a node "Blob" semantics
    class Blob : public NaryTree::Facade<points_t> {
    public:
        
        Blob() = default;
        virtual ~Blob() {}
            

        geo_point_t center_pos() const;
	int_t num_points() const;
        bool overlap_fast(const Blob& b, const int offset) const;

        // fixme: make data private.  factor into collections.
        float_t charge {0};
        float_t center_x {0};
        float_t center_y {0};
        float_t center_z {0};
	int_t npoints {0};
	
        int_t slice_index_min {0}; // unit: tick
        int_t slice_index_max {0};

        int_t u_wire_index_min {0};
        int_t u_wire_index_max {0};
        int_t v_wire_index_min {0};
        int_t v_wire_index_max {0};
        int_t w_wire_index_min {0};
        int_t w_wire_index_max {0};

    protected:
        virtual void notify_new_node_base();
    };

    // Give a node "Cluster" semantics.  A cluster node's children are blob nodes.
    class Cluster : public NaryTree::FacadeParent<Blob, points_t> {

        // The expected scope.
        const WireCell::PointCloud::Tree::Scope scope = { "3d", {"x","y","z"} };

    public:

        Cluster() = default;
        virtual ~Cluster() {}

        // Return charge-weighted average position of points of blobs withing distance of point.
        geo_point_t calc_ave_pos(const geo_point_t& origin, const double dis, const int alg = 0) const;

        // Return blob containing the returned point that is closest to the given point.
        using point_blob_map_t = std::map<geo_point_t, const Blob*>;
	std::pair<geo_point_t, const Blob*> get_closest_point_mcell(const geo_point_t& point) const;

        // Return set of blobs each with a corresponding point.  The set
        // includes blobs with at least one point within the given radius of the
        // given point.  The point is one in the blob and that is closest to the
        // given point.
        //
        // Note: radius must provide a LINEAR distance measure.
        using const_blob_point_map_t = std::map<const Blob*, geo_point_t>;
	const_blob_point_map_t get_closest_mcell(const geo_point_t& point, double radius) const;

	std::pair<geo_point_t, double> get_closest_point_along_vec(
            geo_point_t& p_test, geo_point_t dir, double test_dis,
            double dis_step, double angle_cut, double dis_cut) const;

        // Return the number of points in the k-d tree
	int get_num_points() const;

        // Return the number of points within radius of the point.  Note, radius
        // is a LINEAR distance through the L2 metric is used internally.
	int get_num_points(const geo_point_t& point, double radius) const;

        // Return the number of points in the k-d tree partitioned into pair
        // (#forward,#backward) based on given direction of view from the given
        // point.
	std::pair<int, int> get_num_points(const geo_point_t& point, const geo_point_t& dir) const;

        // Return the number of points with in the radius of the given point in
        // the k-d tree partitioned into pair (#forward,#backward) based on
        // given direction of view from the given point.
        //
        // Note: the radius is a LINEAR distance measure.
	std::pair<int, int> get_num_points(const geo_point_t& point, const geo_point_t& dir, double radius) const;

        // Return the points at the extremes of the X-axis.
        //
        // Note: the two points are in ASCENDING order!
	std::pair<geo_point_t, geo_point_t> get_earliest_latest_points() const;

        // Return the points at the extremes of the given Cartesian axis.  Default is Y-axis.
        //
        // Note: the two points are in DESCENDING order!
	std::pair<geo_point_t, geo_point_t> get_highest_lowest_points(size_t axis=1) const;
	
	
        std::vector<const Blob*> is_connected(const Cluster& c, const int offset) const;

        // The Hough-based direction finder works in a 2D parameter space.  The
        // first dimension is associated with theta (angle w.r.t. Z-axis) and
        // can use an angle or a cosine measure.  Theta angle measure has a
        // non-uniform metric space, especially near the poles.  Perhaps this
        // bias is useful.  The cosine(theta) metric space is uniform.
        enum HoughParamSpace {
            costh_phi,      // (cos(theta), phi)
            theta_phi       // (theta, phi)
        };

        // Return parameter values characterizing the points within radius of
        // given point.
        //
        // Note: radius must provide a LINEAR distance measure.
        std::pair<double, double> hough_transform(
            const geo_point_t& point, const double radius,
            HoughParamSpace param_space = HoughParamSpace::theta_phi) const;

        // Call hough_transform() and transform result as to a directional vector representation.
        //
        // Note: radius must provide a LINEAR distance measure.
        geo_vector_t vhough_transform(const geo_point_t& point, const double radius,
                                      HoughParamSpace param_space = HoughParamSpace::theta_phi) const;

        // Return a quasi geometric size of the cluster based on its transverse
        // extents in each view and in time.
        double get_length(const TPCParams& tp) const;

	// Return blob at the front of the time blob map.
	const Blob* get_first_blob() const;

	// Return blob at the back of the time blob map.
	const Blob* get_last_blob() const;
	
    protected:
        virtual void notify_new_node_parent();

    private:
        // start slice index (tick number) to blob facade pointer can be
        // duplicated, example usage:
        // https://github.com/HaiwangYu/learn-cpp/blob/main/test-multimap.cxx
	std::multimap<int, const Blob*> m_time_blob_map;

        // Cached and lazily calculated in get_length().
        // Getting a new node invalidates by setting to 0.
        mutable double m_length{0};

        // Return the number of unique wires or ticks.  FIXME: is this really
        // what is wanted?  It does not return what is normally considered a
        // "range".
        std::tuple<int, int, int, int> get_uvwt_range() const;
    };


    // Give a node "Grouping" semantics.  A grouping node's children are cluster
    // nodes that are related in some way.
    class Grouping : public NaryTree::FacadeParent<Cluster, points_t> {
    public:

    };


    // fixme: why do we inline these?
    inline double cal_proj_angle_diff(const geo_vector_t& dir1, const geo_vector_t& dir2, double plane_angle) {
        geo_vector_t temp_dir1;
        geo_vector_t temp_dir2;

        temp_dir1.set(dir1.x(), 0, -sin(plane_angle) * dir1.y() + cos(plane_angle) * dir1.z());
        temp_dir2.set(dir2.x(), 0, -sin(plane_angle) * dir2.y() + cos(plane_angle) * dir2.z());

        return temp_dir1.angle(temp_dir2);
    }

    inline bool is_angle_consistent(const geo_vector_t& dir1, const geo_vector_t& dir2, bool same_direction, double angle_cut, double uplane_angle, double vplane_angle, double wplane_angle, int num_cut=2) {
        double angle_u = cal_proj_angle_diff(dir1, dir2, uplane_angle);
        double angle_v = cal_proj_angle_diff(dir1, dir2, vplane_angle);
        double angle_w = cal_proj_angle_diff(dir1, dir2, wplane_angle);
        int num = 0;
        // input is degrees ...
        angle_cut *= 3.1415926 / 180.;

        if (same_direction) {
            if (angle_u <= angle_cut) num++;
            if (angle_v <= angle_cut) num++;
            if (angle_w <= angle_cut) num++;
        } else {
            if ((3.1415926 - angle_u) <= angle_cut) num++;
            if ((3.1415926 - angle_v) <= angle_cut) num++;
            if ((3.1415926 - angle_w) <= angle_cut) num++;
        }

        if (num >= num_cut) return true;
        return false;
    }

}  // namespace WireCell::PointCloud

#endif
