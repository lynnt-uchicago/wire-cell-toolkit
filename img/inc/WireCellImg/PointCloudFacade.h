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

        bool overlap_fast(const Blob& b, const int offset) const;

        float_t charge () const { return  charge_; }
        float_t center_x () const { return  center_x_; }
        float_t center_y () const { return  center_y_; }
        float_t center_z () const { return  center_z_; }
	int_t npoints () const { return  npoints_; }
	
        // units are number of ticks
        int_t slice_index_min () const { return  slice_index_min_; }
        int_t slice_index_max () const { return  slice_index_max_; }

        int_t u_wire_index_min () const { return  u_wire_index_min_; }
        int_t u_wire_index_max () const { return  u_wire_index_max_; }
        int_t v_wire_index_min () const { return  v_wire_index_min_; }
        int_t v_wire_index_max () const { return  v_wire_index_max_; }
        int_t w_wire_index_min () const { return  w_wire_index_min_; }
        int_t w_wire_index_max () const { return  w_wire_index_max_; }


    private:

        float_t charge_ {0};
        float_t center_x_ {0};
        float_t center_y_ {0};
        float_t center_z_ {0};
	int_t npoints_ {0};
	
        int_t slice_index_min_ {0}; // unit: tick
        int_t slice_index_max_ {0};

        int_t u_wire_index_min_ {0};
        int_t u_wire_index_max_ {0};
        int_t v_wire_index_min_ {0};
        int_t v_wire_index_max_ {0};
        int_t w_wire_index_min_ {0};
        int_t w_wire_index_max_ {0};

    protected:

        // Receive notification when this facade is created on a node.
        virtual void on_construct(node_type* node);

    };
    std::ostream& operator<<(std::ostream& os, const Blob& blob);

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
	std::pair<geo_point_t, const Blob*> get_closest_point_blob(const geo_point_t& point) const;

        // Return set of blobs each with a corresponding point.  The set
        // includes blobs with at least one point within the given radius of the
        // given point.  The point is one in the blob and that is closest to the
        // given point.
        //
        // Note: radius must provide a LINEAR distance measure.
        using const_blob_point_map_t = std::map<const Blob*, geo_point_t>;
	const_blob_point_map_t get_closest_blob(const geo_point_t& point, double radius) const;

	std::pair<geo_point_t, double> get_closest_point_along_vec(
            geo_point_t& p_test, geo_point_t dir, double test_dis,
            double dis_step, double angle_cut, double dis_cut) const;

        // Return the number of points in the k-d tree
	int npoints() const;

        // Return the number of points within radius of the point.  Note, radius
        // is a LINEAR distance through the L2 metric is used internally.
	int nnearby(const geo_point_t& point, double radius) const;

        // Return the number of points in the k-d tree partitioned into pair
        // (#forward,#backward) based on given direction of view from the given
        // point.
	std::pair<int, int> ndipole(const geo_point_t& point, const geo_point_t& dir) const;

        // Return the number of points with in the radius of the given point in
        // the k-d tree partitioned into pair (#forward,#backward) based on
        // given direction of view from the given point.
        //
        // Note: the radius is a LINEAR distance measure.
	// std::pair<int, int> nprojection(const geo_point_t& point, const geo_point_t& dir, double radius) const;

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

	// Return blob at the front of the time blob map.  Raises ValueError if cluster is empty.
	const Blob* get_first_blob() const;

	// Return blob at the back of the time blob map.  Raises ValueError if cluster is empty.
	const Blob* get_last_blob() const;
	
    private:
        // start slice index (tick number) to blob facade pointer can be
        // duplicated, example usage:
        // https://github.com/HaiwangYu/learn-cpp/blob/main/test-multimap.cxx

	using time_blob_map_t = std::multimap<int, const Blob*>;
        const time_blob_map_t& time_blob_map() const;
	mutable time_blob_map_t m_time_blob_map; // lazy, do not access directly.

        // Cached and lazily calculated in get_length().
        // Getting a new node invalidates by setting to 0.
        mutable double m_length{0};
        // Cached and lazily calculated in npoints()
        mutable int m_npoints{0};

        // Return the number of unique wires or ticks.  FIXME: is this really
        // what is wanted?  It does not return what is normally considered a
        // "range".
        std::tuple<int, int, int, int> get_uvwt_range() const;
    };
    std::ostream& operator<<(std::ostream& os, const Cluster& cluster);


    // Give a node "Grouping" semantics.  A grouping node's children are cluster
    // nodes that are related in some way.
    class Grouping : public NaryTree::FacadeParent<Cluster, points_t> {
    public:

    };
    std::ostream& operator<<(std::ostream& os, const Grouping& grouping);

    // Dump the grouping to a string eg for debugging.  Level 0 dumps info about
    // just the grouping, 1 includes info about the clusters and 2 includes info
    // about the blobs.
    std::string dump(const Grouping& grouping, int level=0);

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
