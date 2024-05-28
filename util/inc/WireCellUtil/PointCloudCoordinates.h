#ifndef WIRECELLUTIL_POINTCLOUDCOORDINATES
#define WIRECELLUTIL_POINTCLOUDCOORDINATES

#include "WireCellUtil/PointCloudDataset.h"
#include "WireCellUtil/Logging.h"

#include <boost/iterator/iterator_adaptor.hpp>
#include <memory>

namespace WireCell::PointCloud {

    /**
       A transposed view of a const selection 
     */
    template<typename ElementType=double, typename Selection=Dataset::selection_t>
    class coordinate_array {
      public:
        class point {
            coordinate_array* cap;
          public:
            using value_type = ElementType;

            point() = delete;
            ~point() {
                SPDLOG_TRACE("coordinate_array::point dtor this: {} cap: {} data: {} index={} size={}",
                              (void*)this, (void*)cap, (void*)cap->data(), this - cap->data(), cap->size());
                cap = nullptr;
            }
            explicit point(coordinate_array* cap) : cap(cap) {
                SPDLOG_TRACE("coordinate_array::point ctor this: {} cap: {} data: {} index={} size={}",
                              (void*)this, (void*)cap, (void*)cap->data(), this - cap->data(), cap->size());
            }
            point(const point& o) : cap(o.cap) {
                SPDLOG_TRACE("coordinate_array::point copy this: {} cap: {} data: {} index={} size={}",
                              (void*)this, (void*)cap, (void*)cap->data(), this - cap->data(), cap->size());
            }

            // The number of dimensions of the point.
            size_t size() const {
                return cap->ndims();
            }
            bool empty() const {
                return 0 == size();
            }

            // Return value at dimension, no bounds check.
            const value_type& operator[](size_t dim) const {
                const size_t ind = cap->index(this);
                return (*cap)(dim, ind);
            }
            value_type& operator[](size_t dim) {
                const size_t ind = cap->index(this);
                return (*cap)(dim, ind);
            }

            // Return value at dimension, with bounds check.
            const value_type& at(size_t dim) const {
                const size_t ind = cap->index(this);
                return cap->at(dim, ind);
            }
            value_type& at(size_t dim) {
                const size_t ind = cap->index(this);
                return cap->at(dim, ind);
            }

        };

        using value_type = point;
        using element_type = ElementType;
        using selection_t = Selection;


        coordinate_array() = delete;
        /// Construct a coordinate array, hold reference to selection.
        explicit coordinate_array(Selection& sel)
            : sel(sel)
            , pts(sel[0]->size_major(), point(this)) // blows up if sel is empty!
        {
            SPDLOG_TRACE("coordinate_array ctor cap: {} data: {} size={}", (void*)this, (void*)data(), size());
        }

        ~coordinate_array() {
            SPDLOG_TRACE("coordinate_array dtor cap: {} data: {} size={}", (void*)this, (void*)data(), size());
        }
        coordinate_array(const coordinate_array& o)
            : sel(o.sel)
            , pts(sel[0]->size_major(), point(this))
        {
            SPDLOG_TRACE("coordinate_array copy cap: {} data: {} size={}", (void*)this, (void*)data(), size());
        }

        // Number of dimensions
        size_t ndims() const {
            return sel.size();
        }

        // Number of points
        size_t size() const {
            return pts.size();
        }
        bool empty() const {
            return pts.empty();
        }

        point& operator[](size_t ind) {
            return pts[ind];
        }
        const point& operator[](size_t ind) const {
            return pts[ind];
        }
        point& at(size_t ind) {
            return pts.at(ind);
        }
        const point& at(size_t ind) const {
            return pts.at(ind);
        }
        const point* data() const {
            return pts.data();
        }

        using point_array = std::vector<point>;
        using iterator = typename point_array::iterator;
        using const_iterator = typename point_array::const_iterator;

        iterator begin() { return pts.begin(); }
        iterator end() { return pts.end(); }
        const_iterator begin() const { return pts.begin(); }
        const_iterator end() const { return pts.end(); }

        const element_type& operator()(size_t dim, size_t ind) const {
            std::shared_ptr<Array const> aptr = sel[dim];
            return aptr->element<ElementType>(ind);
        }
        element_type& operator()(size_t dim, size_t ind) {
            std::shared_ptr<Array> aptr = sel[dim];
            return aptr->element<ElementType>(ind);
        }

        const element_type& at(size_t dim, size_t ind) const {
            std::shared_ptr<Array const> aptr = sel.at(dim);
            if (ind >= aptr->size_major()) {
                throw std::out_of_range("coordinate point index out of range");
            }
            return aptr->element<ElementType>(ind);
        }
        element_type& at(size_t dim, size_t ind) {
            std::shared_ptr<Array> aptr = sel.at(dim);
            if (ind >= aptr->size_major()) {
                throw std::out_of_range("coordinate point index out of range");
            }
            return aptr->element<ElementType>(ind);
        }

      private:
        Selection& sel;
        point_array pts;

      private:
        friend class point;

        // Convert a point to its index
        size_t index(const point* pt) const {
            const size_t ind = pt - pts.data();
            if (ind < size()  && &pts[ind] == pt) {
                return ind;
            }
            throw std::logic_error("C++ does not work the way you think");
        }


    };

    /**
       A coordinate point provides vector-like access to a "slice" (a point)
       across the arrays of a selection of a dataset at a given index along the
       major axis.
    */
    template<typename ElementType=double>
    class coordinate_point {
      public:
        using value_type = ElementType;

        using selection_t = Dataset::selection_t;

        coordinate_point() 
            : selptr(nullptr), index_(0)
        {
            SPDLOG_TRACE("coordinate_point ctor default this: {} selptr={} {}", (void*)this, (void*)selptr, index_);
        }
        explicit coordinate_point(selection_t* selptr, size_t ind=0)
            : selptr(selptr), index_(ind)
        {
            SPDLOG_TRACE("coordinate_point ctor pointer this: {} selptr={} {}", (void*)this, (void*)selptr, index_);
        }
        explicit coordinate_point(selection_t& sel, size_t ind=0)
            : selptr(&sel), index_(ind)
        {
            SPDLOG_TRACE("coordinate_point ctor reference this: {} selptr={} {}", (void*)this, (void*)selptr, index_);
        }

        // copy ctor
        coordinate_point(const coordinate_point& o)
            : selptr(o.selptr), index_(o.index_)
        {
            SPDLOG_TRACE("coordinate_point ctor copy other: {} -> this: {} selptr={} {}", (void*)&o, (void*)this, (void*)selptr, index_);
        }

        // assignment
        coordinate_point& operator=(const coordinate_point& o)
        {
            selptr = o.selptr;
            index_ = o.index_;
            SPDLOG_TRACE("coordinate_point assignment other: {} -> this: {} selptr={} {}", (void*)&o, (void*)this, (void*)selptr, index_);
            return *this;
        }

        ~coordinate_point()
        {
            SPDLOG_TRACE("coordinate_point dtor this: {} selptr={} {}", (void*)this, (void*)selptr, index_);
            selptr = nullptr;
            index_ = 0xdeadbeaf;
        }

        // number of dimensions of the point.
        size_t size() const {
            if (selptr) {
                return selptr->size();
            }
            return 0;
        }

        // no bounds checking
        value_type operator[](size_t dim) const {
            auto arr = (*selptr)[dim];
            return arr->element<ElementType>(index_);
        }

        void assure_valid(size_t dim) const {
            if (selptr==nullptr) {
                throw std::out_of_range("coordinate point has no selection");
            }
            const size_t ndims = selptr->size();
            if (!ndims) {
                throw std::out_of_range("coordinate point has empty selection");
            }   
            if (dim >= ndims) {
                throw std::out_of_range("coordinate point dimension out of range");
            }
            if (index_ >= (*selptr)[0]->size_major()) {
                throw std::out_of_range("coordinate point index out of range");
            }
        }

        // size_t npoints() const
        // {
        //     if (!selptr || selptr->empty() || (*selptr)[0].empty()) return 0;
        //     return (*selptr)[0]->size_major();
        // }

        value_type at(size_t dim) const {
            assure_valid(dim);
            return (*this)[dim];
        }

        void set_index(size_t ind)
        {
            SPDLOG_TRACE("coordinate_point::set_index {} -> {}", index_, ind);
            index_ = ind;
        }
        size_t index() const { return index_; }

        bool operator==(const coordinate_point& other) const {
            return index_ == other.index_ && selptr == other.selptr;
        }

      private:
        selection_t* selptr{nullptr};
        size_t index_{0};
    };
    using real_coordinate_point = coordinate_point<double>;

    /**
       An iterator over a coordinate range.
     */
    template<typename PointType = real_coordinate_point>
    class coordinate_iterator
        : public boost::iterator_facade<coordinate_iterator <PointType>
                                        , PointType
                                        , boost::random_access_traversal_tag>
    {
      public:
        using self_type = coordinate_iterator<PointType>;
        using base_type = boost::iterator_facade<self_type
                                                 , PointType
                                                 , boost::random_access_traversal_tag>;
        using difference_type = typename base_type::difference_type;
        using value_type = typename base_type::value_type;
        using pointer = typename base_type::pointer;
        using reference = typename base_type::reference;

        using selection_t = typename PointType::selection_t;

        coordinate_iterator(selection_t* sel=nullptr, size_t ind=0)
            : point(sel, ind)
        {
        }

        coordinate_iterator(const coordinate_iterator& o)
            : point(o.point)
        {            
        }

        // copy ctor
        template<typename OtherIter>
        coordinate_iterator(OtherIter o)
            : point(o.point)
        {
        }

        // assignment
        coordinate_iterator& operator=(const coordinate_iterator& o)
        {
            point = o.point;
            return *this;
        }

      private:
        mutable value_type point;

      private:
        friend class boost::iterator_core_access;

        bool equal(const coordinate_iterator& o) const {
            return point == o.point;
        }

        void increment () {
            point.set_index(point.index() + 1);
        }
        void decrement () {
            point.set_index(point.index() - 1);
        }
        void advance (difference_type n) {
            if (n == 0) { return; }
            size_t index = point.index();
            SPDLOG_TRACE("coordinate_iterator::advance {} + {}", index, n);
            point.set_index(index + n);
        }

        difference_type
        distance_to(self_type const& other) const
        {
            return other.point.index() - this->point.index();
        }

        reference dereference() const
        {
            return point;
        }
    };


    /**
       A transpose of a point cloud selection.

     */
    template<typename PointType = real_coordinate_point>
    class coordinate_range {
      public:
        using point_type = PointType;
        using element_type = typename PointType::value_type;
        using iterator = coordinate_iterator<PointType>;
        using const_iterator = coordinate_iterator<PointType const>;

        using selection_t = typename PointType::selection_t;

        coordinate_range() : selptr(nullptr){ }
        ~coordinate_range() { }

        // copy ctor
        template<typename OtherCoordRange>
        coordinate_range(OtherCoordRange& o) : selptr(o.selptr) { }

        // assigmnet
        template<typename OtherCoordRange>
        coordinate_range& operator=(OtherCoordRange& o)
        {
            selptr = o.selptr;
            return *this;
        }
            

        // The selection to transpose must remain in place.
        //
        explicit coordinate_range(selection_t& sel)
            : selptr(&sel)
        {
        }
        explicit coordinate_range(selection_t* selptr)
            : selptr(selptr)
        {
        }

        /// Number of coordinate points.
        size_t size() const {
            if (!selptr || selptr->empty()) return 0;
            return (*selptr)[0]->size_major();
        }

        iterator begin()  { 
            return iterator(selptr, 0);
        }
        iterator end()  {
            return iterator(selptr, size());
        }
        const_iterator begin() const { 
            return const_iterator(selptr, 0);
        }
        const_iterator end() const {
            return const_iterator(selptr, size());
        }

      private:
        selection_t* selptr{nullptr};
        
    };
    using real_coordinate_range = coordinate_range<real_coordinate_point>;


    /// Return an unweighted mean point over a range of coordinate points.
    template<typename PointType = typename std::vector<double>,
             typename CoordRange = real_coordinate_range>
    void mean_point(PointType& mu, const CoordRange& pts) {
        const size_t ndim=pts.begin()->size();
        size_t num=0;
        for (const auto& pt : pts) {
            for (size_t ind=0; ind<ndim; ++ind) {
                mu[ind] = mu[ind] + pt[ind];
            }
            ++num;
        }
        for (size_t ind=0; ind<ndim; ++ind) {
            mu[ind] = mu[ind] / num;
        }
    }
    
    /// Return a weighted mean point over a range of coordinate points.
    template<typename CoordRange,
             typename PointType = typename CoordRange::point_type,
             typename WeightsType = std::vector<typename PointType::value_type>>
    void mean_point(PointType& mu, const CoordRange& pts, const WeightsType& wts) {
        size_t num=0;
        typename WeightsType::value_type wtot = 0;
        const size_t ndim=pts.begin()->size();
        for (const auto& pt : pts) {
            const typename WeightsType::value_type w = wts[num];
            for (size_t ind=0; ind<ndim; ++ind) {
                mu[ind] = mu[ind] + w * pt[ind];
            }
            ++num;
            wtot += w;
        }
        for (size_t ind=0; ind<ndim; ++ind) {
            mu[ind] = mu[ind] / wtot;
        }
    }

}

#endif


