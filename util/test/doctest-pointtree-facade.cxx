/**
   Test bolting on "facades" to PC tree nodes.

   Here, we take a common facade type across each layer of the tree.
 */

#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/doctest.h"

#include <variant>

namespace WireCell::Clustering {
    using namespace PointCloud::Tree;
    struct Layer : public Facade
    {
        virtual ~Layer() {};
        virtual void set_node(PointsNode* n) { node = n; }
        PointsNode* node;
    };
    struct L0 : public Layer { int level{0}; };
    struct L1 : public Layer { int level{1}; };
    struct L2 : public Layer { int level{2}; };

    template<typename L>
    L& layer(Facade* l)
    {
        L* ll = dynamic_cast<L*>(l);
        if (!ll) {
            raise<TypeError>("can not dynamic cast point tree node facade");
        }
        return *ll;
    }
    template<typename L>
    L& layer(Points& value)
    {
        auto* l = value.facade();
        return layer<L>(l);
    }
    template<typename L>
    L& layer(PointsNode& node)
    {
        return layer<L>(node.value);
    }
    template<typename L>
    L& layer(PointsNode* node)
    {
        return layer<L>(node->value);
    }



}

using namespace WireCell;
using namespace WireCell::Clustering;

TEST_CASE("point tree subclass construct")
{
    Points::node_t root;
    root.value.set_facade(std::make_unique<L0>());
    auto* l1 = root.insert();
    REQUIRE(l1);
    l1->value.set_facade(std::make_unique<L1>());
    auto* l2 = l1->insert();
    REQUIRE(l2);
    l2->value.set_facade(std::make_unique<L2>());

    {
        auto& l = layer<L0>(root);
        CHECK(l.level == 0);
        REQUIRE(l.node);
    }
    {
        auto& l = layer<L1>(root.children()[0]);
        CHECK(l.level == 1);
        REQUIRE(l.node);
    }
    {
        auto& l = layer<L1>(l1);
        CHECK(l.level == 1);
        REQUIRE(l.node);
    }
    {
        auto& l = layer<L2>(l1->children()[0]);
        CHECK(l.level == 2);
        REQUIRE(l.node);
    }
    {
        auto& l = layer<L2>(l2);
        CHECK(l.level == 2);
        REQUIRE(l.node);
    }

    {
        REQUIRE(l2->value.facade());
        auto dead = l1->remove(l2);
        REQUIRE(dead);
        REQUIRE(dead.get() == l2);
        REQUIRE(dead->value.facade());
        auto& l = layer<L2>(*dead);
        REQUIRE(l.node);
        CHECK(l.level == 2);
        dead = nullptr;
    }

}
