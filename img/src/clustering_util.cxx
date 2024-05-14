#include <WireCellImg/ClusteringFuncs.h>

#include <iostream>              // temp debug

using namespace WireCell::PointCloud::Facade;

bool WireCell::PointCloud::Facade::cluster_less(const Cluster* a, const Cluster* b)
{
    if (a==b) return false;
    {
        const int na = a->nchildren();
        const int nb = b->nchildren();
        if (na < nb) return true;
        if (nb < na) return false;
    }
    {
        const int na = a->npoints();
        const int nb = b->npoints();
        if (na < nb) return true;
        if (nb < na) return false;
    }
    {
        auto ar = a->get_uvwt_min();
        auto br = b->get_uvwt_min();
        if (get<0>(ar) < get<0>(br)) return true;
        if (get<0>(br) < get<0>(ar)) return false;
        if (get<1>(ar) < get<1>(br)) return true;
        if (get<1>(br) < get<1>(ar)) return false;
        if (get<2>(ar) < get<2>(br)) return true;
        if (get<2>(br) < get<2>(ar)) return false;
        if (get<3>(ar) < get<3>(br)) return true;
        if (get<3>(br) < get<3>(ar)) return false;
    }
    {
        auto ar = a->get_uvwt_max();
        auto br = b->get_uvwt_max();
        if (get<0>(ar) < get<0>(br)) return true;
        if (get<0>(br) < get<0>(ar)) return false;
        if (get<1>(ar) < get<1>(br)) return true;
        if (get<1>(br) < get<1>(ar)) return false;
        if (get<2>(ar) < get<2>(br)) return true;
        if (get<2>(br) < get<2>(ar)) return false;
        if (get<3>(ar) < get<3>(br)) return true;
        if (get<3>(br) < get<3>(ar)) return false;
    }

    // The two are very similar.  What is left to check?  Only pointer?.  This
    // will cause "randomness"!
    return a < b;           
}
void WireCell::PointCloud::Facade::sort_clusters(std::vector<const Cluster*>& clusters)
{
    std::sort(clusters.rbegin(), clusters.rend(), cluster_less);
}
void WireCell::PointCloud::Facade::sort_clusters(std::vector<Cluster*>& clusters)
{
    std::sort(clusters.rbegin(), clusters.rend(), cluster_less);
}

void WireCell::PointCloud::Facade::merge_clusters(
    cluster_connectivity_graph_t& g,
    Grouping& live_grouping,
    // Live clusters that are "connected" to some dead cluster
    cluster_set_t& cluster_connected_dead) // in/out
{
    std::unordered_map<int, int> desc2id;
    std::unordered_map<int, std::set<int> > id2desc;
    /*int num_components =*/ boost::connected_components(g, boost::make_assoc_property_map(desc2id));
    for (const auto& [desc, id] : desc2id) {
        id2desc[id].insert(desc);
    }

    // Note, here we do an unusual thing and COPY the vector of children
    // facades.  In most simple access we would get the reference to the child
    // vector to save a little copy time.  We explicitly copy here as we must
    // preserve the original order of children facades even as we remove them
    // from the grouping.  As each child facade is removed, it's
    // unique_ptr<node> is returned which we ignore/drop and thus the child
    // facade dies along with its node.  This leaves the live_clusters element
    // that was just holding the pointer to the doomed facade now holding
    // invalid memory.  But, it is okay as we never revisit the same cluster in
    // the grouping.  All that to explain a missing "&"! :)
    auto live_clusters = live_grouping.children();
    // std::cerr << "merge_clusters: "
    //           << live_clusters.size() << " live clusters " 
    //           << id2desc.size() << " subgraphs\n";


    //    debug("id2desc size: {}", id2desc.size());
    for (const auto& [id, descs] : id2desc) {
        if (descs.size() < 2) {
            continue;
        }
        // std::cerr << "merge_clusters: subgraph id=" << id
        //           << " with " << descs.size() << " vertices\n";

        // it starts with no cluster facade
        Cluster& fresh_cluster = live_grouping.make_child();

        for (const auto& desc : descs) {
            const int idx = g[desc];
            if (idx < 0) {  // no need anymore ...
                continue;
            }
            auto live = live_clusters[idx];
            fresh_cluster.take_children(*live, true);

            live_grouping.remove_child(*live);
            cluster_connected_dead.erase(live);
        }
        cluster_connected_dead.insert(&fresh_cluster);
    }

    // sanity check / debugging
    for (const auto* cluster : live_grouping.children()) {
        if (!cluster) {
            std::cerr << "merge_clusters: null live cluster on output!\n";
            continue;
        }
        if (! cluster->nchildren()) {
            std::cerr << "merge_clusters: empty live cluster on output!\n";
            continue;
        }
        for (const auto* blob : cluster->children()) {
            if (!blob) {
                std::cerr << "merge_clusters: null live blob on output!\n";
                continue;
            }
        }
    }

}


