#include <WireCellImg/ClusteringFuncs.h>

#include <cassert>              // temp debug

using namespace WireCell::PointCloud::Facade;


void WireCell::PointCloud::Facade::merge_clusters(boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, int>& g,
						  Points::node_ptr& root_live,                                   // in/out
						  Cluster::vector& live_clusters,
						  std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
						  std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
						  const TPCParams& tp,                                           // common params
						  const std::set<std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> >& cluster_to_be_deleted
						  ){

  
  
   Cluster::vector live_clusters_new;
    for (auto it = cluster_to_be_deleted.begin(); it != cluster_to_be_deleted.end(); it++) {
        cluster_length_map.erase(*it);
        cluster_connected_dead.erase(*it);
        // delete old clusters ...
        //	live_clusters.erase(find(live_clusters.begin(), live_clusters.end(), *it));
    }

    for (auto it = live_clusters.begin(); it != live_clusters.end(); it++) {
        if (cluster_to_be_deleted.find(*it) == cluster_to_be_deleted.end()) live_clusters_new.push_back(*it);
    }

    // // from dead -> lives graph
    // start = std::chrono::high_resolution_clock::now();
    // // dead: negative, live: positive
    // for (size_t idead = 0; idead < dead_clusters.size(); ++idead) {
    //     const auto& dead = dead_clusters[idead];
    //     const auto ddesc = boost::add_vertex(-idead, g);
    //     for (size_t ilive = 0; ilive < live_clusters.size(); ++ilive) {
    //         const auto& live = live_clusters[ilive];
    //         // insert live to graph if not already
    //         if (ilive2desc.find(ilive) == ilive2desc.end()) {
    //             ilive2desc[ilive] = boost::add_vertex(ilive, g);
    //         }
    //         if (live->is_connected(*dead, m_dead_live_overlap_offset).size()) {
    //             boost::add_edge(ddesc, ilive2desc[ilive], g);
    //         }
    //     }
    //     if (boost::out_degree(ddesc, g) > 1) {
    //         debug("dead2lives-graph {} {} {} {} ", idead, ddesc, g[ddesc], boost::out_degree(ddesc, g));
    //     }
    // }
    // end = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // timers["dead2lives-graph"] += duration;
    // debug("dead2lives-graph {} ms", timers["dead2lives-graph"].count());

    // Make new live node tree
    Points::node_ptr root_live_new = std::make_unique<Points::node_t>();
    //    debug("root_live {} root_live_new {}", root_live->nchildren(), root_live_new->nchildren());
    // std::unordered_set<Cluster::pointer> need_merging;
    // for (const auto& [dead, lives] : dead2lives) {
    //     if (lives.size() < 2) {
    //         continue;
    //     }
    //     debug("dead2lives size for dead cluster: {}", lives.size());
    //     need_merging.insert(lives.begin(), lives.end());
    //     auto cnode = root_live_new->insert(std::move(std::make_unique<Points::node_t>()));
    //     for (const auto& live : lives) {
    //         for (const auto& blob : live->m_blobs) {
    //             // this also removes blob node from root_live
    //             cnode->insert(blob->m_node);
    //         }
    //         // manually remove the cnode from root_live
    //         root_live->remove(live->m_node);
    //     }
    // }
    // debug("need_merging size: {}", need_merging.size());

    std::unordered_map<int, int> desc2id;
    std::unordered_map<int, std::set<int> > id2desc;
    /*int num_components =*/ boost::connected_components(g, boost::make_assoc_property_map(desc2id));
    for (const auto& [desc, id] : desc2id) {
        id2desc[id].insert(desc);
    }
    //    debug("id2desc size: {}", id2desc.size());
    for (const auto& [id, descs] : id2desc) {
        if (descs.size() < 2) {
            continue;
        }

	//        debug("id {} descs size: {}", id, descs.size());

        auto cnode1 = std::make_unique<Points::node_t>();
        for (const auto& desc : descs) {
            const int idx = g[desc];
            if (idx < 0) {  // no need anymore ...
                continue;
            }
            const auto& live = live_clusters[idx];
            for (const auto& blob : live->m_blobs) {
                // this also removes blob node from root_live
                cnode1->insert(blob->m_node);
            }
            // manually remove the cnode from root_live
            root_live->remove(live->m_node);
        }

        // new cluster information (need Haiwang to take a look at Facade ...)
        auto cnode = root_live_new->insert(std::move(cnode1));
        auto new_cluster = std::make_shared<Cluster>(cnode);
        cluster_length_map[new_cluster] = new_cluster->get_length(tp);
        live_clusters_new.push_back(new_cluster);
        //	std::cout << "xin6:  " <<  cluster_length_map[new_cluster]/units::cm << std::endl;
        cluster_connected_dead.insert(new_cluster);
    }
    //    debug("root_live {} root_live_new {}", root_live->nchildren(), root_live_new->nchildren());

    // move remaining live clusters to new root
    for (auto* cptr : root_live->children()) {
        assert(cptr);
        auto cptr2 = root_live_new->insert(cptr);
        assert(cptr2);
    }
    //    debug("root_live {} root_live_new {}", root_live->nchildren(), root_live_new->nchildren());
    // replace old with new
    root_live = std::move(root_live_new);
    live_clusters = std::move(live_clusters_new);
  
}

