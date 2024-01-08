#include "WireCellUtil/doctest.h"
#include "WireCellUtil/Logging.h"
#include "WireCellUtil/GraphTools.h"

#include <boost/graph/adjacency_list.hpp>

#include <unordered_map>

using namespace WireCell;
using spdlog::debug;

// Define a struct for the vertex properties
struct VertexProperties {
    size_t ident;
};
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, VertexProperties> graph_t;


namespace {
    // function make SimpleCluster with two connected blobs
    graph_t graph1()
    {
        // make a graph
        graph_t graph;
        boost::add_vertex({1}, graph);
        boost::add_vertex({2}, graph);
        // add edges
        boost::add_edge(0, 1, graph);
        return graph;
    }
    graph_t graph2()
    {
        // make a graph
        graph_t graph;
        boost::add_vertex({10}, graph);
        boost::add_vertex({20}, graph);
        // add edges
        boost::add_edge(0, 1, graph);
        return graph;
    }

    // print the graph into a string
    std::string print_graph(const graph_t& graph)
    {
        std::stringstream ss;
        for (auto const& vtx : GraphTools::mir(boost::vertices(graph))) {
            const auto& node = graph[vtx];
            ss << node.ident << " ";
        }
        ss << "\n";
        for (auto const& edge : GraphTools::mir(boost::edges(graph))) {
            const auto& src = boost::source(edge, graph);
            const auto& tgt = boost::target(edge, graph);
            ss << src << " " << tgt << "\n";
        }
        return ss.str();
    }

    // function to merge  boost graph
    graph_t merge_graphs(const std::vector<graph_t>& graphs)
    {
        graph_t merged_graph;

        // merge the graphs
        for (const auto& graph : graphs) {
            std::unordered_map<size_t, size_t> vertex_map;
            // add the vertices
            for (const auto& vtx : GraphTools::mir(boost::vertices(graph))) {
                const auto& node = graph[vtx];
                auto new_vtx = boost::add_vertex({node.ident}, merged_graph);
                vertex_map[vtx] = new_vtx;
            }
            // add the edges
            for (const auto& edg : GraphTools::mir(boost::edges(graph))) {
                const auto& src = boost::source(edg, graph);
                const auto& tgt = boost::target(edg, graph);
                const auto& edge = graph[edg];
                auto new_src = vertex_map[src];
                auto new_tgt = vertex_map[tgt];
                boost::add_edge(new_src, new_tgt, edge, merged_graph);
                std::cout << "src " << src << " tgt " << tgt << " new_src " << new_src << " new_tgt " << new_tgt << "\n";
            }
        }
        return merged_graph;
    }
}


TEST_CASE("test merge cluster")
{
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    const auto g1 = graph1();
    const auto g2 = graph2();
    const auto merged = merge_graphs({g2, g1});
    debug("g1 {}", print_graph(g1));
    debug("g2 {}", print_graph(g2));
    debug("merged {}", print_graph(merged));
}