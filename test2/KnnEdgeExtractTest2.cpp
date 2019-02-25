//
// Created by whubooks on 18-4-5.
// paper : A Fast Edge Extraction Method for Mobile Lidar Points
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <list>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Util.h>

struct Node
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    Eigen::Vector3f pos;
    Eigen::Vector3f orientation;
};

struct Edge
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    int s_id;
    int e_id;
    Eigen::Vector3f orientation;
};

int main(int argc,char **argv)
{
    std::string pcdfile = (argc > 1) ? std::string(argv[1]) : "50_mls_nms.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);
    std::cout<<"Load Cloud Size ~ "<<cloud->size()<<std::endl;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    flann->setInputCloud(cloud);
    int K = 8;

    /// init knn graph using edge cloud
    std::vector<Edge> graph;
    std::vector<Node> nodes;
    int edge_id = 0;
    for (int i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZI pt = cloud->points[i];

        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->nearestKSearch(pt, K + 1, vindices, vdistance);

        Node node;
        node.id = i;
        node.pos = Eigen::Vector3f(pt.x, pt.y, pt.z);
        nodes.push_back(node);

        for (const int &index : vindices)
        {
            if (index == i)
                continue;
            /// create a new edge
            Edge edge;
            edge.id = edge_id++;
            edge.s_id = i;
            edge.e_id = index;
            graph.push_back(edge);
        }
    }
    std::cout<<"Initialize Knn Graph Size ~ "<<graph.size()<<std::endl;

    /// traverse the knn edge graph with decrease angle threshold
    for (float theta = 80.0f; theta > 20.0f; theta = theta - 10.0f)
    {
        const float theta_grad=theta*M_PI/180.f;
        std::sort(graph.begin(), graph.end(), [](const Edge &edge1, const Edge &edge2) {
            return edge1.s_id < edge2.s_id || (edge1.s_id == edge2.s_id && edge1.e_id < edge2.e_id);
        });

        /// group the edges using edge's s_id and smooth the node's pos
        std::vector<Node> tmp_nodes;
        int old_s_id = -1;
        std::vector<Edge> knn_edges;
        for (const Edge &edge : graph)
        {
            /// if get a new edge group
            if (old_s_id != -1 && old_s_id != edge.s_id)
            {
                Eigen::Vector3f cur_pos = nodes[old_s_id].pos;
                Eigen::Vector3f knn_ave_pos(0.0f, 0.0f, 0.0f);
                for (const Edge &knn_edge : knn_edges)
                    knn_ave_pos += nodes[knn_edge.e_id].pos;
                knn_ave_pos /= knn_edges.size();

                /// smooth the node's pos using knn node
                Node node;
                node.id = old_s_id;
                node.pos = 0.5 * (cur_pos + knn_ave_pos);
                tmp_nodes.push_back(node);

                knn_edges.clear();
            }

            old_s_id = edge.s_id;
            knn_edges.push_back(edge);
        }
        nodes.swap(tmp_nodes);
        tmp_nodes.clear();
        std::cout << "Smooth Done!" << std::endl;

        /// estimate orientation which minimize all edge's included angle
        old_s_id = -1;
        knn_edges.clear();
        std::vector<int> remove_edge_id;
        for (Edge &edge : graph)
        {
            /// if get a new edge group,
            if (old_s_id != -1 && old_s_id != edge.s_id)
            {
                /// find node's orientation by calculate included angle
                float min_sum_angle = 1000000.0f;
                Eigen::Vector3f node_orientation;
                std::vector<int> tmp_remove_edge_id;

                for (int ii = 0; ii < knn_edges.size(); ii++)
                {
                    std::vector<int> remove_edge_id_ii;
                    float sum_angle = 0.0f;
                    Eigen::Vector3f orien_ii = knn_edges[ii].orientation;
                    for (int jj = 0; jj < knn_edges.size(); jj++)
                    {
                        Eigen::Vector3f orien_jj = knn_edges[jj].orientation;
                        float product = orien_ii.transpose() * orien_jj;
                        float angle = std::acos(product / (orien_ii.norm() * orien_jj.norm()));
                        sum_angle += angle;

                        /// if one knn edge's included angle larger than theta
                        if (angle > theta_grad)
                            remove_edge_id_ii.push_back(knn_edges[jj].id);
                    }
                    /// if the sum included angle is smaller, than think it's the node's orientation
                    if (min_sum_angle > sum_angle)
                    {
                        min_sum_angle = sum_angle;
                        node_orientation = knn_edges[ii].orientation;
                        tmp_remove_edge_id.swap(remove_edge_id_ii);
                    }
                }

                /// update node's orientation and collect new removal edge's id
                nodes[old_s_id].orientation = node_orientation;
                remove_edge_id.insert(remove_edge_id.end(), tmp_remove_edge_id.begin(), tmp_remove_edge_id.end());

                knn_edges.clear();
            }

            /// estimate edge's orientation using the pos after smooth
            edge.orientation = nodes[edge.e_id].pos - nodes[edge.s_id].pos;
            old_s_id = edge.s_id;
            knn_edges.push_back(edge);
        }
        std::cout<<"Remove Edge Size ~ "<<remove_edge_id.size()<<std::endl;

        /// remove edge which included in remove_edge_id
        if(!remove_edge_id.empty())
        {
            std::sort(graph.begin(), graph.end(), [](const Edge &edge1, const Edge &edge2) {
                return edge1.id < edge2.id;
            });
            std::sort(remove_edge_id.begin(), remove_edge_id.end());
            int remove_edge_id_index = 0;
            int tmp_edge_remove_id = remove_edge_id[remove_edge_id_index];
            std::vector<Edge> tmp_graph;
            for (const Edge &edge : graph)
            {
                if (edge.id == tmp_edge_remove_id)
                {
                    remove_edge_id_index++;
                    if (remove_edge_id_index < remove_edge_id.size())
                        tmp_edge_remove_id = remove_edge_id[remove_edge_id_index];
                    continue;
                }
                tmp_graph.push_back(edge);

//            bool flag = false;
//            for (const int &tmp : remove_edge_id)
//            {
//                flag = flag || (tmp == edge.id);
//            }
//            if (!flag)
//                tmp_graph.push_back(edge);
            }
            assert(tmp_graph.size() + remove_edge_id.size() == graph.size());
            graph.swap(tmp_graph);
        }
        std::cout << "New Graph Size ~ " << graph.size() << std::endl << std::endl;
    }

    /// unique the edges of graph
    for(Edge &tmp : graph)
    {
        int tmp_s_id=tmp.s_id;
        int tmp_e_id=tmp.e_id;
        tmp.s_id=(tmp_s_id>tmp_e_id)?tmp_s_id:tmp_e_id;
        tmp.e_id=(tmp_s_id<tmp_e_id)?tmp_s_id:tmp_e_id;
    }

    std::sort(graph.begin(),graph.end(),[](const Edge &edge1,const Edge &edge2){
        return edge1.s_id<edge2.s_id || (edge1.s_id==edge2.s_id && edge1.e_id<edge2.e_id);
    });

    int last_s_id=-1000000,last_e_id=-1000000;
    std::vector<Edge> tmp_graph;
    for(const Edge &tmp : graph)
    {
        if(last_s_id!=tmp.s_id || last_e_id != tmp.e_id)
            tmp_graph.push_back(tmp);
        last_s_id=tmp.s_id;
        last_e_id=tmp.e_id;
    }
    graph.swap(tmp_graph);
    std::cout<<"Unique Graph Size ~ "<<graph.size()<<std::endl;

    std::vector<std::vector<int>> lines;
    while (!graph.empty())
    {
        /// select one node as first seed
        std::vector<int> subgraph_seed_id;
        subgraph_seed_id.push_back(graph.front().s_id);

        bool flag = true;
        while (flag)
        {
            std::function<int(const Edge&,const std::vector<int>&)> ContainFuc=[](const Edge &tmp_edge,const std::vector<int> &vec){
                for(const int &tmp_seed : vec)
                {
                    if(tmp_edge.s_id == tmp_seed)
                        return 1;
                    if(tmp_edge.e_id == tmp_seed)
                        return -1;
                }
                return 0;
            };

            /// find node in graph which connect to seed node
            std::vector<int> tmp_subgraph_seed_id;
            std::vector<Edge> tmp_graph;

            for(const Edge &edge : graph)
            {
                int contain_result=ContainFuc(edge,subgraph_seed_id);
                if(contain_result==1)
                    tmp_subgraph_seed_id.push_back(edge.e_id);
                else if(contain_result==-1)
                    tmp_subgraph_seed_id.push_back(edge.s_id);
                else
                    tmp_graph.push_back(edge);
            }

            assert(tmp_graph.size()+tmp_subgraph_seed_id.size()==graph.size());

            /// if can't find new node
            if (tmp_subgraph_seed_id.empty())
                break;

            /// merge new node
            subgraph_seed_id.insert(subgraph_seed_id.end(), tmp_subgraph_seed_id.begin(), tmp_subgraph_seed_id.end());

            /// update graph, remove the found edge
            graph.swap(tmp_graph);

        }

        /// remove repeat seeds in the sub-graph
        std::vector<int> tmp_subgraph_seed_id;
        std::sort(subgraph_seed_id.begin(),subgraph_seed_id.end());
        int last_seed_id=-100000;
        for(const int tmp_seed_id : subgraph_seed_id)
        {
            if(tmp_seed_id!=last_seed_id)
                tmp_subgraph_seed_id.push_back(tmp_seed_id);
            last_seed_id=tmp_seed_id;
        }
        subgraph_seed_id.swap(tmp_subgraph_seed_id);

        std::cout << "New Sub-Graph Size ~ " << subgraph_seed_id.size() << std::endl;
        lines.push_back(subgraph_seed_id);
    }
    std::cout << "Edge Size ~ " << lines.size() << std::endl;

    /// write result
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edges(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i<lines.size();i++)
    {
        std::vector<int> line=lines[i];
        float intensity=(float)i/lines.size();
        for(const int &node_id : line)
        {
            Node node=nodes[node_id];
            pcl::PointXYZI pt;
            pt.x=node.pos(0);
            pt.y=node.pos(1);
            pt.z=node.pos(2);
            pt.intensity=intensity;
            cloud_edges->push_back(pt);
        }
    }
    std::string edge_file=Util::SplitNameWithoutExt(pcdfile)+"_edge.pcd";
    pcl::io::savePCDFileBinary(edge_file,*cloud_edges);
    return 1;
}