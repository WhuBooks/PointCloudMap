//
// Created by books on 2018/4/6.
// paper : Vision-based Localization Using an Edge Map Extracted From 3D Laser Range Data
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <list>
#include <algorithm>
#include <cmath>
#include <map>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Util.h>
#include <CloudUtil.h>

typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> PosVec;

struct Point
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    float intensity;
    Eigen::Vector3f pos;
    Eigen::Vector3f nor;
};

struct Planar
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    std::vector<int> data; /// store the point id inside the planar
    Eigen::Vector3f centroid;
    Eigen::Vector3f normal;

    /// calculate the planar's centroid and normal
    void Init(const std::vector<Point> &pts)
    {
        /// centroid
        Eigen::Vector3f pos_sum(0.0f, 0.0f, 0.0f);
        PosVec pos_vec;
        for (const int &index : data)
        {
            Eigen::Vector3f pos_index = pts[index].pos;
            pos_sum += pos_index;
            pos_vec.push_back(pos_index);
        }
        centroid = pos_sum / data.size();

        /// compute normal by inverse distance with centroid
        float sum_inverse_dis=0.0f;
        Eigen::Vector3f sum_nor(0.0f,0.0f,0.0f);
        for(const int &index : data)
        {
            Eigen::Vector3f pos_index=pts[index].nor;
            Eigen::Vector3f nor_index=pts[index].nor;

            float inverse_dis_index=1.0f/(pos_index-centroid).norm();
            sum_inverse_dis+=inverse_dis_index;
            sum_nor+=inverse_dis_index*nor_index;
        }
        sum_nor/=sum_inverse_dis;
        sum_nor.normalize();
        normal=sum_nor;
        return;

        /// covariance
        Eigen::Matrix3f cov;
        cov.setZero();
        for (const Eigen::Vector3f &pos : pos_vec)
        {
            Eigen::Vector3f pos_diff = pos - centroid;
            cov += pos_diff * pos_diff.transpose();
        }
        cov /= data.size();

        /// normal
        Eigen::EigenSolver<Eigen::Matrix3f> solver(cov);
        Eigen::Matrix3f D = solver.pseudoEigenvalueMatrix();
        Eigen::Matrix3f V = solver.pseudoEigenvectors();

        float min_eigen_value = 100000000000;
        Eigen::Vector3f result;
        for (int i = 0; i < 3; i++)
        {
            if (min_eigen_value > D(i, i))
            {
                min_eigen_value = D(i, i);
                result = V.col(i);
            }
        }
        result.normalize();
        normal = result;
    }
};

/// merge two planars' data
std::vector<int> MergePlanar(const Planar &planar1,const Planar &planar2)
{
    std::vector<int> merge_data,result;
    merge_data.insert(merge_data.end(),planar1.data.begin(),planar1.data.end());
    merge_data.insert(merge_data.end(),planar2.data.begin(),planar2.data.end());
    std::sort(merge_data.begin(),merge_data.end());
    
    int last_id=-10000;
    for(const int &id : merge_data)
    {
        if(last_id!=id)
        {
            result.push_back(id);
            last_id=id;
        }
    }
    
    return result;
}

/// calculate two planars' similarity
float PlanarSimilarity(const Planar &planar1,const Planar &planar2)
{
    Eigen::Vector3f cen1 = planar1.centroid;
    Eigen::Vector3f nor1 = planar1.normal;
    Eigen::Vector3f cen2 = planar2.centroid;
    Eigen::Vector3f nor2 = planar2.normal;

    float dis_o_12 = (cen1 - cen2).transpose() * nor1;
    float dis_o_21 = (cen1 - cen2).transpose() * nor2;
    float dis_o = dis_o_12 * dis_o_12 + dis_o_21 * dis_o_21;

    float dis_n = 1.0f - (nor1.transpose() * nor2) * (nor1.transpose() * nor2);

    const float weight_o = 0.5f, weight_n = 0.5f;
    return weight_o * dis_o + weight_n * dis_n;

}

/// represent the link relation of planars
struct Link
{
    int s_id;   ///planar id
    int e_id;   ///planar id
    float similarity;
};

float StdDev(const std::vector<Link> &vector)
{
    std::vector<float> sim_vec;
    for(const Link &lk : vector)
        sim_vec.push_back(lk.similarity);

    int size=vector.size();

    float ave=std::accumulate(sim_vec.begin(),sim_vec.end(),0.0f)/size;
    float square_sum=0.0f;
    for(const float &tmp : sim_vec)
        square_sum+=(tmp-ave)*(tmp-ave);

    float std_dev=std::sqrt(square_sum/size);
    return std_dev;
}

///// delete the repeat links
//void UniqueLinks(std::vector<Link> &links)
//{
//
//    std::vector<Link> tmp_links;
//    for (const Link &link : links)
//    {
//        bool flag = true;
//        for (const Link &tmp_link :tmp_links)
//        {
//            if (tmp_link.s_id == link.s_id && tmp_link.e_id == link.e_id)
//            {
//                flag = false;
//                break;
//            }
//            if (tmp_link.s_id == link.e_id && tmp_link.e_id == link.s_id)
//            {
//                flag = false;
//                break;
//            }
//        }
//        if (flag)
//            tmp_links.push_back(link);
//    }
//
//    links.swap(tmp_links);
//}

//void LogSimilarity(const std::vector<Link> links,std::string filename)
//{
//    std::ofstream ofs(filename,std::ios::app);
//    for(const Link &lk : links)
//    {
//        ofs<<std::setprecision(10)<<lk.similarity<<"\t";
//    }
//    ofs<<std::endl;
//    ofs.close();
//}

//void WriteMedianResult(const std::vector<Planar> &planars,const std::vector<Point> &pts,std::string filename)
//{
//    int num=planars.size();
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    for(int i=0;i<planars.size();i++)
//    {
//        for(const int &pt_id : planars[i].data)
//        {
//            Point p=pts[pt_id];
//            Eigen::Vector3f pos=p.pos;
//            pcl::PointXYZI pt;
//            pt.x=pos(0);
//            pt.y=pos(1);
//            pt.z=pos(2);
//            pt.intensity=p.intensity;
//            cloud->push_back(pt);
//        }
//    }
//
//    pcl::io::savePCDFileASCII(filename,*cloud);
//}

void WritePlanarResult(const std::vector<Planar> &planars,const std::vector<Point> &pts,std::string filename)
{
    int num=planars.size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i<planars.size();i++)
    {
        float intensity=(float)i/num;
        for(const int &pt_id : planars[i].data)
        {
            Point p=pts[pt_id];
            Eigen::Vector3f pos=p.pos;
            pcl::PointXYZI pt;
            pt.x=pos(0);
            pt.y=pos(1);
            pt.z=pos(2);
            pt.intensity=intensity;
            cloud->push_back(pt);
        }
    }
    pcl::io::savePCDFileBinary(filename,*cloud);
}

int main(int argc,char **argv)
{
    std::string pcdfile = (argc > 1) ? std::string(argv[1]) : "50_mls.pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(pcdfile, *cloud);
    std::cout << "Load Origin Cloud Size ~ " << cloud->size() << std::endl;

    PCM::ConditionFilter(cloud);

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    flann->setInputCloud(cloud);

    /// convert pcl's point to Point, and collect neighbour's id
    /// also check the planar point and initialize planar
    const float planar_eigenvalue_threshold = 10;
    std::vector<Point> pts;
    std::vector<Planar> planars;
    std::vector<Link> index_links;
    std::vector<int> index_ids(cloud->points.size(), -1);
    int planar_id = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_init_planars(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZI pt = cloud->points[i];

        /// initialize a point
        Point p;
        p.id = i;
        p.pos = Eigen::Vector3f(pt.x, pt.y, pt.z);
        p.nor = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        p.intensity = pt.intensity;

        /// knn search
        std::vector<int> vindices;
        std::vector<float> vdistance;
        flann->nearestKSearch(pt, 16, vindices, vdistance);

        /// collect knn point' pos and id
        std::vector<int> tmp_knn_id;
        PosVec knn_posvec;
        Eigen::Vector3f pos_sum(0.0f, 0.0f, 0.0f);
        for (int j = 0; j < vindices.size(); j++)
        {
            int index = vindices[j];
            float dis = vdistance[j];

            if (index != i && dis <= 2.0f)
            {
                tmp_knn_id.push_back(index);

                pcl::PointXYZI pt_knn = cloud->points[index];
                Eigen::Vector3f pos_knn(pt_knn.x, pt_knn.y, pt_knn.z);
                knn_posvec.push_back(pos_knn);
                pos_sum += pos_knn;
            }
        }

        /// if current point has enough knn_pt within fixed radius
        if (tmp_knn_id.size() > 4)
        {
            Eigen::Vector3f pos(pt.x, pt.y, pt.z);
            knn_posvec.push_back(pos);
            pos_sum += pos;
            Eigen::Vector3f pos_ave = pos_sum / (knn_posvec.size());

            /// calculate cov of knn pos
            Eigen::Matrix3f knn_cov;
            knn_cov.setZero();
            for (const Eigen::Vector3f &tmp_pos : knn_posvec)
            {
                Eigen::Vector3f pos_diff = tmp_pos - pos_sum;
                knn_cov += pos_diff * pos_diff.transpose();
            }
            knn_cov /= (knn_posvec.size());

            /// eigen solver
            Eigen::EigenSolver<Eigen::Matrix3f> solver(knn_cov);
            Eigen::Matrix3f D = solver.pseudoEigenvalueMatrix();
            Eigen::Matrix3f V = solver.pseudoEigenvectors();

            /// collect first and second minimum eigen-value
            float min_eigen_val = 100000000, sec_min_eigen_val = 1000000000;
            Eigen::Vector3f normal;
            for (int jj = 0; jj < 3; jj++)
            {
                float eigen_val = D(jj, jj);
                /// first select second mininum
                if (sec_min_eigen_val > eigen_val)
                {
                    sec_min_eigen_val = eigen_val;
                }

                /// then compare mininum with second mininum
                if (min_eigen_val >= sec_min_eigen_val)
                {
                    float tmp_min_eigen_val = min_eigen_val;
                    min_eigen_val = sec_min_eigen_val;
                    sec_min_eigen_val = tmp_min_eigen_val;

                    normal = V.col(jj);
                    normal.normalize();
                }
            }

            /// planar seed point's pos is the average pos of knn, normal is knn normal
            p.pos = pos_ave;
            p.nor = normal;

            /// if second mininum far more larger than mininum, consider it as a planar seed
            if (sec_min_eigen_val > planar_eigenvalue_threshold * min_eigen_val)
            {
                cloud_init_planars->push_back(pt);

                /// set value=planar_id of the same place in index_ids
                index_ids[i] = planar_id;

                /// initialize a planar
                Planar planar;
                planar.id = planar_id;
                planar.data.push_back(i);
                //planar.data=vindices;
                planar.normal = normal;
                planar.centroid = pos_ave;
                planars.push_back(planar);

                /// add the knn relation to link
                for (const int &index : tmp_knn_id)
                {
                    Link lk;
                    lk.s_id = planar_id;
                    lk.e_id = index;
                    lk.similarity = 100000.0f;
                    index_links.push_back(lk);
                }
                planar_id = planar_id + 1;
            }
        }

        pts.push_back(p);
    }
    std::cout << "Initialize Planar Size ~ " << planars.size() << std::endl;

    /// write the planar seed result
    std::string dir = Util::GetNameFromTime();
    Util::DirBuild(dir);
    std::string initfile = dir + "/init.pcd";
    pcl::io::savePCDFileBinary(initfile, *cloud_init_planars);

    /// update links's eid with planar_id
    std::vector<Link> links;
    for (const Link &lk : index_links)
    {
        Link tmp_lk;
        tmp_lk.s_id = lk.s_id;
        int tmp_eid = index_ids[lk.e_id];
        if (tmp_eid == -1 || lk.s_id == tmp_eid)
            continue;
        tmp_lk.e_id = tmp_eid;
        tmp_lk.similarity = lk.similarity;
        links.push_back(tmp_lk);
    }

    /// assume that s_id less than e_id
    for (Link &lk : links)
    {
        int tmp_s_id = lk.s_id;
        int tmp_e_id = lk.e_id;
        lk.s_id = (tmp_s_id < tmp_e_id) ? tmp_s_id : tmp_e_id;
        lk.e_id = (tmp_s_id > tmp_e_id) ? tmp_s_id : tmp_e_id;
    }

    std::sort(links.begin(), links.end(), [](const Link &lk1, const Link &lk2) {
        return (lk1.s_id < lk2.s_id) || (lk1.s_id == lk2.s_id && lk1.e_id < lk2.e_id);
    });

    /// delete the repeat links
    index_links.clear();
    int last_s_id = -10000000, last_e_id = -1000000;
    for (const Link &lk : links)
    {
        if (lk.s_id != last_s_id || lk.e_id != last_e_id)
            index_links.push_back(lk);
        last_s_id = lk.s_id;
        last_e_id = lk.e_id;
    }
    links.swap(index_links);
    std::cout << "Unique Links Size ~ " << links.size() << std::endl;

    for (Link &lk : links)
    {
        Planar s_planar, e_planar;
        s_planar = planars[lk.s_id];
        e_planar = planars[lk.e_id];

        lk.similarity = PlanarSimilarity(s_planar, e_planar);
    }
    std::cout << "Initialize Link Size ~ " << links.size() << std::endl;

    /// merge planars iteratively
    const float similarity_threshold = 3.0f;
    bool traverse_flag = true;
    int traverse_iter = 0;
    while (traverse_flag)
    {
        /// sort link by similarity
        std::sort(links.begin(), links.end(), [](const Link &lk1, const Link &lk2) {
            return lk1.similarity < lk2.similarity;
        });

//        LogSimilarity(links,logfile);
        traverse_iter++;
        if (traverse_iter % 10000 == 0)
        {
            std::string tmp_file = dir + "/" + std::to_string(traverse_iter) + ".pcd";
            WritePlanarResult(planars, pts, tmp_file);
            std::cout << "Traverse turns ~ " << traverse_iter << std::endl;
            std::cout << "Planars Size ~ " << planars.size() << std::endl;
            std::cout << "Link Size ~ " << links.size() << std::endl;
            std::cout << "Std of Link Similarity ~ " << std::setprecision(8) << StdDev(links) << std::endl << std::endl;
        }
        float min_similarity = links.front().similarity;
        int min_similarity_id1 = links.front().s_id, min_similarity_id2 = links.front().e_id;

        /// if minimum similarity larger than threshold, stop the merge process
        if (min_similarity > similarity_threshold)
            break;

        /// update planars
        std::vector<Planar> tmp_planars;
        Planar tmp_planar1, tmp_planar2;
        for (const Planar &tmp : planars)
        {
            if (tmp.id == min_similarity_id1)
                tmp_planar1 = tmp;
            else if (tmp.id == min_similarity_id2)
                tmp_planar2 = tmp;
            else
                tmp_planars.push_back(tmp);
        }

        /// merge tmp_planar1 and tmp_planar2
        std::vector<int> merge_data;
        merge_data.insert(merge_data.end(), tmp_planar1.data.begin(), tmp_planar1.data.end());
        merge_data.insert(merge_data.end(), tmp_planar2.data.begin(), tmp_planar2.data.end());
        Planar merge_planar;
        merge_planar.id = tmp_planar1.id;
        merge_planar.data = merge_data;
        merge_planar.Init(pts);
        tmp_planars.push_back(merge_planar);



        /// update links
        std::vector<Link> tmp_links;
        for (const Link &lk : links)
        {
            /// remove the link between tmp_planar1 and tmp_planar2
            if ((lk.s_id == min_similarity_id1 && lk.e_id == min_similarity_id2) ||
                (lk.s_id == min_similarity_id2 && lk.e_id == min_similarity_id1))
                continue;

            /// unchange the link which has no relation to tmp_planar1 or tmp_planar2
            if (lk.s_id != min_similarity_id1 && lk.s_id != min_similarity_id2 && lk.e_id != min_similarity_id1 &&
                lk.e_id != min_similarity_id2)
            {
                tmp_links.push_back(lk);
                continue;
            }

            /// if link's sid equal to id1 or id2
            if (lk.s_id == min_similarity_id1 || lk.s_id == min_similarity_id2)
            {
                float similarity_lk_planar1 = 1000000;
                for (const Planar &tmp_planar : planars)
                {
                    if (tmp_planar.id == lk.e_id)
                    {
                        similarity_lk_planar1 = PlanarSimilarity(tmp_planar1, tmp_planar);
                        break;
                    }
                }

                Link tmp_lk;
                tmp_lk.similarity = similarity_lk_planar1;
                tmp_lk.s_id = lk.s_id;
                tmp_lk.e_id = lk.e_id;
                if (lk.s_id == min_similarity_id2)
                    tmp_lk.s_id = min_similarity_id1;
                tmp_links.push_back(tmp_lk);
                continue;
            }

            /// if link's eid equal to id1 or id2
            if (lk.e_id == min_similarity_id1 || lk.e_id == min_similarity_id2)
            {
                float similarity_lk_planar1 = 1000000;
                for (const Planar &tmp_planar : planars)
                {
                    if (tmp_planar.id == lk.s_id)
                    {
                        similarity_lk_planar1 = PlanarSimilarity(tmp_planar1, tmp_planar);
                        break;
                    }
                }
                Link tmp_lk;
                tmp_lk.similarity = similarity_lk_planar1;
                tmp_lk.s_id = lk.s_id;
                tmp_lk.e_id = lk.e_id;
                if (lk.e_id == min_similarity_id2)
                    tmp_lk.e_id = min_similarity_id1;
                tmp_links.push_back(tmp_lk);
                continue;
            }

            std::cout << "Extra Condition Occur!" << std::endl;
        }

        /// delete the repeat links
        for (Link &lk : tmp_links)
        {
            int tmp_s_id = lk.s_id;
            int tmp_e_id = lk.e_id;
            lk.s_id = (tmp_s_id < tmp_e_id) ? tmp_s_id : tmp_e_id;
            lk.e_id = (tmp_s_id > tmp_e_id) ? tmp_s_id : tmp_e_id;
        }

        std::sort(tmp_links.begin(), tmp_links.end(), [](const Link &lk1, const Link &lk2) {
            return (lk1.s_id < lk2.s_id) || (lk1.s_id == lk2.s_id && lk1.e_id < lk2.e_id);
        });

        index_links.clear();
        int last_s_id = -10000000, last_e_id = -1000000;
        for (const Link &lk : links)
        {
            if (lk.s_id != last_s_id || lk.e_id != last_e_id)
                index_links.push_back(lk);
            last_s_id = lk.s_id;
            last_e_id = lk.e_id;
        }
        links.swap(index_links);

        planars.swap(tmp_planars);
        links.swap(tmp_links);
    }
    std::cout << "Merge Planars Size ~ " << planars.size() << std::endl;
    std::cout << "Merge Iteration ~ " << traverse_iter << std::endl;

    WritePlanarResult(planars, pts, dir + "/Merge.pcd");

    /// remove planars which is too small
    const int num_threshold = 100;
    std::vector<Planar> clusters;
    for (const Planar &planar : planars)
    {
        if (planar.data.size() > num_threshold)
            clusters.push_back(planar);
    }
    std::cout << "Clusters Size ~ " << clusters.size() << std::endl;

    WritePlanarResult(clusters, pts, dir + "/Cluster.pcd");

}

