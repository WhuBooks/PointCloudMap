//
// Created by books on 2018/3/26.
//

#include "CloudUtil.h"

namespace PCM {
    bool ConvertPcd2Las(const std::string &pcdfile, const std::string &lasfile)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        int pcd_result = pcl::io::loadPCDFile(pcdfile, *cloud);
        if (pcd_result < 0)
        {
            std::cerr << "Read Pcd File Failed." << std::endl;
            return false;
        }

        std::vector<double> x_vec, y_vec, z_vec, intensity_vec;
        for (const pcl::PointXYZI &pt : cloud->points)
        {
            x_vec.push_back(pt.x);
            y_vec.push_back(pt.y);
            z_vec.push_back(pt.z);
            intensity_vec.push_back(pt.intensity);
        }
        std::cout << "Point Size ~ " << x_vec.size() << std::endl;

        double minI = *(std::min_element(intensity_vec.begin(), intensity_vec.end()));
        double maxI = *(std::max_element(intensity_vec.begin(), intensity_vec.end()));

        std::ofstream ofs(lasfile);
        if (!ofs.is_open())
        {
            std::cerr << "Create Las File Failed." << std::endl;
            return false;
        }

        liblas::Header header;
        header.SetDataFormatId(liblas::ePointFormat1); // Time only

        header.SetScale(0.01, 0.01, 0.01);

        // Set coordinate system using GDAL support
        liblas::SpatialReference srs;
        srs.SetFromUserInput("EPSG:4326");
        header.SetSRS(srs);
        liblas::Writer writer(ofs, header);

        for (int i = 0; i < x_vec.size(); i++)
        {
            liblas::Point tmp(&header);
            tmp.SetCoordinates(x_vec[i], y_vec[i], z_vec[i]);
            uint16_t intensity = static_cast<uint16_t >((intensity_vec[i] - minI) / (maxI - minI) * 65536);
            tmp.SetIntensity(intensity);
            writer.WritePoint(tmp);
        }
        return true;
    }

    void WriteLas(CloudXYZPtr line_cloud,CloudXYZPtr left_cloud,CloudXYZPtr right_cloud ,const std::string &lasfile)
    {
        if(line_cloud->empty() && left_cloud->empty() && right_cloud->empty())
            return;

        std::ofstream ofs(lasfile);
        if (!ofs.is_open())
        {
            std::cerr << "Create Las File Failed." << std::endl;
            return;
        }

        liblas::Header header;
        header.SetDataFormatId(liblas::ePointFormat2); // Time only

        header.SetScale(0.01, 0.01, 0.01);

        // Set coordinate system using GDAL support
        liblas::SpatialReference srs;
        srs.SetFromUserInput("EPSG:4326");
        header.SetSRS(srs);
        liblas::Writer writer(ofs, header);

        liblas::Color line_color(255,0,0);
        liblas::Color left_color(0,255,0);
        liblas::Color right_color(0,0,255);

        for(const pcl::PointXYZ &pt : line_cloud->points)
        {
            liblas::Point tmp(&header);
            tmp.SetCoordinates(pt.x,pt.y,pt.z);
            tmp.SetIntensity(100);
            tmp.SetColor(line_color);
            writer.WritePoint(tmp);
        }
        for(const pcl::PointXYZ &pt : left_cloud->points)
        {
            liblas::Point tmp(&header);
            tmp.SetCoordinates(pt.x,pt.y,pt.z);
            tmp.SetIntensity(100);
            tmp.SetColor(left_color);
            writer.WritePoint(tmp);
        }
        for(const pcl::PointXYZ &pt : right_cloud->points)
        {
            liblas::Point tmp(&header);
            tmp.SetCoordinates(pt.x,pt.y,pt.z);
            tmp.SetIntensity(100);
            tmp.SetColor(right_color);
            writer.WritePoint(tmp);
        }

    }

    void WriteLas(CloudXYZPtr line_cloud,const std::string &lasfile)
    {
        if(line_cloud->empty())
            return;

        std::ofstream ofs(lasfile);
        if (!ofs.is_open())
        {
            std::cerr << "Create Las File Failed." << std::endl;
            return;
        }

        liblas::Header header;
        header.SetDataFormatId(liblas::ePointFormat2); // Time only

        header.SetScale(0.01, 0.01, 0.01);

        // Set coordinate system using GDAL support
        liblas::SpatialReference srs;
        srs.SetFromUserInput("EPSG:4326");
        header.SetSRS(srs);
        liblas::Writer writer(ofs, header);

        liblas::Color line_color(255,0,0);
        liblas::Color left_color(0,255,0);
        liblas::Color right_color(0,0,255);

        for(const pcl::PointXYZ &pt : line_cloud->points)
        {
            liblas::Point tmp(&header);
            tmp.SetCoordinates(pt.x,pt.y,pt.z);
            tmp.SetIntensity(100);
            tmp.SetColor(line_color);
            writer.WritePoint(tmp);
        }
    }

    void ConditionFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        /// condition removal filter
        pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond_and(new pcl::ConditionAnd<pcl::PointXYZI>);

        /// z point to front
        pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_z_1(
                new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, -20.0));
        range_cond_and->addComparison(cond_z_1);
        pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_z_2(
                new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, 20.0));
        range_cond_and->addComparison(cond_z_2);

        /// y point to bottom
        pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_y_1(
                new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, 3.0));
        range_cond_and->addComparison(cond_y_1);
        pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_y_2(
                new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, -10.0));
        range_cond_and->addComparison(cond_y_2);

        /// x point to right
        pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_x_1(
                new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, -20.0));
        range_cond_and->addComparison(cond_x_1);
        pcl::FieldComparison<pcl::PointXYZI>::ConstPtr cond_x_2(
                new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, 20.0));
        range_cond_and->addComparison(cond_x_2);

        pcl::ConditionalRemoval<pcl::PointXYZI> conditionalRemoval;
        conditionalRemoval.setInputCloud(cloud);
        conditionalRemoval.setCondition(range_cond_and);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cond(new pcl::PointCloud<pcl::PointXYZI>);
        conditionalRemoval.filter(*cloud_cond);
        std::cout << "Condition Removal Result Size ~ " << cloud_cond->size() << std::endl;

        cloud->swap(*cloud_cond);

//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_r(new pcl::PointCloud<pcl::PointXYZI>);
//        for(const pcl::PointXYZI &pt : cloud->points)
//        {
//            if(pt.intensity!=0.0f)
//                cloud_r->push_back(pt);
//        }
//        cloud->swap(*cloud_r);

    }

    void StatisticFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> stat_removal;
        stat_removal.setMeanK(100);
        stat_removal.setStddevMulThresh(1);
        stat_removal.setInputCloud(cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stat(new pcl::PointCloud<pcl::PointXYZI>);
        stat_removal.filter(*cloud_stat);
        std::cout << "Statistic Filter Result Size ~ " << cloud_stat->size() << std::endl;

        cloud->swap(*cloud_stat);
    }

    void VoxelFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZI>);
        voxel_filter.filter(*cloud_voxel);
        std::cout << "Voxel Filter Result Size ~ " << cloud_voxel->size() << std::endl;

        cloud->swap(*cloud_voxel);
    }

    void MlsFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>::Ptr mls(
                new pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>);
        mls->setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree;
        mls->setSearchMethod(kdtree);
        mls->setSearchRadius(0.05);
        mls->setComputeNormals(true);
        mls->setPolynomialFit(true);
        mls->setPolynomialOrder(4);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mls(new pcl::PointCloud<pcl::PointXYZI>);
        mls->process(*cloud_mls);
        std::cout << "MlS Smooth Result Size ~ " << cloud_mls->points.size() << std::endl;

        cloud->swap(*cloud_mls);
    }

    void RansacRemoveGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        /// estimate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        normalEstimation.setSearchMethod(kdtree);
        normalEstimation.setRadiusSearch(0.3);
        normalEstimation.compute(*normals);

        /// planar segmentation
        pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> segmentationFromNormals;
        segmentationFromNormals.setOptimizeCoefficients(true);
        segmentationFromNormals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        segmentationFromNormals.setNormalDistanceWeight(0.3);
        segmentationFromNormals.setMethodType(pcl::SAC_RANSAC);
        segmentationFromNormals.setMaxIterations(1000);
        segmentationFromNormals.setDistanceThreshold(0.3);
        segmentationFromNormals.setInputCloud(cloud);
        segmentationFromNormals.setInputNormals(normals);

        /// segmentation
        pcl::ModelCoefficients::Ptr coefficients_ground(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_ground(new pcl::PointIndices);
        segmentationFromNormals.segment(*inliers_ground, *coefficients_ground);
        std::cout << "Ground coefficients ~ " << *coefficients_ground << std::endl;

        /// extract ground
        pcl::ExtractIndices<pcl::PointXYZI> extract_ground(true);
        extract_ground.setInputCloud(cloud);
        extract_ground.setIndices(inliers_ground);
        extract_ground.setNegative(false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>);
        extract_ground.filter(*cloud_ground);

        /// extract plane indices
        pcl::IndicesConstPtr inliers_plane;
        inliers_plane = extract_ground.getRemovedIndices();

        /// extract plane
        pcl::ExtractIndices<pcl::PointXYZI> extract_plane(true);
        extract_plane.setInputCloud(cloud);
        extract_plane.setIndices(inliers_plane);
        extract_plane.setNegative(false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
        extract_plane.filter(*cloud_plane);
        std::cout << "Ground Cloud Size ~ " << cloud_ground->size() << std::endl;
        std::cout << "Plane Cloud Size ~ " << cloud_plane->size() << std::endl;

        cloud->swap(*cloud_plane);
    }

    std::vector<int> UniqueIndex(std::vector<int> vec)
    {
        std::vector<int> result;
        std::sort(vec.begin(), vec.end());
        int last_index = -1000000;
        for (const int index : vec)
        {
            if (index != last_index)
                result.push_back(index);
            last_index = index;
        }
        return result;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    TransformCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Sophus::SE3f &se)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
        for (const pcl::PointXYZI &pt : cloud->points)
        {
            Eigen::Vector3f pos(pt.x, pt.y, pt.z);
            Eigen::Vector3f pos_trans = se * pos;

            pcl::PointXYZI p;
            p.x = pos_trans(0);
            p.y = pos_trans(1);
            p.z = pos_trans(2);
            p.intensity = pt.intensity;
            cloud_trans->push_back(p);
        }
        return cloud_trans;
    }

    std::vector<pcl::PointXYZI> TransformCloud(const std::vector<pcl::PointXYZI> &vec, const Sophus::SE3f &se)
    {
        std::vector<pcl::PointXYZI> result;
        for (const pcl::PointXYZI &pt : vec)
        {
            Eigen::Vector3f pos(pt.x, pt.y, pt.z);
            Eigen::Vector3f pos_trans = se * pos;

            pcl::PointXYZI p;
            p.x = pos_trans(0);
            p.y = pos_trans(1);
            p.z = pos_trans(2);
            p.intensity = pt.intensity;
            result.push_back(p);
        }
        return result;
    }

//    std::vector<pcl::PointXYZI> SearchByPos(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const PosVec &vec)
    std::vector<int> SearchByPos(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,const PosVec &vec)
    {
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZI>);
        flann->setInputCloud(cloud);

        const float radius = 0.05f;
        std::vector<int> indices;
        for (const Eigen::Vector3f &pos : vec)
        {
            pcl::PointXYZI pt;
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);

            std::vector<int> vindices;
            std::vector<float> vdistance;
            flann->radiusSearch(pt, radius, vindices, vdistance);

            indices.insert(indices.end(), vindices.begin(), vindices.end());
        }

        std::sort(indices.begin(), indices.end());
        std::vector<int> tmp_indices;
        int last_index = -10000000;
        for (const int &tmp_index : indices)
        {
            if (tmp_index != last_index)
                tmp_indices.push_back(tmp_index);
            last_index = tmp_index;
        }
        return tmp_indices;
    }

    Eigen::Matrix3f CalRotation(const Eigen::Vector3f &azix, const float angle)
    {
        const float a = azix(0), b = azix(1), c = azix(2);
        const float eps = 0.00000000001f;

        float sqrt_bb_cc = std::sqrt(b * b + c * c);
        if (sqrt_bb_cc == 0.0f)
            sqrt_bb_cc += eps;

        float sqrt_aa_bb_cc = std::sqrt(a * a + b * b + c * c);
        if (sqrt_aa_bb_cc == 0.0f)
            sqrt_aa_bb_cc += eps;

        float cos_alpha = c / sqrt_bb_cc;
        float sin_alpha = b / sqrt_bb_cc;
        float cos_beta = sqrt_bb_cc / sqrt_aa_bb_cc;
        float sin_beta = a / sqrt_aa_bb_cc;
        float cos_theta = std::cos(angle);
        float sin_theta = std::sin(angle);

        Eigen::Matrix3f rx_neg_alpha = Eigen::Matrix3f::Identity();
        rx_neg_alpha(1, 1) = cos_alpha;
        rx_neg_alpha(1, 2) = sin_alpha;
        rx_neg_alpha(2, 1) = -sin_alpha;
        rx_neg_alpha(2, 2) = cos_alpha;
//    std::cout<<"rx_neg_alpha ~ \n"<<rx_neg_alpha<<std::endl;

        Eigen::Matrix3f ry_beta = Eigen::Matrix3f::Identity();
        ry_beta(0, 0) = cos_beta;
        ry_beta(0, 2) = sin_beta;
        ry_beta(2, 0) = -sin_beta;
        ry_beta(2, 2) = cos_beta;
//    std::cout<<"ry_beta ~ \n"<<ry_beta<<std::endl;

        Eigen::Matrix3f rz_theta = Eigen::Matrix3f::Identity();
        rz_theta(0, 0) = cos_theta;
        rz_theta(0, 1) = sin_theta;
        rz_theta(1, 0) = -sin_theta;
        rz_theta(1, 1) = cos_theta;
//    std::cout<<"rz_theta ~ \n"<<rz_theta<<std::endl;

        Eigen::Matrix3f ry_neg_beta = Eigen::Matrix3f::Identity();
        ry_neg_beta(0, 0) = cos_beta;
        ry_neg_beta(0, 2) = -sin_beta;
        ry_neg_beta(2, 0) = sin_beta;
        ry_neg_beta(2, 2) = cos_beta;
//    std::cout<<"ry_neg_beta ~ \n"<<ry_neg_beta<<std::endl;

        Eigen::Matrix3f rx_alpha = Eigen::Matrix3f::Identity();
        rx_alpha(1, 1) = cos_alpha;
        rx_alpha(1, 2) = -sin_alpha;
        rx_alpha(2, 1) = sin_alpha;
        rx_alpha(2, 2) = cos_alpha;
//    std::cout<<"rx_alpha ~ \n"<<rx_alpha<<std::endl;

        Eigen::Matrix3f rotate = rx_neg_alpha * ry_beta * rz_theta * ry_neg_beta * rx_alpha;
        return rotate;
    }

    Eigen::Matrix3f CalRotation(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2)
    {
        float product = vec1.transpose() * vec2;
        float angle = std::acos(product / (vec1.norm() * vec2.norm()));

        float a1 = vec1(0), a2 = vec1(1), a3 = vec1(2);
        float b1 = vec2(0), b2 = vec2(1), b3 = vec2(2);
        Eigen::Vector3f azix(a2 * b3 - a3 * b2, a3 * b1 - a1 * b3, a1 * b2 - a2 * b1);
        azix.normalize();
        float w1 = azix(0), w2 = azix(1), w3 = azix(2);

        Eigen::Matrix3f skew_symmetry;
        skew_symmetry << 0.0f, -w3, w2, w3, 0.0f, -w1, -w2, w1, 0.0f;

        Eigen::Matrix3f rotate = Eigen::Matrix3f::Identity() + skew_symmetry * std::sin(angle) +
                                 skew_symmetry * skew_symmetry * (1.0f - std::cos(angle));
        return rotate;
    }

    CloudXYZPtr SearchClosestWithinRadius(CloudXYZPtr cloud, CloudXYZPtr current)
    {
        if (current->empty())
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        flann->setInputCloud(cloud);
        flann->setSortedResults(true);

        const float radius = 0.05f;
        std::vector<int> search_indices;
        for (const pcl::PointXYZ &pt : current->points)
        {
            std::vector<int> vindices;
            std::vector<float> vdistance;
            flann->radiusSearch(pt, radius, vindices, vdistance);

            if (vindices.empty())
                continue;

            search_indices.push_back(vindices.front());
        }

        if (search_indices.empty())
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        std::sort(search_indices.begin(), search_indices.end());
        std::vector<int> unique_indices;
        int last_index = -100000000;
        for (const int &index : search_indices)
        {
            if (last_index != index)
                unique_indices.push_back(index);
            last_index = index;
        }

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        indices->indices.swap(unique_indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *indices, *cloud_search);
        return cloud_search;
    }

    CloudXYZPtr SearchWithinRadius(CloudXYZPtr cloud, CloudXYZPtr current)
    {
        if (current->empty())
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        flann->setInputCloud(cloud);

        const float radius = 0.05f;
        std::vector<int> search_indices;
        for (const pcl::PointXYZ &pt : current->points)
        {
            std::vector<int> vindices;
            std::vector<float> vdistance;
            flann->radiusSearch(pt, radius, vindices, vdistance);

            if (vindices.empty())
                continue;

            search_indices.insert(search_indices.end(),vindices.begin(),vindices.end());
        }

        if (search_indices.empty())
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        std::sort(search_indices.begin(), search_indices.end());
        std::vector<int> unique_indices;
        int last_index = -100000000;
        for (const int &index : search_indices)
        {
            if (last_index != index)
                unique_indices.push_back(index);
            last_index = index;
        }

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        indices->indices.swap(unique_indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_search(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *indices, *cloud_search);
        return cloud_search;
    }

    CloudXYZPtr SearchKnnMeans(CloudXYZPtr cloud, CloudXYZPtr current)
    {
        if (current->empty())
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        flann->setInputCloud(cloud);

        const int K = 3;
        PCM::PosVec pos_vec;
        for (const pcl::PointXYZ &pt : current->points)
        {
            std::vector<int> vindices;
            std::vector<float> vdistance;
            flann->nearestKSearch(pt, K, vindices, vdistance);

            Eigen::Vector3f pos(0.0f, 0.0f, 0.0f);
            for (const int &index : vindices)
            {
                pcl::PointXYZ pt_index = cloud->points[index];
                pos += Eigen::Vector3f(pt_index.x, pt_index.y, pt_index.z);
            }
            pos /= K;
            pos_vec.push_back(pos);
        }

        std::sort(pos_vec.begin(), pos_vec.end(), [](const Eigen::Vector3f &pos1, const Eigen::Vector3f &pos2) {
            return pos1(0) < pos2(0) || (pos1(0) == pos2(0) && pos1(1) < pos2(1)) ||
                   (pos1(0) == pos2(0) && pos1(1) == pos2(1) && pos1(2) < pos2(2));
        });

        float x_last = 1000000000.0f, y_last = 1000000000.0f, z_last = 100000000000.0f;
        PCM::CloudXYZPtr result(new pcl::PointCloud<pcl::PointXYZ>);
        for (const Eigen::Vector3f &pos : pos_vec)
        {
            if (x_last != pos(0) || y_last != pos(1) || z_last != pos(2))
                result->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            x_last = pos(0);
            y_last = pos(1);
            z_last = pos(2);
        }
        return result;
    }

    CloudXYZPtr SacLine(CloudXYZPtr cloud, std::vector<float> &coeff)
    {
        if (cloud->size() < 4)
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::SACSegmentation<pcl::PointXYZ> sac;
        sac.setInputCloud(cloud);
        sac.setOptimizeCoefficients(true);
        sac.setModelType(pcl::SACMODEL_LINE);
        sac.setMethodType(pcl::SAC_RANSAC);
        sac.setDistanceThreshold(0.05f);
        sac.setMaxIterations(10000);

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
        sac.segment(*indices, *coefficents);

        if (indices->indices.size() < 4)
        {
//            std::cerr << "Sac Line Failed." << std::endl;
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

        coeff.assign(coefficents->values.begin(), coefficents->values.end());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *indices, *cloud_edge);
        return cloud_edge;
    }

    CloudXYZPtr SacPlanar(CloudXYZPtr cloud, std::vector<float> &coeff)
    {
        if (cloud->size() < 10)
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::SACSegmentation<pcl::PointXYZ> sac;
        sac.setInputCloud(cloud);
        sac.setOptimizeCoefficients(true);
        sac.setModelType(pcl::SACMODEL_PLANE);
        sac.setMethodType(pcl::SAC_RANSAC);
        sac.setDistanceThreshold(0.03f);
        sac.setMaxIterations(10000);

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficents(new pcl::ModelCoefficients);
        sac.segment(*indices, *coefficents);

        if (indices->indices.size() < 10)
        {
//            std::cerr << "Sac Planar Failed." << std::endl;
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

        coeff.assign(coefficents->values.begin(), coefficents->values.end());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planar(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, *indices, *cloud_planar);
        return cloud_planar;
    }

    CloudXYZPtr InterpolateLine(CloudXYZPtr cloud, const std::vector<float> &coeff)
    {
        Eigen::Vector3f line_pt(coeff[0], coeff[1], coeff[2]);
        Eigen::Vector3f line_dir(coeff[3], coeff[4], coeff[5]);
        float max_dis = -100000000000.0f, min_dis = 10000000000000.0f;
        for (const pcl::PointXYZ &pt : cloud->points)
        {
            Eigen::Vector3f vec = Eigen::Vector3f(pt.x, pt.y, pt.z) - line_pt;
            float dis = line_dir.dot(vec) / line_dir.norm();
            min_dis = (min_dis < dis) ? min_dis : dis;
            max_dis = (max_dis > dis) ? max_dis : dis;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_straghit_edge(new pcl::PointCloud<pcl::PointXYZ>);
        for (float dis = min_dis; dis <= max_dis; dis += 0.05f)
        {
            Eigen::Vector3f pos = line_pt + dis * line_dir;
            cloud_straghit_edge->push_back(pcl::PointXYZ(pos(0),pos(1),pos(2)));
        }
        return cloud_straghit_edge;
    }

    CloudXYZPtr Project2Planar(CloudXYZPtr cloud, const std::vector<float> &coeff)
    {
        float nx = coeff[0], ny = coeff[1], nz = coeff[2], p = coeff[3];

        PCM::CloudXYZPtr project(new pcl::PointCloud<pcl::PointXYZ>);
        for (const pcl::PointXYZ &pt : cloud->points)
        {
            float x = pt.x, y = pt.y, z = pt.z;
            float t = (nx * x + ny * y + nz * z + p) / (nx * nx + ny * ny + nz * nz);

            float xp = x - nx * t;
            float yp = y - ny * t;
            float zp = z - nz * t;

            pcl::PointXYZ tmp(xp, yp, zp);
            project->push_back(tmp);
        }

        return project;
    }

    float PlanarDensity(CloudXYZPtr cloud)
    {
        float minx = 1000000000.0f, maxx = -10000000000.0f;
        float miny = 1000000000.0f, maxy = -10000000000.0f;
        float minz = 1000000000.0f, maxz = -10000000000.0f;

        for (const pcl::PointXYZ &pt : cloud->points)
        {
            minx = minx < pt.x ? minx : pt.x;
            maxx = maxx > pt.x ? maxx : pt.x;
            miny = miny < pt.y ? miny : pt.y;
            maxy = maxy > pt.y ? maxy : pt.y;
            minz = minz < pt.z ? minz : pt.z;
            maxz = maxz > pt.z ? maxz : pt.z;
        }

        float volume = (maxx - minx) * (maxy - miny) * (maxz - minz);
        return cloud->size() / (volume + 0.00000001f);
    }

//    pcl::PointCloud<pcl::PointXYZI>::Ptr
//    ExtractEdge(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::vector<std::vector<int>> &edge_vec)
//    {
//        std::map<int,int> edge_map;
//        for(const std::vector<int> &edge : edge_vec)
//        {
//            for(const int index : edge)
//                edge_map[index]++;
//        }
//
//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZI>);
//        for(const std::pair<int,int> &pair : edge_map)
//        {
//            int index=pair.first;
//            int num=pair.second;
//
//            pcl::PointXYZI pt=cloud->points[index];
//            pt.intensity=num;
//            cloud_edge->push_back(pt);
//        }
//        return cloud_edge;
//
//    }



//    pcl::PointCloud<pcl::PointXYZI>::Ptr LSR3d(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,const PosVec &vec)
//    {
//        std::vector<int> indices;
//        const float radius=0.05f;
//
//        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr flann(new pcl::KdTreeFLANN<pcl::PointXYZI>);
//        flann->setInputCloud(cloud);
//        for(const Eigen::Vector3f &pos : vec)
//        {
//            pcl::PointXYZI pt;
//            pt.x=pos(0);
//            pt.y=pos(1);
//            pt.z=pos(2);
//            pt.intensity=1;
//
//            std::vector<int> vindices;
//            std::vector<float> vdistance;
//            flann->radiusSearch(pt,radius,vindices,vdistance);
//
//            indices.insert(indices.end(),vindices.begin(),vindices.end());
//        }
//
//        std::sort(indices.begin(),indices.end());
//        int last_index=-100000000;
//        std::vector<int> tmp_indices;
//        for(const int &index : indices)
//        {
//            if(index!=last_index)
//                tmp_indices.push_back(index);
//            last_index=index;
//        }
//        indices.swap(tmp_indices);
//
//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZI>);
//        pcl::copyPointCloud(*cloud,indices,*cloud_result);
//        return cloud_result;
//    }
//
//    Eigen::Vector3f ProjectDirection(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
//    {
//        std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> normals;
//        for(const pcl::PointXYZINormal &pt : cloud->points)
//            normals.push_back(Eigen::Vector3f(pt.normal_x,pt.normal_y,pt.normal_z));
//
//        Eigen::Vector3f seed1=normals[0];
//        Eigen::Vector3f seed2=normals[1];
//
//        /// simple kmeans
//        while(true)
//        {
//            Eigen::Vector3f seed1_update(0.0f, 0.0f, 0.0f), seed2_update(0.0f, 0.0f, 0.0f);
//            int seed1_num = 0, seed2_num = 0;
//            for (const Eigen::Vector3f &normal : normals)
//            {
//                float dis1 = (normal - seed1).norm();
//                float dis2 = (normal - seed2).norm();
//
//                if (dis1 < dis2)
//                {
//                    seed1_update += normal;
//                    seed1_num++;
//                }
//                else
//                {
//                    seed2_update += normal;
//                    seed2_num++;
//                }
//            }
//
//            seed1_update /= seed1_num;
//            seed2_update /= seed2_num;
//
//            if(seed1==seed1_update && seed2==seed2_update)
//                break;
//
//            seed1=seed1_update;
//            seed2=seed2_update;
//        }
////        Eigen::Cross
//        Eigen::Vector3f project_direction=seed1.cross(seed2);
//
//        return project_direction;
//    }


}