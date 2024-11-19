# 点特征直方图（PFH）描述符

> 参考文档：https://pcl.readthedocs.io/projects/tutorials/en/latest/pfh_estimation.html#pfh-estimation

## 理论入门

多维直方图概括点周围平均曲率编码点的k邻域几何属性

能应对采样密度差异和噪声水平

法线作用捕捉表面变化

超空间取决于每个点表面法线估计质量

![图片/pfh_diagram.png](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/pfh_diagram.png)

<center>图1.PFH计算影响区域图</center>

如图1所示，查询点为红色，邻域点为蓝色，计算邻域点对的关系直方图，计算复杂度$O(k^2)$

![图片/pfh_frame.png](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/pfh_frame.png)

<center>图2.邻域点的关系空间</center>

角度特性：

$\alpha=v\cdot n_{t}$

$\displaystyle\phi=u\cdot\frac{p_{t}-p_{s}}{d}$

$\theta=\arctan(w\cdot n_{t},u\cdot n_{t})$

距离特性：

$d=\left\lVert p_{t}-p_{s} \right\rVert_{2}$

四元组PFH

```C++
computePairFeatures(cosnt Eigen::Vector4f &p1,const Eigen::Vector4f &n1,const Eigen::Vector4f &p2,const Eigen::Vector4f &n2,float &f1,float &f2,float &f3,float &f4);
```

## PFH特征估计

默认PFH使用5个分箱细分，不包括距离

```C++
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>

{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    
    //read,pass in or create a point cloud with normals
    
    pcl::PFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    
    pcl::search::Kdtree<pcl::PointXYZ>::Ptr tree(new pcl::search::Kdtree<pcl::PointXYZ>());
    pfh.setSearchMethod(tree);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125> ());
    pfh.setRadiusSearch(0.05);
    pfh.compute(*pfhs);
}
```

