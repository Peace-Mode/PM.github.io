# 快速点特征直方图（FPFH）描述符

> https://pcl.readthedocs.io/projects/tutorials/en/latest/fpfh_estimation.html

将PFH算法复杂度降低到$O(nk)$，同时保留PFH大部分判别能力

## 理论基础

两步操作简化：

1.对每个查询点，计算其自身与邻居间的一组元组，如PFH中所描述，称为简化点特征直方图（SPFH）；

2.每个点重新确定邻域，使用邻域SPFH重新加权最终的直方图（FPFH）

$\displaystyle FPFH(p_q)=SPFH(P_q)+\frac{1}{k}\sum\limits_{i=1}^{k}\frac{1}{\omega_i}\cdot SPFH(p_i)$

其中$\omega_i$表示查询点到度量空间邻域点$p_q$间的距离

![图片/fpfh_diagram.png](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/fpfh_diagram.png)

<center>图1.FPFH</center>

## PFH与FPFH差异

1.FPFH可能缺少查询周围几何形状的值对

2.PFH查询面精确，FPFH可能存在多点对

3.FPFH复杂度大大降低，提高效率

## FPFH特征估计

```C++
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::Point<pcl::Normal>());
    
    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33>fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    
    pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
    
    fpfh.setSearchMethod(tree);
    
    pcl::PointCloud<pcl;:FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    
    fpfh.setRadiusSearch(0.05);
    
    fpfh.comput(*fpfhs);
}
```

