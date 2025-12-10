#include "ch5_my/my_bfnn.h"
#include <execution>

namespace sad{

// 单线程之间的匹配
int bfnn_point(CloudPtr cloud, const Vec3f& point) {
    return std::min_element(cloud->points.begin(), cloud->points.end(),
                   // lambda表达式 &point 这样可以访问point里面的数值
                    [&point](const PointType& pt1, const PointType& pt2) -> bool {   
                        // pt1.getVector3fMap()                          返回点云的向量对象
                        // pt1.getVector3fMap() - point                  用于计算pt1 和 目标点point之间的向量差
                        // (pt1.getVector3fMap() - point).squaredNorm()  上述向量差的二范数，也就是平方
                        // < 如果 p1 的距离更近，就返回true
                        return  (pt1.getVector3fMap() - point).squaredNorm() <
                                (pt2.getVector3fMap() - point).squaredNorm();
                    }) -                                               // 找到一个点，使得它比任何点里point都近
            cloud->points.begin();                                     // 返回迭代器的索引，整数
}

std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f& point, int k ){
    // 用于存储索引点和目标之间的距离关系，距离采取的是二范数
    struct IndexAndDis {
        IndexAndDis() {}
        IndexAndDis(int index, double dis2) : index_(index), dis2_(dis2) {}
        int index_ = 0;
        double dis2_ = 0;
    };
    
    // 遍历得到点云每个点和目标点之间的二范数
    std::vector<IndexAndDis> index_and_dis(cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        index_and_dis[i] = {i, (cloud->points[i].getVector3fMap() - point).squaredNorm()};
    }
    
    // 排序结果 将距离结果从小到大的排序
    std::sort(index_and_dis.begin(), index_and_dis.end(),
                [](const auto& d1, const auto& d2 ) {return d1.dis2_ < d2.dis2_;});
    std::vector<int> ret;

    // 将距离前k个结果返回
    // transform将一个数组变换得到另一个数组
    // index_and_dis的前k个数组转换到 ret里面去，自动从末尾插入，避免覆盖
    // std::back_inserter 将结果插入到后面
    std::transform(index_and_dis.begin(), index_and_dis.begin() + k, std::back_inserter(ret),
                [](const auto& d1) {return d1.index_;});
    return ret;
}


void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {
    // 单线程版本
    std::vector<size_t> index(cloud2->size());
    // for_each 对迭代器每个元素都执行相同的操作
    // idx = 0 初始化捕获列表 mutable 就是允许模板捕获变量idx
    // 最终的结果赋值为0, 1,2,3, cloud->size()-1
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable {i = idx++;});

    matches.resize(index.size());

    // 这个是seq就是顺序执行，而非并行执行，类似for
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx){
        // 这里第二个元素为cloud2的点idx，第一个元素为
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}


// 多线程的匹配cloud1和cloud2的关系  
// 输入两个点云指针cloud1 cloud2，输出他们之间的匹配关系 mtches
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches) {

    // 生成索引
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(),[idx = 0](size_t & i) mutable { i = idx++; });

    // 并行化 for_each
    matches.resize(index.size());
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx){
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}



}
