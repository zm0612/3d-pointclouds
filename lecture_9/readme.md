代码所用测试数据下载地址
链接：https://pan.baidu.com/s/11OhSCA2Ck-WA6_b-iCNsfA 
提取码：hyi3



## 思路一

1. 首先提取特征点
2. 计算描述子
3. 进行描述子匹配。只进行单项匹配
4. 然后计算ransac icp，迭代几万甚至几十万次，找到`o3d.pipelines.registration.evaluate_registration`评分中fitness最大的那个，它对应的`R`和`t`就是最终的ransac结果。
5. 以ransac的匹配结果，为初始匹配，使用icp进行精细匹配

> 该思路经过验证，成功率非常低，主要原因是fitness最大时，并不代表就是一个真正好的匹配结果。所以收敛条件判定不合理。



## 思路二

1. 提取特征点
2. 计算描述子
3. 进行描述子匹配。进行双向匹配，一个两个描述子相互匹配之后都是最近邻时，才认定为一次正确的匹配。
4. 然后通过ransac icp，迭代计算`R`和`t`，找到inliers数量最多的那个`R`和`t`
5. 以上一步的`R`和`t`为初值，执行精细的icp匹配。