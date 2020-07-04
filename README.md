# paper-reading-vslam
- Direct Sparse Odometry(TPAMI 2017)
  - DSO-Initial.
  - DSO-Tracking.
  
- LDSO: Direct Sparse Odometry with Loop Closure(IROS 2018)

- Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization(ICRA2018)

- Decoupled, Consistent Node Removal and Edge Sparsification for Graph-based SLAM(IROS2016)

- Depth Filter.

- Line Feature.

- Match or No Match: Keypoint Filtering based on Matching Probability(CVPR2020)

  本文主要基于两点，一是错误的特征点匹配会降低图像匹配精度，二是特征点并不是越多越好，那么提出一种思路，在特征点提取和匹配之间加入基于二分类随机森林分类器的过滤模块，随机森林用于预测该特征点成功匹配的概率，主要是过滤掉容易匹配失败和并不能提升匹配效果反而增加算力的特征点。以城市环境的三维重建为例，需要过滤的特征点主要包括植被、重复纹理等；本文挑选了八个特征: $$u_x$$、$$u_y$$、size、orientation、response、octave、the number of dominant orientations、intensity of the green channel，其中$$u_x$$、$$u_y$$是指在图像中的坐标信息，图像边缘会比图像中间与其他图像匹配时有更高概率的overlap；size、orientation、response、octave来自特征点提取器(比如SIFT)中的信息，分别表示领域大小、方向、被检测为关键点概率、被检测时所在金字塔层数；the number of dominant orientations表示纹理复杂度以及关键点方向准确性的度量，intensity of the green channel为了过滤植被。

  笔者说：本文的思路很值得学习，虽然比较考验过滤策略的设计，这里稍微补充三点，a. 关于特征中$$u_x$$和$$u_y$$的引入，至于是否更大概率overlap的处理有些粗糙，边缘大于中间的先验不一定成立，应该结合序列动态调整大概率overlap区域；b. 对于植被的过滤操作，单纯绿色通道欠妥；c. 除了二分的思路之外，对于特征点不足的场景，也可以多保留一些点，并引入置信度权重，提升鲁棒性的前提下稍微牺牲精度。