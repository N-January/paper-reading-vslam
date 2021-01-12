# paper-reading-vslam
- Direct Sparse Odometry(TPAMI 2017)
  - DSO-Initial
  - DSO-Tracking
  
- LDSO: Direct Sparse Odometry with Loop Closure(IROS 2018)

- Direct Sparse Visual-Inertial Odometry using Dynamic Marginalization(ICRA2018)

- Decoupled, Consistent Node Removal and Edge Sparsification for Graph-based SLAM(IROS2016)

- Depth Filter

- Line Feature

- Match or No Match: Keypoint Filtering based on Matching Probability(CVPR2020)

  本文主要基于两点，一是错误的特征点匹配会降低图像匹配精度，二是特征点并不是越多越好，那么提出一种思路，在特征点提取和匹配之间加入基于二分类随机森林分类器的过滤模块，随机森林用于预测该特征点成功匹配的概率，主要是过滤掉容易匹配失败和并不能提升匹配效果反而增加算力的特征点。以城市环境的三维重建为例，需要过滤的特征点主要包括植被、重复纹理等；本文挑选了八个特征: $$u_x$$、$$u_y$$、size、orientation、response、octave、the number of dominant orientations、intensity of the green channel，其中$$u_x$$、$$u_y$$是指在图像中的坐标信息，图像边缘会比图像中间与其他图像匹配时有更高概率的overlap；size、orientation、response、octave来自特征点提取器(比如SIFT)中的信息，分别表示领域大小、方向、被检测为关键点概率、被检测时所在金字塔层数；the number of dominant orientations表示纹理复杂度以及关键点方向准确性的度量，intensity of the green channel为了过滤植被。

  笔者说：本文的思路很值得学习，虽然比较考验过滤策略的设计，这里稍微补充三点，a. 关于特征中$$u_x$$和$$u_y$$的引入，至于是否更大概率overlap的处理有些粗糙，边缘大于中间的先验不一定成立，应该结合序列动态调整大概率overlap区域；b. 对于植被的过滤操作，单纯绿色通道欠妥；c. 除了二分的思路之外，对于特征点不足的场景，也可以多保留一些点，并引入置信度权重，提升鲁棒性的前提下稍微牺牲精度。

- Faster than FAST: GPU-Accelerated Frontend for High-Speed VIO(IROS2020, ETHZ)

  本文主要针对CPU-GPU这种异构系统对前端进行优化，充分结合GPU硬件结构改进了特征提取算法。本文的特征提取过程同步实现了非极大值抑制、特征响应函数计算，同时兼顾均匀特征分布。本文在TX2平台上可将前端提速到1kHz，结合ICE-BA可实现200Hz的VIO。

  特征提取，毫无疑问目前FAST是大哥，CPU版目前KFAST最快，GPU版目前OpenCV和ArrayFIre最优。GPU版的两种方法都是通过查表的方式进行加速，前者使用8K的表后者64K。但是这些方法都是当特征点数达到数目后停止提取而未考虑特征点的分布问题。

  非极大值抑制，相关的方法有一些，但是都大同小异，无非是对于"抑制候选范围"的定义不同，比如候选点半径、个数、pattern(间隔点、scan-line、quarter-block)形式等，这些方法同样不能保证点特征的分布以及有可能输出很多的候选点。(Match or No Match也算是一种同类方法吧，只是它算是对特征的有效性的非极大值抑制，总之都是提升特征鲁棒性、准确性的一种方法。)

  特征跟踪，主要三种策略，feature matching、filter-based tracking、differential tracking。

  本文的并行化策略，这一块主要讲解如何将特征提取算法和cuda特性相结合，包括结合warp中thread数量去分析图像、为非极大值抑制设计pattern、查表法的FAST特征提取等，论文讲得比较细详见论文。

  本文的实际效果，速度毋庸置疑的提升，只是精度方面会稍微差一些，本文的前端+ICE-BA的后端和完整的ICE-BA相比，在EuRoC上，一些case最多差一个点，关于差别作者给出的解释是，在一些黑暗、快速运动的场景引入了跟踪噪声。

  笔者说：本文结合cuda特性优化了特征提取算法，这个思路比较常规，只是圈内在这个层面的研究已经很少了，毕竟API快乐(皮一下)。关于帧率提升意义的思考，对于实时的定义，我想这个取决于应用场景，对于低速自主机器人等来说，可能10Hz、30Hz级别已经够用，另外降帧率也是一种节约算力的方法，简单粗暴效果爆炸；但是对于一些人机交互的行为，比如AR/VR，或许还是希望帧率稍高一些，定位信息更新的频率影响交互体验，当然也有基于IMU预估位姿的方法，只是如果能够准确计算那高帧率也是挺好的，虽然说成本挺高。对于作者的思路再提炼一下，一个共性的思路是，结合业务的特化优化效果显著，即结合业务将多个环节在一个环节进行实现是很有意义的。另外想重点说一下效果，竟然会比CPU方法差一个点，作者并没有给出更细致的解释，但是是值得深究的，笔者认为可能的原因是，在作者的实现中，比如非最大值抑制环节为了并行还是有一些近似操作，以及作者实现的前端仅基于LK光流，还不能算得上完整的前端，所以精度可能会差一些，毕竟完整SLAM框架里的各种trick都是很有意义的，当然可能本文重点强调的特征点分布更加均匀进而实现鲁棒性提升的点可能没有体现出来，毕竟没有鲁棒就没有精度。最后，这也算是一篇吹ICE-BA的文章。

- PL-VINS: Real-Time Monocular Visual-Inertial SLAM with Point and Line Features(ICRA2020)