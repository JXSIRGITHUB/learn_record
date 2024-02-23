# Reactive Base Control for On-The-Move Mobile Manipulation in Dynamic Environments

在动态环境中的移动操作中的反应性基座控制

**摘要—**我们提出了一种反应性基座控制方法，使得机器人在移动中可以高效地进行移动操作，同时在静态和动态障碍物环境中操作。在机器人基座保持运动的同时执行操作任务可以显著减少执行多步任务所需的时间，同时提高机器人运动的优雅性。现有的在移动中进行操作的方法要么忽视障碍物避免问题，要么依赖于执行计划轨迹，这在存在动态物体和障碍物的环境中并不合适。所提出的控制器解决了这两个问题，并展示了在动态环境中进行拾取和放置任务的稳健性能。该性能在几个模拟和真实世界任务上进行了评估。在一个具有静态障碍物的真实任务中，我们在总任务时间方面比现有方法提高了48％。此外，我们提供了我们的机器人在工作区域中避开第二个自主机器人进行操作任务的实际示例。请参阅 benburgesslimerick.github.io/MotM-BaseControl 获取补充材料。

## I. INTRODUCTION

在机器人基座保持运动的同时执行移动操作任务，与机器人暂停执行操作任务相比，可以显著减少执行时间。这在多步任务中特别有价值，例如，机器人可以在拾取物体的同时朝着第二个位置行驶。最近的工作已经探索了实现这种“移动操作”（MotM）的控制方法。

依赖规划的系统能够声称生成的轨迹的最优性，并避开场景中已知的障碍物。然而，在真实世界环境中，规划方法存在困难，因为它们无法对感知误差、定位误差或不准确的控制做出反应，并且无法在存在不可预测移动的对象和障碍物的环境中执行任务。

在我们之前的工作[1]中，我们介绍了一种实现反应式移动操作的架构，并展示了在移动过程中抓取不可预测的动态对象的能力。然而，之前呈现的实现使用了一个不考虑障碍物避让的移动基座控制器。在这项工作中，我们开发了一个移动基座控制器，用于集成到反应式移动操作的架构中，以在避免静态和动态障碍物的同时最小化任务时间。此外，我们在冗余解析模块中解决的二次规划问题中增加了一个约束条件，用于为机械臂提供障碍物避让功能。

![image-20240220103406397](D:\project\learning\mobile_manipulation\image\image-20240220103406397.png)

我们的反应式方法在具有动态障碍物的复杂环境中表现出了稳健的性能。图1a显示了一个来自真实世界试验的画面，在该画面中，我们的系统正在执行一个移动操作的同时抓取物体，并避开了场景中移动的第二个机器人。图1b中的数字双生体显示了系统对空间的理解，包括环境中障碍物的实时检测。

这项工作的主要贡献是:

1) 一个反应式基座控制系统，通过在移动中执行操作任务，最小化多步任务的总任务时间。
2) 一个冗余解析模块，使机械臂和底座在执行移动操作任务时能够避开障碍物。
3) 在真实世界中展示了首次反应式移动操作任务，其中包含静态和动态障碍物。

这些能力在许多模拟和真实世界的实验中得到了证明。

## II. RELATED WORKS

移动操作中的基座控制方法指导移动基座到一个姿态，从这个姿态可以到达操作目标而不违反运动学约束或与障碍物碰撞。通常通过显式基座控制来实现，其中确定了最佳的基座姿态，并使用传统的移动机器人控制器将机器人导航到该姿态[3]。另一种选择是隐式或整体的基座控制方法，通常从期望的末端执行器运动开始，并利用基座和机械臂的联合运动来实现该运动。

我们回顾了移动操作中常见的基座控制方法以及在执行移动操作时基座控制面临的额外挑战。最近对移动操作的控制策略进行的调查包括[4]。

### A. Explicit Base Control

**Optimal Base Placement**：已经提出了许多规划器来计算最佳执行操作任务的基座姿势。一般来说，目标是找到一个姿势，使得机器人在无碰撞的配置下可以到达目标 [5]。方法通常旨在生成相对于其他指标（如可操作性 [6]、[7]或机器人的刚度 [8]）最优的解决方案。其他方法旨在通过计算基座姿势来最小化任务时间，从而可以在不重新定位的情况下到达多个目标 [9]，[10]。通过根据机器人在立即目标之后必须去的位置选择姿势，可以在多步任务中优化时间效率 [3]，[11]。通过频繁重新计算最佳基座姿势，可以实现反应性 [3]。

**Mobile Base Control**：一旦计算出最佳基座姿势，机器人就会通过移动基座控制器驱动到该姿势。通常使用分层规划器，它结合了全局规划和局部规划器，以在驱动到目标姿势时实现反应性避障 [12]。

[13]中介绍了一种反应性、短期中止A$*$（STAA$*$）方法，与常用的局部规划器相比，在具有静态和动态障碍物的环境中表现出改进的性能。STAA*通过搜索可见性图来规划无碰撞的全局路径，然后计算全局路径与局部规划区域边界的交点，以制定中间目标。离散加速度空间用于对新状态进行采样，以在有时间限制的A*搜索中进行探索。搜索使用考虑障碍物的启发式方法，确保沿着无碰撞路径进行探索并避免局部最小值。

在大多数情况下，使用显式基座控制器的移动操纵器将基座和机械臂运动完全分开，直到基座达到所需姿势时，机械臂才开始移动。最近的方法通过协调机械臂运动和基座运动，使手同时抵达目标，同时基座抵达所需姿势，从而提高了任务完成时间 [3]。

### B. Holistic Control

与单独考虑基座和机械臂运动不同，一些方法采用整体控制器将子系统结合起来 [14]。这些方法利用移动底座和机械手的综合自由度来实现所需的末端执行器运动。机器人的整体控制使得通过同时移动两个组件来减少任务时间成为可能，并且通过利用额外的自由度来优化次要目标，如可操作性 [14]、障碍物避免 [15] 或可见性 [16]，进一步改善了优化能力。

整体控制系统的输入可以来自运动规划器，并在开环控制下执行 [15]，或者是反应性的，其中控制器使用视觉反馈进行闭环控制 [14]，[17]。模型预测控制问题的公式已被用于实现机械臂和底座的碰撞回避 [18]，[19]。[20]中提出了一个学习的基座控制器，它将期望的末端执行器速度和局部占用地图转换为基座运动，以确保全向移动机器人避开障碍物。

这些工作展示了移动操纵器的整体控制，并且能够避开障碍物。然而，它们仅关注执行即时目标，并未考虑在移动到下一个目标的过程中执行操纵任务的多步任务的时间效率。

### C. Manipulation On-The-Move

最早的移动操纵方法限制了基座运动为恒定速度、直线运动 [21]。最近的研究在拥挤的环境中规划无碰撞轨迹，能够以最短时间完成移动操作任务 [2]，[22]–[25]。然而，这些方法以开环方式执行计划的轨迹，无法对场景中的动态变化作出反应，也无法补偿感知和控制误差。因此，在现实世界环境中，这些系统通常容易出现故障。

在本工作中，我们提出了一种反应式的基座控制方法，可以在执行移动操作任务的同时避开障碍物。此外，我们通过实施[26]中描述的方法，实现了机械臂的碰撞回避。

## III. BASE CONTROLLER

我们对STAA* [13]中描述的全局和局部规划器进行了几项修改，以提高在移动操作场景中的性能。

### A. Goal Orientation(目标方向)

STAA$*$最重要的改进之一是将目标状态的方向性纳入考虑范围。在STAA*中，仅考虑到达一个点，而我们则包括了方向性，这使得可以实现平稳连接当前目标和下一个目标的姿势。

![image-20240220110241506](D:\project\learning\mobile_manipulation\image\image-20240220110241506.png)

### B. Rotation in Global Planner Cost（全局规划成本中的旋转）

将方向性加入目标状态需要修改全局A$*$搜索中使用的节点成本计算。STAA*仅使用路径上节点之间的累积距离来计算成本。相反，我们考虑PathRTR度量，该度量估计了沿路径在节点之间进行平移和旋转所需的时间。在[13]中详细说明了PathRTR度量，它被用于局部规划器。图2说明了在全局路径规划器中包含旋转成本的价值。对于所示的场景，我们修改后生成的红色路径鼓励机器人在障碍物周围驾驶平滑曲线连接起始和结束姿势。如果没有旋转成本，最短路径将通过障碍物的另一侧，并且需要更多的转弯。

### C. Search Termination Conditions（搜索终止条件）

[13]中介绍的STAA*的实现在探索的节点足够接近目标时终止其搜索。为了在移动中进行操作，我们希望鼓励机器人以高速通过目标点。因此，当路径足够接近目标时，我们也会终止搜索。

### D. Reduction of Proximity Grid Penalty（减少接近性网格惩罚）

STAA*在访问节点时包含了一个成本，基于它们与障碍物的接近程度，使用了膨胀的占用网格。然而，为了完成移动操作任务，如从桌子上拾取和放置物体，机器人必须在与桌子靠近的地方进行操作。例如，在图2中，占用网格由机器人周围地面的颜色表示，绿色表示自由空间，红色表示占用空间。

当基座的目标姿势靠近障碍物时，例如在从桌子上抓取物体时，应用于靠近障碍物的节点的惩罚会抑制探索接近目标的状态。为了限制这种影响，我们根据与物体拾取或放置位置的接近程度减少占用网格成本的权重。我们通过$k = max(0.1, min(th/3, 1))$来缩放网格惩罚，其中th是到达目标所需的预估时间。

![image-20240220110959130](D:\project\learning\mobile_manipulation\image\image-20240220110959130.png)

### E. Bézier Heuristic

在局部规划器中用于A*图搜索的PathRTR启发式假设机器人的平移和旋转将以最大速度但顺序执行。这倾向于高估那些最佳路径到目标是同时旋转和平移的光滑弧线状态的成本。我们引入了一种基于Bézier曲线的额外启发式，仅当Bézier路径的成本较低时，才使用它来代替PathRTR。这鼓励探索可以通过平滑曲线连接到目标的状态。Bézier曲线通过在当前姿势和目标给定的开始和结束点旁边添加两个控制点来构造。第一个控制点位于机器人当前前进方向的前方，第二个控制点位于期望结束姿势的后方相等的距离处。偏移距离被选择为当前姿势和目标姿势之间距离的25％。最佳偏移距离是相对目标姿势和机器人最大线性和角速度能力之间比值的复杂函数。然而，我们发现25％的值在实践中为我们的机器人产生了良好的曲线效果。图3说明了一个示例Bézier路径。

## IV. BASE PLACEMENT

最佳基座放置是从一个离散化的集合中选择的，通过评估从当前机器人姿势到候选基座放置的路径成本以及从候选位置到下一个目标的路径成本来进行（见图4）。候选位置在目标物体周围均匀分布，每隔10°一个候选位置，总共36个可能的基座位置。每个位置都分配了两种可能的方向：机器人的前向量可以切线于圆圈，面向顺时针或逆时针，这总共产生了72个候选位置。

候选位置所在圆的半径动态调整在0.6米至0.8米之间。当靠近目标物体时没有可用的无碰撞候选位置时，半径会增加直到找到解决方案。半径限制由我们机器人的几何形状定义，0.6米是根据机器人基座的半径加上一个安全边距计算得出的，而0.8米是机器人仍然可以执行操纵任务的最大距离。当在这些限制内找不到可行的候选位置时，机器人将朝着距离目标最近的无碰撞位置驱动，假设系统可能会随着地图使用更新的激光雷达数据识别出一个有效的候选位置。

![image-20240220111832566](D:\project\learning\mobile_manipulation\image\image-20240220111832566.png)

对每个候选位置的路径成本使用了[13]中描述的PathRTR度量进行评估。第i个候选位置的总路径成本由两个组成部分的加权和给出：
$$
C_{i}=C_{i, \mathrm{C}}+1.05 \cdot C_{i, \mathrm{~N}}
$$
其中,$C_{i,C}$是从机器人到候选位置的估计成本，$C_{i, \mathrm{~N}}$是从候选位置到下一个目标的估计成本。偏向于最小化$C_{i, \mathrm{~N}}$的加权确保机器人即使靠近物体也继续朝着下一个目标驱动。将这个权重降低到1以下会倾向于一种贪婪的解决方案，它只优化了当前任务而不考虑下一个任务。进一步增加这个权重将鼓励机器人在当前任务上牺牲时间效率，以最小化下一个任务的预期行程时间。

使用具有最低$C_i$的候选位置作为导航的目标。该方法选择在目标的操纵范围内且有效地连接当前机器人姿势与下一个目标的基座放置位置。例如，图4中的球根据它们的路径成本着色，较亮的绿色表示较低的成本。最佳候选位置的路径也被显示出来。

每个候选位置的路径成本在每个控制器步骤（20 Hz）中重新评估，以实现对环境变化的反应性控制和响应。

## V. ARM OBSTACLE AVOIDANCE

为了使机械手避开障碍物，我们修改了[1]中描述的冗余解析控制器。该控制器解决了一个二次规划问题（QP），以计算给定期望末端执行器和基座速度的关节速度。使用类似的控制器，在[26]中展示了移动机械手的反应式障碍物避让。该控制器允许末端执行器速度的实际值有所变化，这可以与冗余自由度一起利用，以避开障碍物。

在二次规划中通过添加一个不等式约束来实现障碍物避让，该约束限制了机械臂上的点在靠近障碍物时的速度。有关实现的更多细节可参见[26]。

在三维中准确地建模环境，以实现鲁棒的碰撞避免，是一个困难的问题，超出了本工作的范围。在[26]中，通过使用可以直接观察到姿态的模拟对象进行了真实世界的试验。在我们的系统中，我们使用机器人底座上的2D激光雷达来构建机械臂的障碍物地图。激光雷达检测到的任何障碍物都被假设为足够高，以至于机械臂应该避开它们。当障碍物很矮时，这是一个保守的假设，机械臂将不必要地避开障碍物上方的空间。然而，当障碍物在激光雷达平面上方更大时（例如，由中央支柱支撑的桌子），系统无法观察到机械臂高度处的几何形状，存在碰撞的风险。我们通过向系统提供环境中大部分碰撞几何形状的预生成地图来缓解这一问题。激光雷达检测到的额外障碍物被添加到地图中。在未来的工作中，可以使用3D激光雷达或深度摄像机在线建模更详细的碰撞几何形状，从而实现更好的障碍物避免。

我们查询构建的机械手障碍物地图，使用一个位于机器人夹爪中心的单个点，而不是像[26]中那样计算最近的链接碰撞网格上的点。这简化了实现过程，并且我们发现在我们的真实世界测试中它可以达到可接受的性能。末端执行器朝物体的速度被限制在
$$
\dot{d_{r o}} \leq \xi \frac{d-d_{s}}{d_{i}-d_{s}}
$$
其中，$\dot{d_{r o}} $ 是末端执行器与最近障碍物之间的距离，$\xi =0.6$ 是控制障碍物避免的激进性的增益，$d_s=0.25$ 米是末端执行器和障碍物之间允许的最小距离，$d_i=0.6 $米是启用限制的阈值。对于 $d_{ro}>d_i$，约束将从QP中移除。

## VI. EXPERIMENTS

我们在我们的 Frankie3 移动操作器上进行了真实世界实验，该操作器由 Omron LD-60 差分驱动移动基座和 Franka-Emika Panda 7 自由度机械臂组成。机器人控制使用 Robotics Toolbox [27] 实现。模拟和数字双生体环境是在 Unity 中实现的。

### A. Baseline Comparisons

文献[3]中提出的方法通过选择基座姿态来优化任务时间，这些姿态与更高层次任务中的下一个目标位置相关联。这与第IV节中描述的方法类似，但没有考虑在移动中执行任务时所面临的挑战。为了与文献[3]中提出的工作进行有意义的比较，我们尽可能在模拟和真实世界中重新创建他们的实验。[3]提供了两个基线和两种版本的性能数据。以下简要描述了这些方法：

1) **Fixed Set Baseline:**机器人从围绕物体桌周围并朝向物体桌的一组固定的7个候选位置中选择一个基座放置位置。候选位置的选择基于最大化可操作性。
2) **Inverse Reachability Maps (IRM) Baseline:**最佳基座放置位置是基于对可操作性度量的评估而选择的，而不考虑导航成本。
3) **Greedy[3]:**基座放置位置考虑了与候选位置相关的可操作性和导航成本，但不包括导航到下一个位置。
4) **Sequential[3]:**基于加权成本的组合，包括可操作性、到候选位置的导航成本以及从候选位置到下一个位置的导航成本，确定了最佳的基座放置位置。

关于这些方法的更多细节见[3]。与这些方法有关的所有数据均来自[3]。

### B. Experiment 1: Static Obstacles

![image-20240220115730857](D:\project\learning\mobile_manipulation\image\image-20240220115730857.png)

将一系列6个物体放置在一个2.4×0.8米的桌子上，并且必须将它们运输到两个放置位置。图5显示了实验布局 - 项目网站上提供了一个带尺寸的图示。桌子和放置点的位置与[3]中使用的位置一致，可以进行有意义的比较。6个物体被随机放置在图5中显示的12个可能的物体位置中，并且分配了随机的顺序。物体必须按顺序拾取并运输到交替的放置位置。这是为了确保与[3]中的实验进行公平比较，然而需要注意的是，通过优选选择顺序和放置位置可以实现性能改进。