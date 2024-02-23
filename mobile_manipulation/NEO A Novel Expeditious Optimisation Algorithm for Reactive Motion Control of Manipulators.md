# NEO: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators

"NEO：一种新型的高效优化算法，用于机械臂的反应式运动控制"

摘要：我们提出了NEO，一种快速且纯粹的反应式运动控制器，适用于机械臂，能够在移动到期望的末端执行器姿态时避开静态和动态障碍物。此外，我们的控制器在轨迹过程中最大化了机器人的操纵性，同时避免了关节位置和速度限制。NEO被包装成严格凸二次规划，考虑到障碍物、关节限制和机械臂的7自由度时，通常在几毫秒内得到解决。虽然NEO并非旨在取代最先进的运动规划器，但我们的实验表明，它是具有适度复杂性场景的可行替代方案，同时也具有反应式控制能力。对于更复杂的场景，NEO更适合作为反应式局部控制器，与全局运动规划器结合使用。我们在仿真环境中将NEO与运动规划器进行了比较，并在动态环境中通过物理机器人进行了演示和验证。我们提供了一个实现我们控制器的开源库。

## I. INTRODUCTION

**现实世界并非静态，因此，机械臂必须应对工作环境的不可预测性挑战。将机械臂的末端执行器从点A移动到点B是机器人控制中的一个基本问题。虽然这项任务看似简单，但在环境变得混乱和动态时，可能会出现许多失败的原因。在这种环境中，动态障碍物避免对机械臂的鲁棒性至关重要。**

控制循环的每次迭代都必须能够考虑环境和机器人的状态，以确保机器人的安全可靠运行。目前在机械臂障碍物避让方面的关注集中在更大的前期计算负载上，导致了开环运动规划。规划器计算一系列关节坐标，而在运行时则转换为关节速度。相比之下，一种能够避免非静态障碍物的反应式方法直接使用关节速度。

我们考虑了**微分运动学**，并计算了一组关节速度，**将末端执行器引导到任务空间中的目标**。这种经典且纯粹的反应式方法被称为**解决速率运动控制（RRMC），计算成本低廉，并且能够轻松以1000 Hz以上的速度运行**。在这封信中，我们为RRMC方法增加了额外的功能，同时保持了其高度反应性的特性。

**微分运动学还使我们能够捕捉机器人的任何部分与环境中存在的任何障碍物之间距离的变化率**。通过利用这种关系，可以开发出一个反应式控制器，以避免与障碍物碰撞。由此产生的控制器可能会过度约束，并且无法达到目标姿态，但我们可以采用两种策略来解决这个问题。

**首先，我们可以使用运动学上冗余的机械臂，其自由度比必要的在任务空间内达到任何姿态所需的自由度更多——这种情况现在越来越常见。其次，我们可以放宽路径规范，允许有意的误差，称为松弛，这使得末端执行器可以偏离直线轨迹以避开障碍物。**

为了让机器人在不稳定的环境中有最佳的反应能力，我们必须考虑机器人的调节性。一项在[1]中提出的操纵性度量描述了机械臂实现任意速度的调节性。因此，通过最大化机器人的操纵性，**我们可以减少由奇异性导致的机器人故障的可能性**，同时提高机器人的障碍物避让能力。

这篇文章的贡献是：

**1）一个用于串联关节机械臂（全自由度或冗余）的反应式运动控制器，能够实现所需的末端执行器姿态，具有避开静止和非静止障碍物、避免关节限制和最大化操纵性的能力。**

**2）在已发布的运动规划基准测试中进行仿真实验验证，以及在实际的FrankaEmika Panda机器人上进行验证。**

**3）提供一个开源的Python库，提供所有必要的工具来在任何串联关节机械臂上实现我们的控制器。基准测试代码和实现细节可在 jhavl.github.io/neo 找到。**

## II. RELATED WORK

经典的规划方法将根据提供的标准找到最优路径。然而，它们的计算时间意味着它们只能用作开环规划器。像STOMP [3]、CHOMP [4]和Trajopt [5]这样的运动规划器使用基于采样的方法来解决一般的高维运动规划问题，而不仅仅是针对机械臂。运动规划器通常可以在整个运动过程中考虑碰撞避免以及其他约束条件，**如末端执行器的方向。在某些情况下，运动规划器可能需要进行后处理，以消除由于采样而产生的突兀或多余的运动。**

STOMP使**用随机且无梯度的方案来优化轨迹**，同时最小化成本函数。该**方案的随机性意味着它可以克服局部最小值**，而纯粹基于梯度的方法（如CHOMP和Trajopt）可能会受到困扰。CHOMP和Trajopt**都利用了解决优化问题的新方法，同时考虑了障碍物。**

这些算法的**规划时间限制了它们在动态环境中的使**用。在一个基准测试中 [5]，规划时间从0.19秒（对于Trajopt）到4.91秒（对于CHOMP）不等。

路径规划器输出一系列姿态，然后通过求解逆运动学（IK）问题来得到一系列关节坐标。在大多数机器人上，逆运动学可以通过解析方法来求解，但更常见的是通过优化框架来求解 [6]。优化问题可以通过成本函数增加等式或不等式约束，以提供额外的功能。**因此，IK求解器可以提供无碰撞和避免关节限制的关节配置**。通常只有具有可利用的零空间的冗余机器人才能获得有用的优势。

然而，仅凭IK无法提供到达目标的有效路径。我们可以考虑速度而不是在每个时间步考虑姿态或关节配置。这种使用微分运动学的方法通常被称为**解决速率运动控制**（RRMC）[7]。

RRMC利用正向运动学的导数来选择关节速度，**这些速度将使机器人从起始末端执行器姿态直线运动到期望的末端执行器姿态**。与之前提到的方法不同，RRMC是纯粹的反应式方法，并且提供下一个时间点的关节速度，而不是整个轨迹的速度。

如果机器人是冗余的，**那么将可以实现期望的末端执行器速度的无限组关节速度。通常会应用额外的约束，例如最小化关节速度的范数。另外，通过梯度投影和利用零空间，机器人可以在完成子任务的同时实现其运动。**

其中一个子任务是**最大化机器人的操纵性**[8]。通过将操纵性雅可比矩阵投影到微分运动学方程的零空间中，机械臂将朝着目标姿态直线移动末端执行器，**同时选择关节速度，使得机器人的操纵性在每一步都得到最大化。这对于机器人操作的稳健性是有益的，因为它降低了机器人接近奇异配置的可能性。**虽然有许多用于运动学灵敏度的度量方法，但吉川操纵性指标[1]是最广泛接受和使用的运动学操纵性度量。**机器人雅可比矩阵包含与平移和旋转速度相关的行**。此外，由混合棱柱和旋转执行器组成的机器人具有非均匀的雅可比矩阵，这是由于使用了不同的单位。因此，当使用基于雅可比矩阵的操纵性度量（例如[1]）时，必须小心考虑长度缩放和混合单位等问题[9]。在动态环境中，操纵性优化使机器人能够在下一个时间步达到任意速度，以满足不可预测的场景动态需求。

RRMC可以重新表述为二次规划（QP）优化问题。QP提供了考虑约束和成本的能力，从而提供了额外的功能。[10]中的工作将移动机械臂的物理关节限制合并到QP中。[11]中的最新工作将松弛引入了RRMC的典型直线方法中，有效地为问题引入了额外的冗余，这对于具有六个或更少自由度的机器人尤其有用。然后，该工作利用额外的冗余来通过反应式QP控制器最大化机器人的操纵性并避免机械臂的物理关节限制。[12]中描述的速度阻尼器方法用于避免关节限制，但也可以用于避开障碍物。

反应式运动控制的一个经典和替代方法是通过势场，其中机器人被从关节限制和障碍物中排斥[13]，[14]。这将工具引导到一个无碰撞的路径到目标，但并不明确地最大化机器人的操纵性或调节性。

对于重要的基于传感器的运动机器人问题，例如视觉伺服[15]或闭环视觉抓取[16]，反应式控制是必不可少的。这两种方法确定下一个控制循环迭代的末端执行器速度，而使用基于规划的方法实现这一点是具有挑战性的。

**本文提出了一种新颖的实时运动控制器，它可以在纯粹的反应式方式下避开移动障碍物（不仅仅是末端执行器），避开物理关节限制，并最大化机械臂的操纵性。**

**在第三节中，我们将微分运动学和操纵性最大化纳入到一个通用的QP问题中。我们在第四节中扩展了这一方法以处理障碍物，然后在第五节中介绍了速度阻尼器和松弛变量。在第六节中，我们提出了NEO控制器，而在第七节中介绍了我们的实验方法。最后，在第八节中，我们总结了我们的实验结果和见解。**

## III. QUADRATIC PROGRAMMING

QP的一般形式是[17]：
$$
\begin{array}{cl}
\min _{x} & f_{o}(\boldsymbol{x})=\frac{1}{2} \boldsymbol{x}^{\top} \boldsymbol{Q} \boldsymbol{x}+\boldsymbol{c}^{\top} \boldsymbol{x} \\
\text { subject to } & \boldsymbol{A}_{1} \boldsymbol{x}=\boldsymbol{b}_{1} \\
& \boldsymbol{A}_{2} \boldsymbol{x} \leq \boldsymbol{b}_{2} \\
& \boldsymbol{d} \leq \boldsymbol{x} \leq \boldsymbol{e}
\end{array} \tag{1}
$$
其中$f_o$是最小化的目标函数;$A_1,b_1$定义等式约束;$A_2,b_2$定义不等式约束，$d$和$e$定义优化变量$x$的上下界；当矩阵Q是正定时，QP是严格凸的[17]。

### A. Incorporating the Differential Kinematics Into a QP

串联连杆机械手的一阶微分运动学描述为：
$$
\boldsymbol{\nu}(t)=\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}(t) \tag{2}
$$
其中$\boldsymbol{\nu}(t)=\left(\begin{array}{llllll}
v_{x} & v_{y} & v_{z} & \omega_{x} & \omega_{y} & \omega_{z}
\end{array}\right)^{\top} \in \mathbb{R}^{6}$为末端执行器的空间速度，$\boldsymbol{J}(\boldsymbol{q}) \in \mathbb{R}^{6 \times n}$为雅可比矩阵；

我们可以将（2）合并到二次规划中，
$$
\begin{aligned}
\min _{\dot{\boldsymbol{q}}} & f_{o}(\dot{\boldsymbol{q}})=\frac{1}{2} \dot{\boldsymbol{q}}^{\top} \boldsymbol{I} \dot{\boldsymbol{q}}, \\
\text { subject to } & \boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}=\boldsymbol{\nu}, \\
& \dot{\boldsymbol{q}}^{-} \leq \dot{\boldsymbol{q}} \leq \dot{\boldsymbol{q}}^{+}
\end{aligned} \tag{3}
$$
其中$I$是一个n × n单位矩阵，$\dot{\boldsymbol{q}}^{-}$和$\dot{\boldsymbol{q}}^{+}$表示关节速度的上限和下限，不需要定义不等式约束。如果机器人具有比到达其整个任务空间所需的更多的自由度，则(3)中的QP将以最小关节速度范数实现期望的末端执行器速度。然而，我们还可以优化其他方面，例如可操作性。

### B. Incorporating Manipulability Into a QP

操纵性度量，由[1]提出，描述了机械臂的调节性，即机器人可以多轻易地实现任意速度。我们观察到，最大化机器人的空间平移速度性能比旋转性能更为重要。为了避免混合单位和相关的非均匀性问题，我们采用了类似于[18]的策略，只使用与平移速度$\boldsymbol{J}_t(\boldsymbol{q}) $对应的机器人雅可比矩阵的三行。平移操纵性度量是一个标量。
$$
m_{t}=\sqrt{\operatorname{det}\left(\boldsymbol{J}_{t}(\boldsymbol{q}) \boldsymbol{J}_{t}(\boldsymbol{q})^{\top}\right)} \tag{4}
$$
我们可以将其视为接近运动学奇异点的距离度量。接近奇异点的一个症状是，由于雅可比矩阵的调节性较差，所需的关节速度达到了不可能的水平[15]。在动态环境中进行反应式控制时，下一步所需的运动事先是未知的，因此实现任意任务空间速度的能力至关重要，这将在第VIII-A节中进行演示。

我们可以使用方程（4）的时间导数，并将其纳入到方程（3）的QP中，以在实现期望的末端执行器速度的同时最大化操纵性[11]。
$$
\begin{array}{l}
\min _{\dot{\boldsymbol{q}}} f_{o}(\dot{\boldsymbol{q}})=\underbrace{\frac{1}{2} \dot{\boldsymbol{q}}^{\top}\left(\lambda_{q} \boldsymbol{I}\right)}_{\mathrm{A}} \dot{\boldsymbol{q}} \underbrace{-\boldsymbol{J}_{m}^{\top} \dot{\boldsymbol{q}}}_{\mathrm{B}},\\
\begin{aligned}
\text { subject to } & \underbrace{\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}}_{\mathrm{C}}=\boldsymbol{\nu}, \\
& \underbrace{\dot{\boldsymbol{q}}^{-} \leq \dot{\boldsymbol{q}} \leq \dot{\boldsymbol{q}}^{+}}_{\mathrm{D}}
\end{aligned}
\end{array} \tag{5}
$$
A     -      最小化速度的范数

B    -     最大限度地提高机器人的可操作性

C  -   确保所需的末端执行器速度

D    -    确保机器人的速度限制得到遵守。

其中λq是速度范数最小化的增益，Jm ∈ Rn是平移可操作性雅可比矩阵，它将可操作性的变化率与关节速度联系起来:
$$
\boldsymbol{J}_{m}^{\top}=\left(\begin{array}{c}
m_{t} \operatorname{vec}\left(\boldsymbol{J}_{t} \boldsymbol{H}_{\boldsymbol{t} \mathbf{1}}^{\top}\right)^{\top} \operatorname{vec}\left(\left(\boldsymbol{J}_{t} \boldsymbol{J}_{t}^{\top}\right)^{-1}\right) \\
m_{t} \operatorname{vec}\left(\boldsymbol{J}_{t} \boldsymbol{H}_{\boldsymbol{t} \mathbf{2}}^{\top}\right)^{\top} \operatorname{vec}\left(\left(\boldsymbol{J}_{t} \boldsymbol{J}_{t}^{\top}\right)^{-1}\right) \\
\vdots \\
m_{t} \operatorname{vec}\left(\boldsymbol{J}_{t} \boldsymbol{H}_{\boldsymbol{t} \boldsymbol{n}}^{\top}\right)^{\top} \operatorname{vec}\left(\left(\boldsymbol{J}_{t} \boldsymbol{J}_{t}^{\top}\right)^{-1}\right)
\end{array}\right) \tag{6}
$$
其中$\boldsymbol{J}_{m}^{\top} \in \mathbb{R}^{n}$，其中向量运算vec（·）：$\mathbb{R}^{a \times b}$→ $\mathbb{R}^{ab}$将矩阵按列转换为向量，$\boldsymbol{H}_{ti} \in \mathbb{R}^{3 \times n}$是操纵器海森张量$\boldsymbol{H}_{} \in \mathbb{R}^{6 \times n \times n}$的第i个平移分量[19]

## IV. MODELLING OBSTACLES

三维空间中的一点可以表示为$\boldsymbol{p} \in \mathbb{R}^{3}$。机器人上的一点$p_r$与障碍物上的一点$p_o$之间的距离$d$为：
$$
d_{r o}=\left\|\boldsymbol{p}_{o}-\boldsymbol{p}_{r}\right\| \tag{7}
$$
并且从$p_r$指向$p_o$的单位向量$\hat{n}_{ro}$是：
$$
\hat{\boldsymbol{n}}_{r o}=\frac{\boldsymbol{p}_{r}-\boldsymbol{p}_{o}}{d_{r o}}=-\hat{\boldsymbol{n}}_{o r} \in \mathbb{R}^{3} \tag{8}
$$
（8）的时间导数为:
$$
\begin{aligned}
\dot{d}_{r o} & =\frac{\mathrm{d}}{\mathrm{d} t}\left\|\boldsymbol{p}_{o}(t)-\boldsymbol{p}_{r}(t)\right\| \\
& =\hat{\boldsymbol{n}}_{o r}^{\top}\left(\dot{\boldsymbol{p}}_{o}(t)-\dot{\boldsymbol{p}}_{r}(t)\right)
\end{aligned} \tag{9}
$$
我们从微分运动学中知道，末端执行器的速度为$\boldsymbol{\nu}(t)=\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}(t)$。此外，我们可以通过考虑该点作为机器人末端执行器的修正机械臂雅可比矩阵的平移速度分量来计算固定在机器人上的任意点的速度。
$$
\begin{aligned}
\boldsymbol{J}_{p}\left(\boldsymbol{q}_{0 \ldots k}\right) & =\Lambda\left(\frac{\partial}{\partial \boldsymbol{q}_{0} \ldots k}\left({ }^{0} \boldsymbol{T}_{\boldsymbol{q}_{k}} \bullet{ }^{\boldsymbol{q}_{k}} \boldsymbol{T}_{p}\right)\right) \\
\boldsymbol{J}_{p}(\tilde{\boldsymbol{q}}) & =\Lambda\left(\frac{\partial}{\partial \tilde{\boldsymbol{q}}}\left({ }^{0} \boldsymbol{T}_{\boldsymbol{q}_{k}} \bullet{ }^{\boldsymbol{q}_{k}} \boldsymbol{T}_{P}\right)\right)
\end{aligned} \tag{10}
$$
其中，k 是附着$p_r$的连杆的索引，$\boldsymbol{J}_{p} \in \mathbb{R}^{3 \times k } $是将点 $p$的速度与从关节 0 到 k 的速度相关联的雅可比矩阵，$_{}^{0}T_{q_k}$ 描述了关节 k 相对于机器人基座坐标系的姿态，• 表示组合运算，Λ(·) 是将姿态的偏导数从张量 R6×k×k 转换为平移速度雅可比矩阵 ∈ R3×k 的函数。请注意，该雅可比矩阵仅取决于连接到点的连杆之前的 k 个关节。我们将 Jp 依赖的关节集合表示为$\tilde{\boldsymbol{q}}$，其大小可变 - 如果该点连接到末端执行器，则 k = n。

姿态 $_{}^{q_k}T_P$ 描述了固定在连杆 k 上的参考坐标系 P 相对于连杆 k 的参考坐标系的位置，并且其原点为 $p_r$。该框架的方向是任意的，尽管方向 $\hat{n}$是在此框架中表示的，因此我们忽略了（10）中的偏导数中的角速度分量。有关计算此雅可比矩阵的更多详细信息，请参阅 [19]。我们提供的软件包 [2] 使得计算这些雅可比矩阵变得非常简单。

使用（10），我们现在可以写:
$$
\dot{\boldsymbol{p}}_{r}(t)=\boldsymbol{J}_{p_{r}}(\tilde{\boldsymbol{q}}) \dot{\tilde{\boldsymbol{q}}}(t) \tag{11}
$$
距离的变化率:
$$
\begin{aligned}
\dot{d}_{r o}(t) & =\hat{\boldsymbol{n}}_{o r}^{\top}\left(\dot{\boldsymbol{p}}_{o}(t)-\boldsymbol{J}_{p_{r}}(\tilde{\boldsymbol{q}}) \dot{\tilde{\boldsymbol{q}}}(t)\right) \\
& =\hat{\boldsymbol{n}}_{o r}^{\top} \dot{\boldsymbol{p}}_{o}(t)-\hat{\boldsymbol{n}}_{o r}^{\top} \boldsymbol{J}_{p_{r}}(\tilde{\boldsymbol{q}}) \dot{\tilde{\boldsymbol{q}}}(t) .
\end{aligned} \tag{12}
$$
距离系数$\dot{\tilde{\boldsymbol{q}}}(t) $是距离雅可比矩阵，
$$
\boldsymbol{J}_{d}(\tilde{\boldsymbol{q}})=\hat{\boldsymbol{n}}_{r o}^{\top} \boldsymbol{J}_{p_{r}}(\tilde{\boldsymbol{q}}) \in \mathbb{R}^{6} \tag{13}
$$
将（13）代入（12），我们得到标量:
$$
\boldsymbol{J}_{d}(\tilde{\boldsymbol{q}}) \dot{\tilde{\boldsymbol{q}}}(t)=\dot{d_{r o}}(t)-\hat{\boldsymbol{n}}_{o r}^{\top} \dot{\boldsymbol{p}}_{o}(t) \tag{14}
$$
它现在是一种可用于我们的QP的形式。

## V. OBSTACLE AVOIDANCE

### A. Velocity Dampers

[12]中概述的速度阻尼器方法可以通过在达到限制之前阻尼或限制速度来防止机器人故障。这可以用于在达到限制之前限制关节运动，或者在遇到障碍物之前约束机器人速度。一般的速度阻尼器公式是:
$$
v \leq \xi \frac{d-d_{s}}{d_{i}-d_{s}} \tag{15}
$$
其中，v 是距离 d 的变化速率，ξ 是调整阻尼器侵略性的正增益，di 是阻尼器处于活动状态的影响距离，ds 是距离 d 永远不会小于的停止距离。这些距离在图2中有所说明。在优化器中使用时，必须找到小于速度阻尼器设置的限制的 v。

![image-20240223170302532](D:\project\learning\mobile_manipulation\image\image-20240223170302532.png)

我们可以将避障功能与速度阻尼器结合起来:
$$
v \leq \xi \frac{d-d_{s}}{d_{i}-d_{s}}\dot{d_{r o}}(t) \leq \xi \frac{d-d_{s}}{d_{i}-d_{s}}\boldsymbol{J}_{d}(\tilde{\boldsymbol{q}}) \dot{\tilde{\boldsymbol{q}}}(t)=\dot{d_{r o}}(t)-\hat{\boldsymbol{n}}_{o r}^{\top} \dot{\boldsymbol{p}}_{o}(t)\boldsymbol{J}_{d}(\tilde{\boldsymbol{q}})=\hat{\boldsymbol{n}}_{r o}^{\top} \boldsymbol{J}_{p_{r}}(\tilde{\boldsymbol{q}}) \in \mathbb{R}^{6} \tag{16}
$$
然而，为了将其用作QP的不等式约束，我们必须合并（14）的形式：
$$
\boldsymbol{J}_{d}(\tilde{\boldsymbol{q}}) \dot{\tilde{\boldsymbol{q}}}(t) \leq \xi \frac{d-d_{s}}{d_{i}-d_{s}}-\hat{\boldsymbol{n}}_{o r}^{\top} \dot{\boldsymbol{p}}_{o}(t) \tag{17}
$$
式（17）的作用是优化器必须选择引起机器人与障碍物之间最小距离变化速率增加的关节速度。因此，在阻尼器处于活动状态时，机器人上的点和障碍物上的点永远不会发生碰撞。通过以式（17）的形式添加多个不等式约束，机器人可以同时应对无限多个动态障碍物。多个障碍物可以通过垂直堆叠多个式（17）的实例并列入同一不等式约束中。
$$
\begin{array}{c}
\boldsymbol{A} \dot{\boldsymbol{q}}(t) \leq \boldsymbol{b} \\
\left(\begin{array}{cc}
\boldsymbol{J}_{d_{0}}\left(\tilde{\boldsymbol{q}}_{0}\right) & \mathbf{0}_{1 \times n-k_{0}} \\
\vdots & \vdots \\
\boldsymbol{J}_{d_{l}}\left(\tilde{\boldsymbol{q}}_{l}\right) & \mathbf{0}_{1 \times n-k_{l}}
\end{array}\right) \dot{\boldsymbol{q}}(t) \leq\left(\begin{array}{c}
\xi_{0} \frac{d_{0}-d_{s}}{d_{i}-d_{s}}-\hat{\boldsymbol{n}}_{o r_{0}}^{\top} \dot{\boldsymbol{p}}_{o_{0}}(t) \\
\vdots \\
\xi_{l} \frac{d_{l}-d_{s}}{d_{i}-d_{s}}-\hat{\boldsymbol{n}}_{o r_{l}}^{\top} \dot{\boldsymbol{p}}_{o_{l}}(t)
\end{array}\right)
\end{array} \tag{18}
$$
对于 l 个障碍物，其中矩阵$A \in \mathbb{R}^{l \times n}$中的雅可比矩阵已经被堆叠并填充了零，使它们的长度保持为常数 n。显然，考虑到多个障碍物和有限的机器人自由度，将存在某些情况下没有可行解的情况。

如果式（5）中的等式约束处于活动状态，那么机器人将无法偏离其路径以躲避障碍物，优化器将失败 - 它无法同时满足等式约束和不等式约束。为了避免这种情况，我们将在我们的优化器中添加松弛变量。

### B. Adding Slack

我们可以通过添加一个松弛向量 δ ∈ R6 来扩展我们的优化变量 $\dot{q}$，并将问题修改为
$$
\boldsymbol{\nu}(t)-\boldsymbol{\delta}(t)=\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}(t) \tag{19}
$$
其中，δ 表示期望末端执行器速度与实际末端执行器速度之间的差异，从而放宽了轨迹约束。我们重新制定我们的 QP 问题为:
$$
\begin{aligned}
\min _{x} \quad f_{o}(\boldsymbol{x}) & =\frac{1}{2} \boldsymbol{x}^{\top} \mathcal{Q} \boldsymbol{x}+\mathcal{C}^{\top} \boldsymbol{x}, \\
\text { subject to } \quad \mathcal{J} \boldsymbol{x} & =\boldsymbol{\nu}, \\
\mathcal{A} \boldsymbol{x} & \leq \mathcal{B} \\
\mathcal{X}^{-} & \leq \boldsymbol{x} \leq \mathcal{X}^{+}
\end{aligned} \tag{19}
$$
其中：
$$
\boldsymbol{x}=\left(\begin{array}{c}
\dot{\boldsymbol{q}} \\
\boldsymbol{\delta}
\end{array}\right) \in \mathbb{R}^{(n+6)} \tag{20}
$$

$$
\mathcal{Q}=\left(\begin{array}{cc}
\lambda_{q} \boldsymbol{I}_{n \times n} & \mathbf{0}_{6 \times 6} \\
\mathbf{0}_{n \times n} & \lambda_{\delta} \boldsymbol{I}_{6 \times 6}
\end{array}\right) \in \mathbb{R}^{(n+6) \times(n+6)} \tag{21}
$$

$$
\mathcal{J}=\left(\begin{array}{ll}
\boldsymbol{J} & \boldsymbol{I}_{6 \times 6}
\end{array}\right) \in \mathbb{R}^{6 \times(n+6)} \tag{22}
$$

$$
\mathcal{C}=\left(\begin{array}{c}
\boldsymbol{J}_{m} \\
\mathbf{0}_{6 \times 1}
\end{array}\right) \in \mathbb{R}^{(n+6)} \tag{23}
$$

$$
\mathcal{A}=\left(\begin{array}{cc}
\boldsymbol{J}_{d_{0}}\left(\tilde{\boldsymbol{q}}_{0}\right) & \mathbf{0}_{1 \times 6+n-k_{0}} \\
\vdots & \vdots \\
\boldsymbol{J}_{d_{l}}\left(\tilde{\boldsymbol{q}}_{l}\right) & \mathbf{0}_{1 \times 6+n-k_{l}}
\end{array}\right) \in \mathbb{R}^{(l \times n+6)} \tag{24}
$$

$$
\mathcal{B}=\left(\begin{array}{c}
\xi \frac{d_{0}-d_{s}}{d_{i}-d_{s}}-\hat{\boldsymbol{n}}_{\text {or }}^{\top} \dot{\boldsymbol{p}}_{o_{0}}(t) \\
\vdots \\
\xi \frac{d_{l}-d_{s}}{d_{i}-d_{s}}-\hat{\boldsymbol{n}}_{\text {or }}^{\top} \dot{\boldsymbol{p}}_{o_{l}}(t)
\end{array}\right) \in \mathbb{R}^{l} \tag{25}
$$

$$
\mathcal{X}^{-,+}=\left(\begin{array}{l}
\dot{\boldsymbol{q}}^{-,+} \\
\boldsymbol{\delta}^{-,+}
\end{array}\right) \in \mathbb{R}^{n+6} \tag{26}
$$

我们可以通过添加速度阻尼器来扩展矩阵 A 和 B，以确保控制器遵守关节位置限制。我们用以下方式替换 A 和 B：
$$
\mathcal{A}^{*}=\left(\begin{array}{c}
\mathcal{A} \\
\boldsymbol{I}_{n \times n+6}
\end{array}\right) \in \mathbb{R}^{(l+n \times n+6)} \tag{28}
$$

$$
\mathcal{B}^{*}=\left(\begin{array}{c}
\mathcal{B} \\
\eta \frac{\rho_{0}-\rho_{s}}{\rho_{i}-\rho_{s}} \\
\vdots \\
\eta \frac{\rho_{n}-\rho_{s}}{\rho_{i}-\rho_{s}}
\end{array}\right) \in \mathbb{R}^{l+n} \tag{29}
$$

其中，ρ 表示到最近关节限制的距离（对于旋转关节可能是一个角度），ρi 表示操作阻尼器的影响距离，ρs 表示关节到其限制的最小距离。值得注意的是，只有当距离 d 和 ρ 小于相应的影响距离 di 和 ρi 时，才会向 A∗ 和 B∗ 添加每一行。当向这些矩阵添加一行时，用于避免碰撞或关节限制的速度阻尼器被激活。

![image-20240223172223035](D:\project\learning\mobile_manipulation\image\image-20240223172223035.png)

## VI. PROPOSED CONTROLLER

我们提出的控制器 NEO 在基于位置的伺服（PBS）方案中利用了式（20）描述的 QP。该控制器旨在将机器人的末端执行器从当前姿态直线驱动到期望姿态。PBS 被表述为:
$$
\boldsymbol{\nu}_{e}=\beta \psi\left(\left({ }^{0} \boldsymbol{T}_{e}\right)^{-1} \bullet{ }^{0} \boldsymbol{T}_{e^{*}}\right) \tag{30}
$$
其中，β ∈ R+ 是一个增益项，0Te ∈ SE(3) 是机器人基座坐标系中当前末端执行器姿态，0Te∗ ∈ SE(3) 是机器人基座坐标系中期望的末端执行器姿态，ψ(·): R^4×4 → R^6 是一个函数，它将齐次变换矩阵转换为空间扭转。

通过将式（20）中的 ν 设置为式（30）中的 νe，机器人将朝着目标驱动，但不一定是直线运动。机器人会偏离路径以提高可操纵性，避免关节速度限制、关节位置限制，并躲避静态和动态障碍物。我们详细列出了在表I中改变控制器参数的影响。

所提出的控制器受局部最小值的影响，如果直接路径被障碍物阻挡，可能会阻止其达到目标姿态。我们编写了一个简单的后退启发式方法，在许多常见情况下已被证明是有效的：如果末端执行器与目标之间的直接路径被障碍物阻挡，并且最近的障碍物在影响距离 di 内，那么我们会偏置期望末端执行器速度 νe 的 vx 和 vy 分量，使机械臂朝向机器人的基座坐标系收缩。

我们使用我们的Python库，Robotics Python库[2]来实现我们的控制器。我们使用Python库qpsolvers来实现二次规划求解器[21]来优化（20）

