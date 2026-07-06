**R.O.B Ross**

**Abstract / 项目概述**

Robot arm that paints in color on a 12-inch canvas, with the intent of attracting audience attention and serving as an interactive art installation. Users may choose from preselected art pieces for the robot to paint, then bring the finished canvas home as a souvenir.

一个机器人绘画装置。机械臂在 12 英寸画布上作画，吸引观众停留；用户可以选择预设作品，并把完成的画布作为纪念品带走。

**Target Audience / Environment / 目标受众与使用场景**

Commercial malls, cafés, and similar public consumer spaces. The installation can boost customer engagement and venue attractiveness by creating sentimental value, a memorable experience, and potential social media attention.

适用于商场、咖啡店等公共商业空间。核心价值是提升顾客参与感和场地吸引力，同时制造有纪念意义、适合社交媒体传播的话题体验。

**Potential Components / 潜在组件**

**Robot Claw / 机械爪**

* Secures, picks up, puts down, and navigates the brush.  
* 固定、拿起、放下并移动画笔。

**Stand for the Canvas / 画布支架**

* Provides a stable platform for the canvas while the robot arm paints.  
* 保持画布稳定，方便机械臂在画布上作画。

**Brush / 画笔**

* Transfers paint onto the canvas through motion from the robot claw.  
* 通过机械爪的运动，把颜料转移到画布上。

**Paint / 颜料**

* The material used by the robot claw to paint.  
* 机器人作画所使用的颜料材料。

**Paint Bucket / 颜料桶**

* Container for different types or colors of paint; can also secure brushes when not in use.  
* 存放不同颜色或种类的颜料，也可在画笔不用时进行固定或收纳。

**Robot Computer / 机器人电脑**

* Computes pathing, movement, color selection, and brush selection to direct the robot claw in completing the selected painting.  
* Continuously checks the canvas for mistakes, paint saturation, and adjusts accordingly.  
* 负责计算绘画路径、机械臂动作、颜色选择和画笔选择；同时检测画面错误和颜料饱和度，并进行调整。

**Color Mixer / 调色器**

* Optional component for mixing paint colors instead of preparing every color in advance.  
* 可选组件，用于现场调色，替代提前准备所有颜色的方案。

**Points of Discussion / 讨论重点**

**1\. Do we need a color mixer? / 是否需要调色器？**

Current idea: have different colors readily available, with each color having its own paint bucket and dedicated brushes. An initial setup may include around 20 total colors through 20 paint buckets, each with 2 brushes: one large and one small.

目前设想是提前准备不同颜色，而不是现场调色。初版可准备约 20 种颜色，每种颜色一个颜料桶，并配两支专用画笔：一大一小。

**2\. What type of paint should we use? / 应该使用哪种颜料？**

Options include acrylic paint and oil paint. The decision should consider drying time, fluidity, color consistency, ease of use, cost, and maintenance.

可考虑丙烯颜料或油画颜料。选择时需要比较干燥时间、流动性、颜色稳定性、操作难度、成本和后期维护。

**3\. What art style should we use? / 应该采用什么绘画风格？**

Francois and Bryan proposed a style currently called “Blobs.” The computer analyzes an image and groups similar colors together to create larger areas of one color. White space is left between each blob so that there are gaps between shapes and colors do not mix.

Francois 和 Bryan 提出一种暂称为 “Blobs”（色块）的风格。电脑会分析图片，把相近颜色归成较大的单色区域，并在色块之间保留白色空隙，避免颜色混合。

A potential training method is to connect the system to an illustration app such as Adobe Illustrator. The model could learn through a reward/punishment system: it is rewarded for covering the inside of a target shape and penalized for painting outside the outline. The goal is to train the model to learn optimal pathing for painting one blob with a single stroke using a circular brush. Further testing is required to reduce unwanted white space.

一种训练方式是连接 Adobe Illustrator 等插画软件，让模型通过奖励/惩罚机制学习：覆盖形状内部会被奖励，画出边界会被惩罚。目标是让模型学会用圆形画笔一笔完成一个色块的最佳路径。后续还需要测试如何减少多余白边。

**4\. How long should one painting take? / 一幅画应该花多久完成？**

Since the goal is customer attention and retention, faster is not always better. If the robot finishes too quickly, crowds may not have enough time to gather and watch. If it takes too long, audience members may lose interest. The team should determine the optimal time frame for completing one 12-inch canvas.

由于目标是吸引观众并让顾客停留，速度不一定越快越好。太快会让人来不及聚集观看；太慢又可能让观众失去耐心。因此需要确定完成一幅 12 英寸画布的最佳时间。

**5\. What type of brush should we use? / 应该使用什么类型的画笔？**

Because it may be difficult to fine-tune brush angles during each stroke, the team should compare regular rectangular brushes, circular brushes, and possibly sponge-like tools. A sponge may work similarly to a highlighter, creating broader strokes with simpler motion control.

由于机械臂在每一笔中实时微调角度可能比较困难，需要比较矩形画笔、圆形画笔，甚至海绵类工具。海绵可能像荧光笔一样产生较宽的笔触，并降低控制难度。
