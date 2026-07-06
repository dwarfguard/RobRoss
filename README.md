# RobRoss
R.O.B Ross

Abstract / 项目概述

Robot arm that paints in color on a 12-inch canvas, with the intent of attracting audience attention and serving as an interactive art installation. Users may choose from preselected art pieces for the robot to paint, then bring the finished canvas home as a souvenir.

一个机器人绘画装置。机械臂在 12 英寸画布上作画，吸引观众停留；用户可以选择预设作品，并把完成的画布作为纪念品带走。

Target Audience / Environment / 目标受众与使用场景

Commercial malls, cafés, and similar public consumer spaces. The installation can boost customer engagement and venue attractiveness by creating sentimental value, a memorable experience, and potential social media attention.

适用于商场、咖啡店等公共商业空间。核心价值是提升顾客参与感和场地吸引力，同时制造有纪念意义、适合社交媒体传播的话题体验。

Potential Components / 潜在组件

Robot Claw / 机械爪

    Secures, picks up, puts down, and navigates the brush.
    固定、拿起、放下并移动画笔。

Stand for the Canvas / 画布支架

    Provides a stable platform for the canvas while the robot arm paints.
    保持画布稳定，方便机械臂在画布上作画。

Brush / 画笔

    Transfers paint onto the canvas through motion from the robot claw.
    通过机械爪的运动，把颜料转移到画布上。

Paint / 颜料

    The material used by the robot claw to paint.
    机器人作画所使用的颜料材料。

Paint Bucket / 颜料桶

    Container for different types or colors of paint; can also secure brushes when not in use.
    存放不同颜色或种类的颜料，也可在画笔不用时进行固定或收纳。

Robot Control System

    Computes pathing, movement, color selection, and brush selection to direct the robot claw in completing the selected painting.
    Continuously checks the canvas for mistakes, paint saturation, and adjusts accordingly.
    负责计算绘画路径、机械臂动作、颜色选择和画笔选择；同时检测画面错误和颜料饱和度，并进行调整。
    Artwork Preparation System
    Robot Execution System
    User Interface
    Maintenance Interface

Color Mixer / 调色器

    Optional component for mixing paint colors instead of preparing every color in advance.
    可选组件，用于现场调色，替代提前准备所有颜色的方案。
    
## Team Roles and Collaboration / 团队分工与协作

This project currently involves three core technical roles: project coordination, software development, and hardware engineering. Because the product combines robotics, software, physical materials, and public user experience, clear communication between these areas is essential.

本项目目前包含三个核心技术角色：项目协调、软件开发和硬件工程。由于产品同时涉及机器人、软件、实体材料和公共用户体验，各方向之间的清晰沟通非常重要。

### Project Coordinator and Software Contributor / 项目协调与软件参与

Bryan

The project coordinator is responsible for keeping the product direction, prototype scope, documentation, team communication, and development milestones aligned. This role also contributes to software development, especially in areas related to artwork preparation, path generation, prototype testing, and documentation.

项目协调者负责统一产品方向、原型范围、文档、团队沟通和开发节点。同时，该角色也会参与软件开发，尤其是作品预处理、路径生成、原型测试和文档整理等部分。

Key responsibilities:

* Maintain the project overview and decision log.
* Track open questions, risks, and next steps.
* Translate product goals into clear software and hardware requirements.
* Help define the first prototype scope.
* Support software development for artwork processing and robot path preparation.
* Participate in hardware testing and learn enough hardware context to understand practical constraints.

主要职责：

* 维护项目概览和决策记录。
* 跟踪未解决问题、风险和下一步任务。
* 将产品目标转化为明确的软件和硬件需求。
* 协助确定第一版原型范围。
* 支持作品处理和机器人路径准备相关的软件开发。
* 参与硬件测试，并学习必要的硬件知识以理解现实限制。

### Dedicated Software Developer / 专职软件开发

Raymond

The dedicated software developer is responsible for the main software architecture, robot control integration, code quality, and implementation of the software systems required for the prototype.

专职软件开发者负责主要软件架构、机器人控制集成、代码质量，以及原型所需软件系统的实现。

### Dedicated Hardware Engineer / 专职硬件工程师

Francois

The dedicated hardware engineer is responsible for the robot arm setup, end effector design, brush or tool mounting, canvas stand, paint station, safety considerations, and physical reliability of the prototype.

专职硬件工程师负责机械臂设置、末端执行器设计、画笔或工具固定、画布支架、颜料区域、安全问题以及原型的实体可靠性。

### Collaboration Principle / 协作原则

The team should avoid building software, hardware, and product ideas separately. Every major decision should connect back to the first working prototype: a robot that can reliably complete a simple painting on a 12-inch canvas with minimal human intervention.

团队应避免软件、硬件和产品想法各自独立发展。每个重要决策都应回到第一版可运行原型：让机器人能够在 12 英寸画布上稳定完成一幅简单作品，并尽量减少人工干预。
