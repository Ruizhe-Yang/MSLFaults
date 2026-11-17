# MSLFaults v1.0.0 — 使用手册

[English](UserGuide.md) · 简体中文

## 目录

- [1. 库的用途与包结构](#1-库的用途与包结构)
- [2. 加载库并运行首个示例](#2-加载库并运行首个示例)
- [3. 建立正常参考配置](#3-建立正常参考配置)
- [4. 配置激活与恢复](#4-配置激活与恢复)
- [5. 建立首个故障电路](#5-建立首个故障电路)
- [6. 使用外部触发](#6-使用外部触发)
- [7. 连接物理元件并解读诊断量](#7-连接物理元件并解读诊断量)
- [8. 选择模式并比较响应](#8-选择模式并比较响应)
- [9. 完整模型与类参考](#9-完整模型与类参考)
- [10. 自带示例](#10-自带示例)
- [11. 排错与建模边界](#11-排错与建模边界)
- [12. 文档与延伸阅读](#12-文档与延伸阅读)

## 1. 库的用途与包结构

MSLFaults 提供 **142 个可配置故障的元件**，分别对应 Modelica Standard Library 4.0.0 中的相关元件，覆盖信号处理以及电气、机械、热学、流体和磁路网络。全库包含 **212 个 Modelica 源文件**，包括包、支撑模型、工具内指南及 **10 个完整示例**。唯一库依赖为 **Modelica Standard Library 4.0.0**。

根据正常工况下的关系和连接器接口选择元件，再配置该类的 `FaultMode`。数值目标和实际方程共同决定故障效果。这些元件采用确定性机理模型，模式名称和强度值本身不代表故障率或发生概率。

| 包 | 故障元件数量 | 内容 |
| --- | --- | --- |
| [`Blocks`](../Blocks/package.mo) | 4 | 增益、一阶动态、延迟与限幅。 |
| [`Electrical`](../Electrical/package.mo) | 53 | 模拟电路元件、源、传感器、理想器件与半导体器件。 |
| [`Mechanics`](../Mechanics/package.mo) | 41 | 转动与平动元件及传感器。 |
| [`Thermal`](../Thermal/package.mo) | 19 | 传热与简化热流体元件及传感器。 |
| [`Fluid`](../Fluid/package.mo) | 15 | 采用 Modelica.Fluid 接口的管道、泵、阀与传感器。 |
| [`Magnetic`](../Magnetic/package.mo) | 10 | 磁通管元件、电磁转换与磁传感器。 |

`Examples` 包含完整的对照模型。`Utilities.Internal` 提供示例装配模型与共享激活模块，`UsersGuide` 提供建模工具内的简要使用指南。`Electrical.Analog.Semiconductors.NPNFaultStage` 是晶体管支撑电路，不计入上述 142 个以 `Faultable…` 命名的类。

![MSLFaults 模型图标展示](assets/model-icons.svg)

预览使用模型现有的本地图标 `Icon` 图形，包括红色故障标记，省略实例名与参数标签。Modelica 工具还会根据元件配置显示继承的图形和条件连接器。

## 2. 加载库并运行首个示例

1. 解压压缩包，保留完整的 `MSLFaults` 目录。
2. 在建模环境中加载 Modelica 4.0.0，再打开 `MSLFaults/package.mo`。
3. 打开 `MSLFaults.Examples.ResistorReplacement`，查看其 Diagram 图。
4. 使用自带实验设置：0–5 s、输出间隔 0.01 s、容差 `1e-6`。
5. 绘制 `resistorNativeCurrent`、`resistorNormalCurrent`、`resistorFaultCurrent` 和 `resistorFault.device.effectLevel`。三路 `conductor…Current` 输出提供对应的电导元件对照。

每条电阻通路均使用 12 V 电源和正常值为 10 Ω 的电阻，故障目标为 20 Ω。电导通路的正常值为 0.1 S，故障目标为 0.05 S。下表由这些参数解析推导，可用于对照实际仿真输出。

| 仿真时刻 | 原生元件电流 | Normal 模式电流 | 故障工况电流 |
| --- | --- | --- | --- |
| 0.5 s | 1.2 A | 1.2 A | 1.2 A |
| 2 s | 1.2 A | 1.2 A | 0.6 A |
| 4 s | 1.2 A | 1.2 A | 1.2 A |

故障请求在 `[1,3)` s 内激活，每次状态切换后都有 0.05 s 的过渡。解读电流曲线时，应同时查看电阻或电导本身的变化：在电压固定时，电流与电阻成反比，与电导成正比。

使用 OpenModelica 时，在包含 `MSLFaults` 文件夹的上级目录中执行：

```modelica
loadModel(Modelica, {"4.0.0"});
loadFile("MSLFaults/package.mo");
checkModel(MSLFaults.Examples.ResistorReplacement);
simulate(MSLFaults.Examples.ResistorReplacement,
  startTime=0, stopTime=5, numberOfIntervals=500,
  tolerance=1e-6);
getErrorString();
```

## 3. 建立正常参考配置

使用 `faultMode=FaultMode.Normal` 作为参考配置，必要时通过所选类的完整名称指定枚举类型。保持与原生元件相同的正常参数、连接器方向、介质、初始条件和可选端口设置。选择故障前，先比较周围系统需要使用的各项物理量或信号。

在图形界面中替换元件时，先记录原元件的类名、实例名与修饰参数，再换成所选 `MSLFaults` 元件，并重新连接相同的物理或信号接口。应用需要使用条件端口时，再将其启用。如果上层模型提供兼容的 `replaceable` 声明和约束类，也可以通过重新声明替换元件；自带示例采用了这种方式。

兼容性取决于具体类及其配置。有些元件封装原生模型并修改指令或边界关系，另一些元件使用自身的本构方程。第 9 节逐项列出了重要限制，包括未传给内部模型的继承参数。

`enabled=false` 和 `severity=0` 可以取消激活请求，但不能普遍替代 `Normal` 模式：部分传感器的饱和方程不受激活包络控制，仍会施加限幅。判断参考工况时，应结合该模式的实际方程与观察到的物理量或信号。

## 4. 配置激活与恢复

多数故障元件提供以下控制参数和输出。修改配置前，应查看所选元件自身的声明。

| 设置或输出 | 含义 |
| --- | --- |
| `faultMode` | 各类独立定义的枚举类型；应使用所选元件声明的模式标识符。 |
| `enabled` | 启用所选故障请求。通常默认为 true，而默认模式为 `Normal`。 |
| `severity` | `[0,1]` 内的最大激活包络值，其物理含义由所选方程决定。 |
| `faultStartTime`、`faultEndTime` | 内部请求的左闭右开时间窗 `[faultStartTime,faultEndTime)`。 |
| `transitionTime` | 完整单位幅值上升或下降所需的时间；取零时，包络立即切换。 |
| `useExternalTrigger`、`trigger` | 启用并选择条件布尔触发端口，由其替代内部时间窗。 |
| `active` | 请求或内部锁存产生的布尔激活状态。 |
| `effectLevel` | 模式所使用的强度缩放包络，并非实际物理故障程度的通用度量。 |
| `activationCount` | 激活开始的次数，包括初始化时已激活的请求。 |

共享请求逻辑要求 `enabled=true`、模式不是 `Normal` 且强度为正。`transitionTime` 为正时，包络从零开始变化；过渡途中反向切换时，从当前值连续变化。恢复过程中，`active` 可能已为 false，而 `effectLevel` 仍为正。采用永久锁存的模式会保持激活状态，具体模型会说明是否使用这种机制。

许多连续目标采用 `nominal + effectLevel*(target-nominal)`。部分漂移模式还使用 `driftTime`，另一些名称含 `Drift` 的模式仅使用激活包络。因此，应配置实际目标值，不能仅凭模式名称推断数值增加、损失或老化过程。

部分理想开关、理想二极管和布尔流体阀仅在 `effectLevel>0.5` 时强制切换状态。这些模式的最大强度如果不超过 0.5，就不会越过覆盖阈值，其实际切换时刻也可能晚于请求开始时刻。

保存采样输出、输出规定固定值、停止动态状态演化和永久锁存是不同的行为。例如，`Blocks.Continuous.FaultableFirstOrder` 的 `Stuck` 模式在完整作用时停止状态演化；其 `Freeze` 模式在完整作用时保持已保存的输出，部分作用时则进行插值，而内部状态始终继续演化。选择恢复行为时，应以具体元件的定义为准。

## 5. 建立首个故障电路

将以下模型保存为 `FirstResistorFault.mo`。`within;` 表示顶层模型；如果将其放入命名工作包，应相应修改该声明。

```modelica
within;
model FirstResistorFault "Observe a reversible resistance change"
  Modelica.Electrical.Analog.Sources.ConstantVoltage supply(V=12);
  Modelica.Electrical.Analog.Basic.Ground ground;
  MSLFaults.Electrical.Analog.Basic.FaultableResistor resistor(
    R=10,
    faultMode=MSLFaults.Electrical.Analog.Basic.FaultableResistor.FaultMode.ResistanceStep,
    R_step=20,
    enabled=true,
    severity=1,
    faultStartTime=1,
    faultEndTime=3,
    transitionTime=0.05);
equation
  connect(supply.p, resistor.p);
  connect(resistor.n, ground.p);
  connect(supply.n, ground.p);
  annotation(experiment(StartTime=0, StopTime=5,
    Interval=0.01, Tolerance=1e-6));
end FirstResistorFault;
```

运行模型并绘制 `resistor.i`、`resistor.R_effective`、`resistor.active`、`resistor.effectLevel` 和 `resistor.activationCount`。激活前的预期电流为 1.2 A，上升过渡完成后为 0.6 A，恢复后重新变为 1.2 A。建立正常对照时，仅将 `faultMode` 改为同一类的 `FaultMode.Normal` 后重新运行。

随后设置 `severity=0.5`。包络达到平台值后，电阻为 `10 + 0.5*(20-10) = 15 Ω`，对应电流为 0.8 A。这是电阻插值的结果，强度并不直接按比例缩放电流。只有在所连系统适合参数突变时，才将 `transitionTime` 设为零。

设置 `useHeatPort=true` 时，应将 `heatPort` 连接到合适的热网络，并查看继承的 `LossPower` 输出。默认禁用热端口时，元件在内部使用所指定的温度条件。

## 6. 使用外部触发

下面的信号示例在布尔条件成立时增加偏置。启用 `useExternalTrigger` 后会显示条件连接器 `trigger`，内部故障时间窗不再用于选择请求。

```modelica
within;
model FirstTriggeredGain "Apply a signal bias from a Boolean condition"
  Modelica.Blocks.Sources.RealExpression ramp(y=time);
  Modelica.Blocks.Sources.BooleanExpression condition(
    y=time >= 1 and time < 2);
  MSLFaults.Blocks.Math.FaultableGain gain(
    k=2,
    faultMode=MSLFaults.Blocks.Math.FaultableGain.FaultMode.Bias,
    biasFault=1,
    enabled=true,
    severity=0.5,
    transitionTime=0.1,
    useExternalTrigger=true);
equation
  connect(ramp.y, gain.u);
  connect(condition.y, gain.trigger);
  annotation(experiment(StartTime=0, StopTime=3,
    Interval=0.01, Tolerance=1e-6));
end FirstTriggeredGain;
```

绘制 `ramp.y`、`condition.y`、`gain.y`、`gain.active` 和 `gain.effectLevel`。正常增益输出为 `2*time`。平台阶段的完整偏置为 `severity*biasFault=0.5`，因此 1.5 s 时的预期输出为 3.5。恢复渐变结束后，输出回到 `2*time`。实际应用中，可以保持连接器不变，将布尔表达式换成所需触发条件。

在修饰参数中使用带类名限定的枚举值。`FaultableGain.FaultMode.Bias` 与另一个类中同名的模式属于不同枚举类型，所填模式值必须属于正在配置的元件。

## 7. 连接物理元件并解读诊断量

物理连接器承载网络平衡关系。引入故障元件时，应保持正常工况的连接方向、参考节点和边界条件。

| 领域 | 连接与观察要点 |
| --- | --- |
| 电气 | 保持引脚方向、电气参考节点及条件热端口配置。查看电压、电流和元件的损耗输出。有限开路、短路目标保留原连接拓扑。 |
| 机械 | 使用相应的转动或平动法兰、支承及初始条件。查看力或力矩、运动量与有效参数；锁止力或力矩可能采用有限阻力，而非刚性约束。 |
| 传热 | 连接有物理意义的温度和热流边界，同时观察有效热容或热导及温度响应。 |
| `Fluid` | 采用一致且兼容的 `Modelica.Media` 介质、适当的 `inner Modelica.Fluid.System`，并满足所声明的压力、流量和流股条件。 |
| `Thermal.FluidHeatFlow` | 使用该体系自身的简化介质和连接器，其端口不同于 `Modelica.Fluid` 端口。 |
| 磁路 | 提供磁路参考通路，查看磁通与磁势差。对于电磁转换，应同时观察电气量和磁路量。 |

储能参数发生变化时，仅检查最终参数值还不够。可用的诊断量取决于具体模型。

| 元件类别 | 解读方式 |
| --- | --- |
| 故障电容与电感 | 查看模型提供的电荷或磁链、储能、损耗及参数变化功率项。参数变化期间的瞬态由实际状态关系决定。 |
| `FaultableMass`、`FaultableInertia` | 通过有效质量或转动惯量演化线动量或角动量。`parameterPower` 表示储能变化率、端口功率与模型损耗之间的差额。 |
| 转动与平动 `FaultableSpring` | 储能使用有效刚度计算。刚度过渡会改变储能，应同时查看 `parameterPower`、力或力矩及变形。 |
| `FaultableHeatCapacitor` | 采用固定 0 K 基准，使用 `storedEnergy=C_effective*T` 和 `der(storedEnergy)=port.Q_flow`。`parameterPower` 输出恒为零。 |
| `FaultableMassWithStopAndFriction` | 采用有效质量乘以加速度的关系。到达硬限位时重置位置和速度；该碰撞重置不计入连续的 `lossPower`。 |

部分模式名称对应简化的等效作用。热容模型的 `ThermalRunaway` 通过降低热容起作用，不增加发热项；`SensorLag` 改变的是物理热容。简化热流体管道的 `Leakage` 通过降低压降起作用，不抽取质量；其换热损失模式仅缩放介质侧热输入，外部热端口的热流保持不变。使用前应判断这些关系是否适合目标问题。

## 8. 选择模式并比较响应

先选择一种机理，记录其目标值、起始条件、强度及恢复设置。各次运行保持相同的激励、正常参数、初始条件和求解设置。除 `active` 和 `effectLevel` 外，还应比较系统响应。

选型时，以下区别尤其需要关注：

| 模型或类别 | 实际实现的区别 |
| --- | --- |
| 机械与磁路传感器 | 只要 `faultMode=Saturation`，就会应用饱和限制，不受激活包络控制。噪声模式使用确定性正弦信号。 |
| 电流与电压传感器 | `Stuck` 默认保存测量值，选择固定值选项后才使用规定值。漂移模式在恢复过程中保持已累积的偏移量，包络归零后才移除该偏移。 |
| 电气多量传感器 | 电流、电压和功率测量分别修改，故障后的功率读数不一定等于电流读数与电压读数的乘积。 |
| 电气波形源 | 幅值故障会缩放包括偏移量在内的整个波形；正弦和余弦源的频率故障直接修改相位表达式。 |
| `Rotational.Components.FaultableIdealGear` | `LockedGear` 附加力矩正比于一个已被理想齿轮方程约束为零的相对速度，因此在当前关系中不会产生附加锁止力矩。 |
| 理想电气变压器 | `PrimaryOpen` 和 `MagnetizationLoss` 只有在 `considerMagnetization=true` 时才会影响励磁支路。 |
| 无输出限制的理想运算放大器 | 输入泄漏与短路关系作用于已被约束为零的差分输入电压，因此相应模式不会产生额外输入电流。 |
| `Fluid.Machines.FaultablePump` | 部分效率或摩擦模式通过规定附加轴力矩起作用，不改变水力效率曲线；轴抱死模式使用有限的转速比例力矩。 |

下方完整参考说明进一步列出各类的相关设置与边界。即使输出曲线相似，测量扰动和物理故障也可能产生不同的系统响应。需要区分测量误差与被控对象变化时，应另行观察独立的物理量。

## 9. 完整模型与类参考

下表按包列出全部 212 个源码文件，点击名称可打开相应 Modelica 定义。故障元件同时列出正常 MSL 对应类、配置方法与重要使用边界；可选模式和参数以该元件的 FaultMode 枚举及参数对话框为准。嵌套辅助类在所属源码文件中说明。

### 9.1. MSLFaults

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`MSLFaults`](../package.mo) | 面向 Modelica 标准库接口的元件故障模型。 | 加载 Modelica 4.0.0 和 package.mo，从 Examples.ResistorReplacement 入门。 |

### 9.2. Blocks

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Blocks`](../Blocks/package.mo) | 具有故障模式的信号处理模块。 | 包的用途与内容。 |

### 9.3. Blocks.Continuous

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Continuous`](../Blocks/Continuous/package.mo) | 连续信号动态模块。 | 包的用途与内容。 |
| [`FaultableFirstOrder`](../Blocks/Continuous/FaultableFirstOrder.mo) | 具有参数与输出故障的一阶响应。 正常参考：`Modelica.Blocks.Continuous.FirstOrder`。 | 连接 u、y，设置 k、T 与 initType，再选择模式和目标值。Stuck 在完整作用时停止状态演化；Freeze 将输出向捕获的状态值插值。 |

### 9.4. Blocks.Math

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Math`](../Blocks/Math/package.mo) | 代数信号变换模块。 | 包的用途与内容。 |
| [`FaultableGain`](../Blocks/Math/FaultableGain.mo) | 具有增益变化、偏置、保持及输出限制的增益模块。 正常参考：`Modelica.Blocks.Math.Gain`。 | 设置 k 和所选模式的目标值，连接 u 并读取 y。Freeze 保持激活时捕获的输出；Stuck 使用规定值。 |

### 9.5. Blocks.Nonlinear

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Nonlinear`](../Blocks/Nonlinear/package.mo) | 信号延迟与限幅模块。 | 包的用途与内容。 |
| [`FaultableFixedDelay`](../Blocks/Nonlinear/FaultableFixedDelay.mo) | 具有可变延迟和输出故障的因果延迟模块。 正常参考：`Modelica.Blocks.Nonlinear.FixedDelay`。 | 设置非负 delayTime 与 delayTimeFault。三种延迟变化模式使用同一计算关系，由目标值决定延迟增减。 |
| [`FaultableLimiter`](../Blocks/Nonlinear/FaultableLimiter.mo) | 具有边界变化与固定输出故障的限幅器。 正常参考：`Modelica.Blocks.Nonlinear.Limiter`。 | 设置顺序正确的正常与故障限值，再连接 u、y。Stuck 在限幅后向 stuckValue 插值，输出可能超出限值。 |

### 9.6. Electrical

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Electrical`](../Electrical/package.mo) | 具有可配置故障效果的电气元件。 | 按元件类别浏览本包。 |

### 9.7. Electrical.Analog

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Analog`](../Electrical/Analog/package.mo) | 模拟电气元件及测量模型。 | 按元件类别浏览本包。 |

### 9.8. Electrical.Analog.Basic

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Basic`](../Electrical/Analog/Basic/package.mo) | 无源及受控电气元件。 | 按元件类别浏览本包。 |
| [`FaultableCCC`](../Electrical/Analog/Basic/FaultableCCC.mo) | 具有增益变化及输出损失的电流控制电流源。 正常参考：`Modelica.Electrical.Analog.Basic.CCC`。 | 连接两组端口，设置 gain 和 gainFault，查看实际传递参数与端口输出。 漂移与 StuckGain 模式使用相同的可逆插值；StuckGain 不采集或锁存此前增益。 |
| [`FaultableCCV`](../Electrical/Analog/Basic/FaultableCCV.mo) | 具有跨阻变化及输出损失的电流控制电压源。 正常参考：`Modelica.Electrical.Analog.Basic.CCV`。 | 连接两组端口，设置 transResistance 和 transResistanceFault，查看实际传递参数与端口输出。 漂移与 StuckGain 模式使用相同的可逆插值；StuckGain 不采集或锁存此前增益。 |
| [`FaultableCapacitor`](../Electrical/Analog/Basic/FaultableCapacitor.mo) | 具有容量变化、漏电和串联损耗的电容。 正常参考：`Modelica.Electrical.Analog.Basic.Capacitor`。 | 设置 C 与所选故障目标；容量变化时查看 v_cap、q、storedEnergy、lossPower 与 parameterPower。 parameterPower 表示规定容量变化引起的功率，与耗散损失不同；R_seriesNominal 是故障串联电阻目标的下限，并非正常串联电阻。 |
| [`FaultableConductor`](../Electrical/Analog/Basic/FaultableConductor.mo) | 具有电导及温度系数故障的电导元件。 正常参考：`Modelica.Electrical.Analog.Basic.Conductor`。 | 设置 G、G_fault 或有限开短路目标，连接电气端口及可选热端口；查看 G_actual 与 LossPower。 ConductanceDrift 与 ConductanceStep 使用相同的激活包络插值，均没有独立漂移计时。 |
| [`FaultableGyrator`](../Electrical/Analog/Basic/FaultableGyrator.mo) | 具有独立回转电导故障的回转器。 正常参考：`Modelica.Electrical.Analog.Basic.Gyrator`。 | 连接两组电气端口，按所需耦合变化设置 G1_fault 和 G2_fault。 两个回转电导可独立配置，功率平衡取决于所选参数。 |
| [`FaultableInductor`](../Electrical/Analog/Basic/FaultableInductor.mo) | 具有电感、绕组损耗及匝间短路故障的电感。 正常参考：`Modelica.Electrical.Analog.Basic.Inductor`。 | 设置 L 与所选目标；匝间短路应配置 maxTurnShortRatio 和 R_turnShortLoss，查看 psi、storedEnergy、lossPower 与 parameterPower。 TurnShort 使用等效绕组关系；parameterPower 记录规定电感变化所对应的功率。 |
| [`FaultablePotentiometer`](../Electrical/Analog/Basic/FaultablePotentiometer.mo) | 具有电阻轨及滑片故障的电位器。 正常参考：`Modelica.Electrical.Analog.Basic.Potentiometer`。 | 连接 pin_p、pin_n 和 contact；需要输入控制滑片时启用 useRinput，查看 r_effective、Rp、Rn 与 LossPower。 ContactOpen 在两个电阻轨支路中分别加入有限电阻；WiperStuck 使用预设位置，不采集激活瞬间的位置。 |
| [`FaultableResistor`](../Electrical/Analog/Basic/FaultableResistor.mo) | 具有阻值及温度系数故障的电阻。 正常参考：`Modelica.Electrical.Analog.Basic.Resistor`。 | 设置 R 及对应阻值或 alpha 目标，连接 p、n，并按需连接 heatPort；查看 R_effective、R_actual 与 LossPower。 |
| [`FaultableTransformer`](../Electrical/Analog/Basic/FaultableTransformer.mo) | 具有绕组故障的耦合电感变压器。 正常参考：`Modelica.Electrical.Analog.Basic.Transformer`。 | 协调设置 L1、L2 与 M，再配置对应电感目标或 R_open；查看绕组电流和实际参数。 方程将变化电感乘在电流导数上，并不对随时间变化的磁链矩阵整体求导。 |
| [`FaultableVCC`](../Electrical/Analog/Basic/FaultableVCC.mo) | 具有跨导变化及输出损失的电压控制电流源。 正常参考：`Modelica.Electrical.Analog.Basic.VCC`。 | 连接两组端口，设置 transConductance 和 transConductanceFault，查看实际传递参数与端口输出。 漂移与 StuckGain 模式使用相同的可逆插值；StuckGain 不采集或锁存此前增益。 |
| [`FaultableVCV`](../Electrical/Analog/Basic/FaultableVCV.mo) | 具有增益变化及输出损失的电压控制电压源。 正常参考：`Modelica.Electrical.Analog.Basic.VCV`。 | 连接两组端口，设置 gain 和 gainFault，查看实际传递参数与端口输出。 漂移与 StuckGain 模式使用相同的可逆插值；StuckGain 不采集或锁存此前增益。 |
| [`FaultableVariableCapacitor`](../Electrical/Analog/Basic/FaultableVariableCapacitor.mo) | 输入控制电容的比例、偏置、卡住及容量损失故障。 正常参考：`Modelica.Electrical.Analog.Basic.VariableCapacitor`。 | 连接输入 C，设置 scaleFault、biasFault、stuckValue 或 lossValue，并结合 Cmin 查看 C_effective。 原始输入 C 必须非负；StuckInput 使用预设 stuckValue，不采集激活瞬间的输入。 |
| [`FaultableVariableConductor`](../Electrical/Analog/Basic/FaultableVariableConductor.mo) | 输入控制电导的比例、偏置及开短路故障。 正常参考：`Modelica.Electrical.Analog.Basic.VariableConductor`。 | 连接输入 G 及电气端口，设置 scaleFault、biasFault 或对应有限目标，查看 G_effective 和 G_actual。 目标参数应保持预期无源特性及正温度修正因子；故障后的指令值不会自动钳位为非负值。 |
| [`FaultableVariableInductor`](../Electrical/Analog/Basic/FaultableVariableInductor.mo) | 输入控制电感的比例、偏置、卡住及电感损失故障。 正常参考：`Modelica.Electrical.Analog.Basic.VariableInductor`。 | 连接输入 L，设置 scaleFault、biasFault、stuckValue 或 lossValue，并结合 Lmin 查看 L_effective。 原始输入 L 必须非负；StuckInput 使用预设 stuckValue，不采集激活瞬间的输入。 |
| [`FaultableVariableResistor`](../Electrical/Analog/Basic/FaultableVariableResistor.mo) | 输入控制电阻的比例、偏置及开短路故障。 正常参考：`Modelica.Electrical.Analog.Basic.VariableResistor`。 | 连接输入 R 及电气端口，设置 scaleFault、biasFault 或对应有限目标，查看 R_effective 和 R_actual。 目标参数应保持预期无源特性及正温度修正因子；故障后的指令值不会自动钳位为非负值。 |

### 9.9. Electrical.Analog.Ideal

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Ideal`](../Electrical/Analog/Ideal/package.mo) | 理想开关及放大元件。 | 按元件类别浏览本包。 |
| [`FaultableControlledIdealClosingSwitch`](../Electrical/Analog/Ideal/FaultableControlledIdealClosingSwitch.mo) | 电压控制闭合开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.ControlledIdealClosingSwitch`。 | 连接 p、n 和 control；参数故障设置 RonFault 或 GoffFault，查看 off 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |
| [`FaultableControlledIdealIntermediateSwitch`](../Electrical/Analog/Ideal/FaultableControlledIdealIntermediateSwitch.mo) | 电压控制交叉换向开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.ControlledIdealIntermediateSwitch`。 | 连接全部切换端口及控制输入，查看 control_effective、Ron_effective、Goff_effective 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |
| [`FaultableControlledIdealOpeningSwitch`](../Electrical/Analog/Ideal/FaultableControlledIdealOpeningSwitch.mo) | 电压控制分断开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.ControlledIdealOpeningSwitch`。 | 连接 p、n 和 control；参数故障设置 RonFault 或 GoffFault，查看 off 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |
| [`FaultableControlledIdealTwoWaySwitch`](../Electrical/Analog/Ideal/FaultableControlledIdealTwoWaySwitch.mo) | 电压控制双路转换开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.ControlledIdealTwoWaySwitch`。 | 连接全部切换端口及控制输入，查看 control_effective、Ron_effective、Goff_effective 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |
| [`FaultableIdealClosingSwitch`](../Electrical/Analog/Ideal/FaultableIdealClosingSwitch.mo) | 布尔控制闭合开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealClosingSwitch`。 | 连接 p、n 和 control；参数故障设置 RonFault 或 GoffFault，查看 off 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |
| [`FaultableIdealDiode`](../Electrical/Analog/Ideal/FaultableIdealDiode.mo) | 具有导通故障的分段线性二极管。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealDiode`。 | 连接 p 与 n，配置有限故障目标，查看 off、实际参数和 LossPower。 强制通断要求 effectLevel>0.5；ShortCircuit 保留配置的拐点电压，因此端口关系通常不等同于纯电阻。 |
| [`FaultableIdealIntermediateSwitch`](../Electrical/Analog/Ideal/FaultableIdealIntermediateSwitch.mo) | 布尔控制交叉换向开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealIntermediateSwitch`。 | 连接全部切换端口及控制输入，查看 control_effective、Ron_effective、Goff_effective 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |
| [`FaultableIdealOpAmp`](../Electrical/Analog/Ideal/FaultableIdealOpAmp.mo) | 四端口理想运算放大器及输入故障。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealOpAmp`。 | 连接合适的反馈网络，设置 inputOffsetFault，查看差分输入与电路响应。 InputLeakage 与 InputShort 模式同时约束差分输入电压为零，因此当前方程中的附加输入电流也为零。 |
| [`FaultableIdealOpAmp3Pin`](../Electrical/Analog/Ideal/FaultableIdealOpAmp3Pin.mo) | 三引脚理想运算放大器及输入故障。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealOpAmp3Pin`。 | 连接合适的反馈网络，设置 inputOffsetFault，查看差分输入与电路响应。 InputLeakage 与 InputShort 模式同时约束差分输入电压为零，因此当前方程中的附加输入电流也为零。 |
| [`FaultableIdealOpAmpLimited`](../Electrical/Analog/Ideal/FaultableIdealOpAmpLimited.mo) | 具有失调、增益及输出故障的限幅运算放大器。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealOpAmpLimited`。 | 连接差分输入、输出、两个限幅端口及反馈回路，查看 vin_effective、out_internal 与 out.v。 gainFault 参与归一化特性的斜率定义，实际小信号输出/输入增益还取决于电源轨跨度；OutputStuck 在限幅之后施加，设定值可能位于电源轨之外。 |
| [`FaultableIdealOpeningSwitch`](../Electrical/Analog/Ideal/FaultableIdealOpeningSwitch.mo) | 布尔控制分断开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealOpeningSwitch`。 | 连接 p、n 和 control；参数故障设置 RonFault 或 GoffFault，查看 off 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |
| [`FaultableIdealTransformer`](../Electrical/Analog/Ideal/FaultableIdealTransformer.mo) | 具有变比及励磁支路故障的理想变压器。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealTransformer`。 | 设置 n 与 considerMagnetization，连接两侧绕组，并按所选模式配置 nFault、Lm1Fault 或 R_open。 MagnetizationLoss 和 PrimaryOpen 仅在 considerMagnetization=true 时影响方程，实际变比必须保持非零。 |
| [`FaultableIdealTwoWaySwitch`](../Electrical/Analog/Ideal/FaultableIdealTwoWaySwitch.mo) | 布尔控制双路转换开关及接触故障。 正常参考：`Modelica.Electrical.Analog.Ideal.IdealTwoWaySwitch`。 | 连接全部切换端口及控制输入，查看 control_effective、Ron_effective、Goff_effective 与 LossPower。 强制开关状态要求 effectLevel>0.5；severity<=0.5 时不会进入强制状态，接触电阻与泄漏变化仍采用连续包络。 |

### 9.10. Electrical.Analog.Semiconductors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Semiconductors`](../Electrical/Analog/Semiconductors/package.mo) | 具有可配置退化的半导体器件。 | 按元件类别浏览本包。 |
| [`FaultableDiode`](../Electrical/Analog/Semiconductors/FaultableDiode.mo) | 具有结特性及漏电故障的 PN 二极管。 正常参考：`Modelica.Electrical.Analog.Semiconductors.Diode`。 | 连接 p 与 n，设置所选电流、电压尺度或电阻目标；启用温度依赖时连接 heatPort，并查看 LossPower。 ForwardVoltageDrift 改变结电压尺度，不直接规定恒定正向压降；ShortCircuit 保留半导体支路。 |
| [`FaultableNMOS`](../Electrical/Analog/Semiconductors/FaultableNMOS.mo) | N 沟道 MOS 管及沟道、漏电故障。 正常参考：`Modelica.Electrical.Analog.Semiconductors.NMOS`。 | 连接 D、G、S 与 B，保证实际沟道尺寸有效，并设置对应故障目标；stuckOnOverdrive 的正负应符合沟道类型。 StuckOn 使用过驱动电压模型，不强制为恒定电阻；ShortCircuit 保留沟道关系，栅极和体端电流仍为零。 |
| [`FaultableNPN`](../Electrical/Analog/Semiconductors/FaultableNPN.mo) | 具有增益、结特性及集电极通路故障的 NPN 管。 正常参考：`Modelica.Electrical.Analog.Semiconductors.NPN`。 | 连接 B、C、E，并在启用时连接 S 和 heatPort；配置相应增益、结参数或 RCEShort，查看端口电流与 LossPower。 CollectorEmitterOpen 衰减集电极输运项，但保留基极结与衬底关系，并不切断所有端口电流。 |
| [`FaultablePMOS`](../Electrical/Analog/Semiconductors/FaultablePMOS.mo) | P 沟道 MOS 管及沟道、漏电故障。 正常参考：`Modelica.Electrical.Analog.Semiconductors.PMOS`。 | 连接 D、G、S 与 B，保证实际沟道尺寸有效，并设置对应故障目标；stuckOnOverdrive 的正负应符合沟道类型。 StuckOn 使用过驱动电压模型，不强制为恒定电阻；ShortCircuit 保留沟道关系，栅极和体端电流仍为零。 |
| [`FaultableZDiode`](../Electrical/Analog/Semiconductors/FaultableZDiode.mo) | 具有结特性、击穿及漏电故障的稳压二极管。 正常参考：`Modelica.Electrical.Analog.Semiconductors.ZDiode`。 | 设置 Bv 等正常结参数，再配置 BvFault、IdsFault、VtFault 或对应电阻目标。 电气关系不含温度依赖，可选热端口仅传递损耗；短路模式仍保留正向和击穿电流项。 |
| [`NPNFaultStage`](../Electrical/Analog/Semiconductors/NPNFaultStage.mo) | 带引线电阻及并联电容的 NPN 级。 | 作为电路级连接 b、c、e，设置 scenarioSeverity 和 faultMode，并查看 Tr 的端口电流。 内部晶体管保留默认 faultEndTime=3 s；该级包含附加电路，不能直接视为裸 NPN 管。 |

### 9.11. Electrical.Analog.Sensors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sensors`](../Electrical/Analog/Sensors/package.mo) | 具有输出故障的电气测量模型。 | 按元件类别浏览本包。 |
| [`FaultableCurrentSensor`](../Electrical/Analog/Sensors/FaultableCurrentSensor.mo) | 具有测量故障的理想串联电流传感器。 正常参考：`Modelica.Electrical.Analog.Sensors.CurrentSensor`。 | 将 p 与 n 串联接入，按 A 设置故障幅值，比较 i 与 i_actual。 Drift 使用严重度乘以本次激活持续时间；恢复期间偏移量保持取消激活时的值，直到 effectLevel 降为零后消失。 |
| [`FaultableMultiSensor`](../Electrical/Analog/Sensors/FaultableMultiSensor.mo) | 共用故障模式的电流、电压和功率传感器。 正常参考：`Modelica.Electrical.Analog.Sensors.MultiSensor`。 | 连接两条测量通路，配置 biasCurrent、biasVoltage、biasPower 或对应卡住值，并同时查看三个输出。 故障后的功率输出不一定等于故障后的电流与电压乘积；noiseAmplitude 和 saturationLimit 在各通道单位下使用同一数值。 |
| [`FaultablePotentialSensor`](../Electrical/Analog/Sensors/FaultablePotentialSensor.mo) | 具有输出故障的电位传感器。 正常参考：`Modelica.Electrical.Analog.Sensors.PotentialSensor`。 | 将 p 接在被测节点，比较 phi 与 phi_actual。 Stuck 使用以 V 为单位的预设 stuckValue，不采集激活时的测量值。 |
| [`FaultablePowerSensor`](../Electrical/Analog/Sensors/FaultablePowerSensor.mo) | 具有输出故障的功率传感器。 正常参考：`Modelica.Electrical.Analog.Sensors.PowerSensor`。 | 将 pc/nc 串入电流通路，pv/nv 跨接被测电压，查看 power 与 power_actual。 Stuck 使用以 W 为单位的预设 stuckValue，不采集激活时的测量值。 |
| [`FaultableVoltageSensor`](../Electrical/Analog/Sensors/FaultableVoltageSensor.mo) | 具有测量故障的理想电压传感器。 正常参考：`Modelica.Electrical.Analog.Sensors.VoltageSensor`。 | 将 p 与 n 接在被测节点之间，按 V 设置故障幅值，比较 v 与 v_actual。 Drift 使用严重度乘以本次激活持续时间；恢复期间偏移量保持取消激活时的值，直到 effectLevel 降为零后消失。 |

### 9.12. Electrical.Analog.Sources

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sources`](../Electrical/Analog/Sources/package.mo) | 具有输出故障的规定电气源。 | 按元件类别浏览本包。 |
| [`FaultableConstantCurrent`](../Electrical/Analog/Sources/FaultableConstantCurrent.mo) | 具有输出故障的恒定电流源。 正常参考：`Modelica.Electrical.Analog.Sources.ConstantCurrent`。 | 设置恒定值 I。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableConstantVoltage`](../Electrical/Analog/Sources/FaultableConstantVoltage.mo) | 具有输出故障的恒定电压源。 正常参考：`Modelica.Electrical.Analog.Sources.ConstantVoltage`。 | 设置恒定值 V。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableCosineCurrent`](../Electrical/Analog/Sources/FaultableCosineCurrent.mo) | 具有输出故障的余弦电流源。 正常参考：`Modelica.Electrical.Analog.Sources.CosineCurrent`。 | 设置 I、f、phase、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 FrequencyDrift 直接缩放按时间计算的相位表达式，不对瞬时频率积分。 |
| [`FaultableCosineVoltage`](../Electrical/Analog/Sources/FaultableCosineVoltage.mo) | 具有输出故障的余弦电压源。 正常参考：`Modelica.Electrical.Analog.Sources.CosineVoltage`。 | 设置 V、f、phase、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 FrequencyDrift 直接缩放按时间计算的相位表达式，不对瞬时频率积分。 |
| [`FaultablePulseCurrent`](../Electrical/Analog/Sources/FaultablePulseCurrent.mo) | 具有输出故障的脉冲电流源。 正常参考：`Modelica.Electrical.Analog.Sources.PulseCurrent`。 | 设置 I、width、period、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultablePulseVoltage`](../Electrical/Analog/Sources/FaultablePulseVoltage.mo) | 具有输出故障的脉冲电压源。 正常参考：`Modelica.Electrical.Analog.Sources.PulseVoltage`。 | 设置 V、width、period、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableRampCurrent`](../Electrical/Analog/Sources/FaultableRampCurrent.mo) | 具有输出故障的斜坡电流源。 正常参考：`Modelica.Electrical.Analog.Sources.RampCurrent`。 | 设置 I、duration、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableRampVoltage`](../Electrical/Analog/Sources/FaultableRampVoltage.mo) | 具有输出故障的斜坡电压源。 正常参考：`Modelica.Electrical.Analog.Sources.RampVoltage`。 | 设置 V、duration、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableSignalCurrent`](../Electrical/Analog/Sources/FaultableSignalCurrent.mo) | 具有输出故障的信号控制电流源。 正常参考：`Modelica.Electrical.Analog.Sources.SignalCurrent`。 | 将规定电流连接至输入 i。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableSignalVoltage`](../Electrical/Analog/Sources/FaultableSignalVoltage.mo) | 具有输出故障的信号控制电压源。 正常参考：`Modelica.Electrical.Analog.Sources.SignalVoltage`。 | 将规定电压连接至输入 v。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableSineCurrent`](../Electrical/Analog/Sources/FaultableSineCurrent.mo) | 具有输出故障的正弦电流源。 正常参考：`Modelica.Electrical.Analog.Sources.SineCurrent`。 | 设置 I、f、phase、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 FrequencyDrift 直接缩放按时间计算的相位表达式，不对瞬时频率积分。 |
| [`FaultableSineVoltage`](../Electrical/Analog/Sources/FaultableSineVoltage.mo) | 具有输出故障的正弦电压源。 正常参考：`Modelica.Electrical.Analog.Sources.SineVoltage`。 | 设置 V、f、phase、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 FrequencyDrift 直接缩放按时间计算的相位表达式，不对瞬时频率积分。 |
| [`FaultableStepCurrent`](../Electrical/Analog/Sources/FaultableStepCurrent.mo) | 具有输出故障的阶跃电流源。 正常参考：`Modelica.Electrical.Analog.Sources.StepCurrent`。 | 设置 I、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableStepVoltage`](../Electrical/Analog/Sources/FaultableStepVoltage.mo) | 具有输出故障的阶跃电压源。 正常参考：`Modelica.Electrical.Analog.Sources.StepVoltage`。 | 设置 V、offset 与 startTime。 按规定物理量的正方向连接 p 与 n，并查看 effectiveOutput。 AmplitudeDrift 与 AmplitudeDrop 使用同一比例插值，并作用于含正常偏置的完整输出；StuckOutput 使用固定目标。 frequencyFault 对此源不起作用。 |
| [`FaultableSupplyVoltage`](../Electrical/Analog/Sources/FaultableSupplyVoltage.mo) | 具有电源轨故障的双轨电压源。 正常参考：`Modelica.Electrical.Analog.Sources.SupplyVoltage`。 | 连接 pin_p、pin_n 与 ground，设置 Vps、Vns，并按需配置 railScaleFault 或 groundShiftFault。 GroundShift 使两条电源轨相对所连接的 ground 引脚同向偏移，不改变外部网络规定的 ground 电位。 |

### 9.13. Mechanics

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Mechanics`](../Mechanics/package.mo) | 机械故障元件与传感器。 | 按物理接口与故障规律选择具体元件。 |

### 9.14. Mechanics.Rotational

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Rotational`](../Mechanics/Rotational/package.mo) | 转动故障模型与测量。 | 按物理接口与故障规律选择具体元件。 |

### 9.15. Mechanics.Rotational.Components

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Components`](../Mechanics/Rotational/Components/package.mo) | 具有可配置故障效应的转动元件。 | 按物理接口与故障规律选择具体元件。 |
| [`FaultableBearingFriction`](../Mechanics/Rotational/Components/FaultableBearingFriction.mo) | 可配置摩擦降低、磨损和抱死效应的轴承摩擦模型。 正常参考：`Modelica.Mechanics.Rotational.Components.BearingFriction`。 | 连接轴端法兰与可选支撑，设置 tau_pos 和对应摩擦倍率，查看 tau 与 lossPower。 Wear 即使使用外部触发也按配置的绝对起止时刻计算进度；激活或恢复渐变期间保留该倍率，效果归零后恢复名义值，并非永久磨损状态。 |
| [`FaultableBrake`](../Mechanics/Rotational/Components/FaultableBrake.mo) | 可分别改变静摩擦与滑动摩擦的转动制动器。 正常参考：`Modelica.Mechanics.Rotational.Components.Brake`。 | 输入 f_normalized，设置 fn_max、mu_pos 及相应的静摩擦或动摩擦倍率。 法向力非正时，即使选择 Sticking，制动器仍释放；增大的静摩擦阈值属于有限摩擦近似。 |
| [`FaultableClutch`](../Mechanics/Rotational/Components/FaultableClutch.mo) | 可分别改变静摩擦与滑动摩擦的转动离合器。 正常参考：`Modelica.Mechanics.Rotational.Components.Clutch`。 | 连接两根轴，输入 f_normalized，并设置 fn_max、mu_pos 和相应摩擦倍率。 法向力非正时离合器仍处于释放状态；结合 w_rel、tau 与 lossPower 区分滑动和摩擦锁止。 |
| [`FaultableDamper`](../Mechanics/Rotational/Components/FaultableDamper.mo) | 支持阻尼变化及渐进泄漏的转动阻尼器。 正常参考：`Modelica.Mechanics.Rotational.Components.Damper`。 | 连接两个转动法兰，按模式设置 dLoss、dIncrease、dLeak 或 dSeized。 泄漏随效果包络恢复；Seizure 采用较大的有限阻尼，不施加刚性约束。有效阻尼必须保持非负。 |
| [`FaultableElastoBacklash`](../Mechanics/Rotational/Components/FaultableElastoBacklash.mo) | 支持刚度、阻尼和断齿效应的转动侧隙模型。 正常参考：`Modelica.Mechanics.Rotational.Components.ElastoBacklash`。 | 连接轴端法兰，设置 c、d、总侧隙 b 与相应故障目标，查看 phi_rel 和 tau。 初始化时采用正则化接触关系；ToothBroken 会锁存，触发或时间窗结束后仍持续，完整强度下刚度和阻尼均降为零。 |
| [`FaultableElastoBacklash2`](../Mechanics/Rotational/Components/FaultableElastoBacklash2.mo) | 支持刚度、阻尼和断齿效应的转动侧隙模型，接触开始时允许力矩跳变。 正常参考：`Modelica.Mechanics.Rotational.Components.ElastoBacklash2`。 | 连接轴端法兰，设置 c、d、总侧隙 b 与相应故障目标，查看 phi_rel 和 tau。 初始化时采用正则化接触关系；ToothBroken 会锁存，触发或时间窗结束后仍持续，完整强度下刚度和阻尼均降为零。 |
| [`FaultableGearbox`](../Mechanics/Rotational/Components/FaultableGearbox.mo) | 通过附加力矩表示故障的弹性齿轮箱。 正常参考：`Modelica.Mechanics.Rotational.Components.Gearbox`。 | 设置 ratio、lossTable、c、d 和 b，连接轴与支撑，再选择附加力矩模式。 附加故障力矩不改变原有传动比或齿轮刚度，其做功未计入名义齿轮与侧隙元件的热端口热流。 |
| [`FaultableIdealGear`](../Mechanics/Rotational/Components/FaultableIdealGear.mo) | 支持传动比变化与规定齿损伤力矩的理想齿轮。 正常参考：`Modelica.Mechanics.Rotational.Components.IdealGear`。 | 连接两个轴端法兰与可选支撑，配置 ratioFault 或 toothTorque、toothFrequency。 理想约束使 LockedGear 所使用的运动学残差恒为零，因此该模式不会产生额外锁止力矩；变化的传动比属于规定运动学干预。 |
| [`FaultableIdealPlanetary`](../Mechanics/Rotational/Components/FaultableIdealPlanetary.mo) | 支持齿数比与附加力矩故障的行星齿轮。 正常参考：`Modelica.Mechanics.Rotational.Components.IdealPlanetary`。 | 连接 sun、carrier 与 ring，设置名义齿数比及 ratioFault 或 toothTorque，并查看三轴力矩。 附加力矩不直接施加刚性轴锁止；模型不包含惯量或独立的耗散热端口。 |
| [`FaultableIdealRollingWheel`](../Mechanics/Rotational/Components/FaultableIdealRollingWheel.mo) | 支持半径误差及运动学传动变化的理想滚动轮。 正常参考：`Modelica.Mechanics.Rotational.Components.IdealRollingWheel`。 | 连接 flangeR、flangeT 及相应支撑，设置 radiusFault 或 slipRatioFault，同时查看转动和平动位移。 LockedWheel 完全生效时，平动端相对支撑的位移为零。这些模式改变理想运动学关系，不描述轮胎接触或耗散型滑移规律。 |
| [`FaultableInertia`](../Mechanics/Rotational/Components/FaultableInertia.mo) | 以角动量平衡表示参数变化的转动惯量。 正常参考：`Modelica.Mechanics.Rotational.Components.Inertia`。 | 设置 J 与轴的初始状态，选择 JFault 或 lockDamping，并查看 momentum、storedEnergy、lossPower 和 parameterPower。 变化惯量要求两个端点值均为正；Drift 随激活包络变化，没有独立的老化计时。 |
| [`FaultableLossyGear`](../Mechanics/Rotational/Components/FaultableLossyGear.mo) | 保留名义啮合损耗并附加故障力矩的齿轮。 正常参考：`Modelica.Mechanics.Rotational.Components.LossyGear`。 | 设置 ratio 与 lossTable，再配置 ratioFault、lossTorqueFault 或 lockDamping；分别查看 addedLossTorque 与 lossPower。 lossPower 仅包含 tauLoss*w_a，附加故障力矩的做功不计入该输出或其热端口；wRegularization 应为正。 |
| [`FaultableOneWayClutch`](../Mechanics/Rotational/Components/FaultableOneWayClutch.mo) | 支持摩擦水平变化的单向离合器。 正常参考：`Modelica.Mechanics.Rotational.Components.OneWayClutch`。 | 连接两根轴，输入 f_normalized，设置摩擦表与相应倍率，查看 w_rel 和 tau。 模型保留名义单向运动约束；输入非正时释放摩擦离合部分，Sticking 通过摩擦倍率起作用，不另行增加刚性锁止。 |
| [`FaultableSpring`](../Mechanics/Rotational/Components/FaultableSpring.mo) | 支持刚度、预紧角变化与锁存断裂效应的转动弹簧。 正常参考：`Modelica.Mechanics.Rotational.Components.Spring`。 | 设置 c、phi_rel0 及目标刚度或 phiRel0Fault，查看 tau、storedEnergy 和 parameterPower。 parameterPower 为扣除机械端口功率后的储能变化率，不代表耗散热；Broken 在取消激活后仍持续。 |
| [`FaultableSpringDamper`](../Mechanics/Rotational/Components/FaultableSpringDamper.mo) | 转动弹簧阻尼组合，支持锁存断裂效应。 正常参考：`Modelica.Mechanics.Rotational.Components.SpringDamper`。 | 设置 c、d 与无变形参考坐标，选择 cFault、dFault 或 dSeized，分别观察弹性和阻尼作用。 若需增加阻尼，应设置 dFault>d；两种阻尼模式的默认目标均为 0.5*d。Broken 在触发结束后保持，完整强度下弹性与阻尼项均消失。 |

### 9.16. Mechanics.Rotational.Sensors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sensors`](../Mechanics/Rotational/Sensors/package.mo) | 支持输出故障的转动传感器。 | 按物理接口与故障规律选择具体元件。 |
| [`FaultableAccSensor`](../Mechanics/Rotational/Sensors/FaultableAccSensor.mo) | 支持输出故障的角加速度传感器。 正常参考：`Modelica.Mechanics.Rotational.Sensors.AccSensor`。 | 将 flange 接到被测物体；以 rad/s2 为单位读取 a，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableAngleSensor`](../Mechanics/Rotational/Sensors/FaultableAngleSensor.mo) | 支持输出故障的绝对轴转角传感器。 正常参考：`Modelica.Mechanics.Rotational.Sensors.AngleSensor`。 | 将 flange 接到被测物体；以 rad 为单位读取 phi，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultablePowerSensor`](../Mechanics/Rotational/Sensors/FaultablePowerSensor.mo) | 支持输出故障的转动传递功率传感器。 正常参考：`Modelica.Mechanics.Rotational.Sensors.PowerSensor`。 | 将 flange_a、flange_b 串联接入传力通路；以 W 为单位读取 power，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableSpeedSensor`](../Mechanics/Rotational/Sensors/FaultableSpeedSensor.mo) | 支持输出故障的角速度传感器。 正常参考：`Modelica.Mechanics.Rotational.Sensors.SpeedSensor`。 | 将 flange 接到被测物体；以 rad/s 为单位读取 w，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableTorqueSensor`](../Mechanics/Rotational/Sensors/FaultableTorqueSensor.mo) | 支持输出故障的传递力矩传感器。 正常参考：`Modelica.Mechanics.Rotational.Sensors.TorqueSensor`。 | 将 flange_a、flange_b 串联接入传力通路；以 N.m 为单位读取 tau，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |

### 9.17. Mechanics.Translational

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Translational`](../Mechanics/Translational/package.mo) | 平动故障模型与测量。 | 按物理接口与故障规律选择具体元件。 |

### 9.18. Mechanics.Translational.Components

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Components`](../Mechanics/Translational/Components/package.mo) | 具有可配置故障效应的平动元件。 | 按物理接口与故障规律选择具体元件。 |
| [`FaultableBrake`](../Mechanics/Translational/Components/FaultableBrake.mo) | 通过法向力倍率表示故障的平动制动器。 正常参考：`Modelica.Mechanics.Translational.Components.Brake`。 | 输入 f_normalized，设置 fn_max、mu_pos 及 frictionScaleFault 或 stickingScale，查看 f 与 lossPower。 法向力非正时制动器释放；Sticking 表示有限摩擦增加，不施加位置约束。 |
| [`FaultableDamper`](../Mechanics/Translational/Components/FaultableDamper.mo) | 支持有效阻尼配置的平动阻尼器。 正常参考：`Modelica.Mechanics.Translational.Components.Damper`。 | 连接两个平动法兰，配置 dFault、dIncrease 或 dSeized，查看 f 与 lossPower。 Leakage 直接随激活包络变化，没有独立泄漏计时；Seizure 采用较大的有限阻尼。 |
| [`FaultableElastoGap`](../Mechanics/Translational/Components/FaultableElastoGap.mo) | 支持刚度、阻尼及间隙变化的单侧弹性接触。 正常参考：`Modelica.Mechanics.Translational.Components.ElastoGap`。 | 连接两个法兰，设置 c、d、s_rel0 与指数 n，并观察所选故障下的 contact 和 f。 JammedContact 仍遵循单侧接触条件，不会将已分离的接触面黏合；初始力和位置约束需要一致。 |
| [`FaultableIdealGearR2T`](../Mechanics/Translational/Components/FaultableIdealGearR2T.mo) | 支持运动学故障的理想转动—平动转换器。 正常参考：`Modelica.Mechanics.Translational.Components.IdealGearR2T`。 | 连接 flangeR、flangeT 与所需支撑，设置 ratioFault 或 slipRatioFault，查看相对坐标。 LockedGear 完全生效时，转动端相对支撑的角位移为零；滑移模式改变理想运动学关系，不包含摩擦滑移或热损耗规律。 |
| [`FaultableIdealRollingWheel`](../Mechanics/Translational/Components/FaultableIdealRollingWheel.mo) | 支持半径误差及运动学传动变化的理想滚动轮。 正常参考：`Modelica.Mechanics.Translational.Components.IdealRollingWheel`。 | 连接 flangeR、flangeT 及相应支撑，设置 radiusFault 或 slipRatioFault，同时查看转动和平动位移。 LockedWheel 完全生效时，平动端相对支撑的位移为零。这些模式改变理想运动学关系，不描述轮胎接触或耗散型滑移规律。 |
| [`FaultableMass`](../Mechanics/Translational/Components/FaultableMass.mo) | 以动量平衡表示参数变化的平动质量块。 正常参考：`Modelica.Mechanics.Translational.Components.Mass`。 | 设置 m、初始位置与速度，选择 mFault 或 lockDamping，并查看 momentum 与能量诊断量。 变化质量要求两个端点值均为正；模型没有与质量变化对应的物质流入或流出端口。 |
| [`FaultableMassWithStopAndFriction`](../Mechanics/Translational/Components/FaultableMassWithStopAndFriction.mo) | 支持摩擦故障及限位变化的硬限位滑块。 正常参考：`Modelica.Mechanics.Translational.Components.MassWithStopAndFriction`。 | 设置 m、L、smin、smax 及摩擦参数，在限位内初始化质量块，查看 s、v、stopped 和 lossPower。 质量项采用 m_effective*der(v)，不含变质量动量项；lossPower 记录摩擦功率，不计入硬限位重置所移除的动能。 |
| [`FaultableRod`](../Mechanics/Translational/Components/FaultableRod.mo) | 附加轴向力作用下的无质量刚性杆。 正常参考：`Modelica.Mechanics.Translational.Components.Rod`。 | 连接两个法兰并设置刚性长度，配置 forceFault、damageFrequency，查看 faultForceValue。 RodDamage 不改变轴向刚度，也不切断杆连接；附加力属于外部机械作用，未提供显式反力端口。 |
| [`FaultableRollingResistance`](../Mechanics/Translational/Components/FaultableRollingResistance.mo) | 支持阻力幅值变化及周期力扰动的滚动阻力。 正常参考：`Modelica.Mechanics.Translational.Components.RollingResistance`。 | 设置轮载与滚阻系数，按需输入 cr、inclination，并选择低速正则化方式。 LockedWheel 增加滚阻但不约束车轮转动；周期幅值超过名义滚阻项时，合力可能改变方向。 |
| [`FaultableSpring`](../Mechanics/Translational/Components/FaultableSpring.mo) | 支持刚度、自由长度变化与锁存断裂效应的平动弹簧。 正常参考：`Modelica.Mechanics.Translational.Components.Spring`。 | 设置 c、s_rel0 及 cFault 或 sRel0Fault，查看 f、storedEnergy 和 parameterPower。 若需增加刚度，应设置 cFault>c，默认值为 0.5*c；参数变化功率余量不代表耗散热。 |
| [`FaultableSpringDamper`](../Mechanics/Translational/Components/FaultableSpringDamper.mo) | 平动弹簧阻尼组合，支持锁存断裂效应。 正常参考：`Modelica.Mechanics.Translational.Components.SpringDamper`。 | 设置 c、d 与无变形参考坐标，选择 cFault、dFault 或 dSeized，分别观察弹性和阻尼作用。 若需增加阻尼，应设置 dFault>d；两种阻尼模式的默认目标均为 0.5*d。Broken 在触发结束后保持，完整强度下弹性与阻尼项均消失。 |
| [`FaultableSupportFriction`](../Mechanics/Translational/Components/FaultableSupportFriction.mo) | 可配置摩擦倍率的支撑摩擦模型。 正常参考：`Modelica.Mechanics.Translational.Components.SupportFriction`。 | 连接刚性关联的两个法兰与可选支撑，设置 f_pos、peak 及相应摩擦倍率。 Wear 随激活包络变化，没有累积磨损状态；Seizure 增加有限摩擦，不新增位置约束。 |
| [`FaultableVehicle`](../Mechanics/Translational/Components/FaultableVehicle.mo) | 通过附加故障力表示异常的纵向车辆模型。 正常参考：`Modelica.Mechanics.Translational.Components.Vehicle`。 | 配置 m、J、R、气动及滚阻参数，连接 flangeR 或 flangeT，并为启用的风速、坡度接口提供输入。 addedMass 仅改变惯性力，名义重力与轮载滚阻仍由 m 决定；附加故障力源不提供显式支撑或热端口。 |

### 9.19. Mechanics.Translational.Sensors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sensors`](../Mechanics/Translational/Sensors/package.mo) | 支持输出故障的平动传感器。 | 按物理接口与故障规律选择具体元件。 |
| [`FaultableAccSensor`](../Mechanics/Translational/Sensors/FaultableAccSensor.mo) | 支持输出故障的绝对加速度传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.AccSensor`。 | 将 flange 接到被测物体；以 m/s2 为单位读取 a，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableForceSensor`](../Mechanics/Translational/Sensors/FaultableForceSensor.mo) | 支持输出故障的传递力传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.ForceSensor`。 | 将 flange_a、flange_b 串联接入传力通路；以 N 为单位读取 f，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultablePositionSensor`](../Mechanics/Translational/Sensors/FaultablePositionSensor.mo) | 支持输出故障的绝对位置传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.PositionSensor`。 | 将 flange 接到被测物体；以 m 为单位读取 s，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultablePowerSensor`](../Mechanics/Translational/Sensors/FaultablePowerSensor.mo) | 支持输出故障的平动传递功率传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.PowerSensor`。 | 将 flange_a、flange_b 串联接入传力通路；以 W 为单位读取 power，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableRelAccSensor`](../Mechanics/Translational/Sensors/FaultableRelAccSensor.mo) | 支持输出故障的相对加速度传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.RelAccSensor`。 | 将 flange_a、flange_b 分别接到两个测量位置；以 m/s2 为单位读取 a_rel，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableRelPositionSensor`](../Mechanics/Translational/Sensors/FaultableRelPositionSensor.mo) | 支持输出故障的相对位置传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.RelPositionSensor`。 | 将 flange_a、flange_b 分别接到两个测量位置；以 m 为单位读取 s_rel，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableRelSpeedSensor`](../Mechanics/Translational/Sensors/FaultableRelSpeedSensor.mo) | 支持输出故障的相对速度传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.RelSpeedSensor`。 | 将 flange_a、flange_b 分别接到两个测量位置；以 m/s 为单位读取 v_rel，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableSpeedSensor`](../Mechanics/Translational/Sensors/FaultableSpeedSensor.mo) | 支持输出故障的绝对速度传感器。 正常参考：`Modelica.Mechanics.Translational.Sensors.SpeedSensor`。 | 将 flange 接到被测物体；以 m/s 为单位读取 v，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |

### 9.20. Thermal

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Thermal`](../Thermal/package.mo) | 可配置故障的热学元件。 | 先选择热学连接器体系，再选择具体元件。 |

### 9.21. Thermal.FluidHeatFlow

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`FluidHeatFlow`](../Thermal/FluidHeatFlow/package.mo) | 简化热流体网络中的故障。 | 使用兼容的 FlowPort 连接，并保持介质记录一致。 |

### 9.22. Thermal.FluidHeatFlow.Components

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Components`](../Thermal/FluidHeatFlow/Components/package.mo) | 具有故障的简化管道与阀。 | 先配置正常特性，再设置故障目标。 |
| [`FaultableOneWayValve`](../Thermal/FluidHeatFlow/Components/FaultableOneWayValve.mo) | 具有分支状态与泄漏故障的单向阀。 正常参考：`Modelica.Thermal.FluidHeatFlow.Components.OneWayValve`。 | 连接 FlowPort 端口对，设置 V_flowNominal、dpForward、dpNominal 和 V_flowBackward；查看 backward_effective 与有效特性参数。 卡滞分支的强制选择要求 effectLevel>0.5；低于该阈值时参数插值仍可发生。 |
| [`FaultablePipe`](../Thermal/FluidHeatFlow/Components/FaultablePipe.mo) | 具有压降与热耦合故障的简化管道。 正常参考：`Modelica.Thermal.FluidHeatFlow.Components.Pipe`。 | 使用 FlowPort 连接，配置正常摩擦与高差，并查看 addedPressureDrop；启用热端口时，对比 Q_flowHeatPort 与缩放后的介质热输入。 Leakage 降低压降而不移除质量；HeatTransferLoss 仅缩放介质侧热输入，两侧热交换量可能不一致。 |
| [`FaultableValve`](../Thermal/FluidHeatFlow/Components/FaultableValve.mo) | 具有开度与泄漏故障的简化阀。 正常参考：`Modelica.Thermal.FluidHeatFlow.Components.Valve`。 | 连接 y 与 FlowPort 端口对，设置 y1、Kv1、kv0、dp0 和 rho0；查看 y_effective、kv0_effective 与 Kv。 零开度指令与 Blockage 仍保留非零闭阀流量特性。 |

### 9.23. Thermal.FluidHeatFlow.Sensors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sensors`](../Thermal/FluidHeatFlow/Sensors/package.mo) | 具有故障的简化热流体测量元件。 | 选择绝对量、相对量或通流测量，并按输出单位设置故障幅值。 |
| [`FaultableEnthalpyFlowSensor`](../Thermal/FluidHeatFlow/Sensors/FaultableEnthalpyFlowSensor.mo) | 具有输出故障的焓流测量。 正常参考：`Modelica.Thermal.FluidHeatFlow.Sensors.EnthalpyFlowSensor`。 | 将 FlowPort 端口对串联接入，以瓦特对比 y_actual 与 y。 使用 FluidHeatFlow 的 FlowPort 连接器与介质记录，其接口与 Modelica.Fluid 不同。 |
| [`FaultableMassFlowSensor`](../Thermal/FluidHeatFlow/Sensors/FaultableMassFlowSensor.mo) | 简化流体网络中的质量流量测量。 正常参考：`Modelica.Thermal.FluidHeatFlow.Sensors.MassFlowSensor`。 | 将 FlowPort 端口对串联接入，设置介质记录，并以 kg/s 对比 y_actual 与 y。 使用 FluidHeatFlow 的 FlowPort 连接器与介质记录，其接口与 Modelica.Fluid 不同。 |
| [`FaultablePressureSensor`](../Thermal/FluidHeatFlow/Sensors/FaultablePressureSensor.mo) | 简化流体网络中的压力测量。 正常参考：`Modelica.Thermal.FluidHeatFlow.Sensors.PressureSensor`。 | 将 flowPort 连接到所需节点，以帕斯卡设置压力故障幅值。 使用 FluidHeatFlow 的 FlowPort 连接器与介质记录，其接口与 Modelica.Fluid 不同。 |
| [`FaultableRelPressureSensor`](../Thermal/FluidHeatFlow/Sensors/FaultableRelPressureSensor.mo) | 具有输出故障的压差测量。 正常参考：`Modelica.Thermal.FluidHeatFlow.Sensors.RelPressureSensor`。 | 将两测量端口连接到目标节点，对比 y_actual 与 y，并保留 a 减 b 的符号约定。 使用 FluidHeatFlow 的 FlowPort 连接器与介质记录，其接口与 Modelica.Fluid 不同。 |
| [`FaultableRelTemperatureSensor`](../Thermal/FluidHeatFlow/Sensors/FaultableRelTemperatureSensor.mo) | 具有输出故障的温差测量。 正常参考：`Modelica.Thermal.FluidHeatFlow.Sensors.RelTemperatureSensor`。 | 使用一致的介质记录，以开尔文对比两测量端口间的 y_actual 与 y。 使用 FluidHeatFlow 的 FlowPort 连接器与介质记录，其接口与 Modelica.Fluid 不同。 |
| [`FaultableTemperatureSensor`](../Thermal/FluidHeatFlow/Sensors/FaultableTemperatureSensor.mo) | 简化流体网络中的温度测量。 正常参考：`Modelica.Thermal.FluidHeatFlow.Sensors.TemperatureSensor`。 | 连接 flowPort，设置介质记录，并以开尔文对比 y_actual 与 y。 使用 FluidHeatFlow 的 FlowPort 连接器与介质记录，其接口与 Modelica.Fluid 不同。 |
| [`FaultableVolumeFlowSensor`](../Thermal/FluidHeatFlow/Sensors/FaultableVolumeFlowSensor.mo) | 简化流体网络中的体积流量测量。 正常参考：`Modelica.Thermal.FluidHeatFlow.Sensors.VolumeFlowSensor`。 | 将 FlowPort 端口对串联接入，以 m3/s 对比 y_actual 与 y。 使用 FluidHeatFlow 的 FlowPort 连接器与介质记录，其接口与 Modelica.Fluid 不同。 |

### 9.24. Thermal.HeatTransfer

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`HeatTransfer`](../Thermal/HeatTransfer/package.mo) | 传热、热储能及热学测量。 | 使用 HeatPort 连接，并提供合适的温度或储能关系。 |

### 9.25. Thermal.HeatTransfer.Components

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Components`](../Thermal/HeatTransfer/Components/package.mo) | 热储能与传热故障。 | 按储能、传热或信号适配角色选择元件，查看各自故障模式。 |
| [`FaultableBodyRadiation`](../Thermal/HeatTransfer/Components/FaultableBodyRadiation.mo) | 具有辐射系数故障的辐射传热。 正常参考：`Modelica.Thermal.HeatTransfer.Components.BodyRadiation`。 | 连接两个热节点，设置 Gr 与故障目标，结合 Q_flow 查看 Gr_effective。 EmissivityDrift 与 SurfaceContamination 使用相同的激活包络关系；辐射系数应非负。 |
| [`FaultableConvection`](../Thermal/HeatTransfer/Components/FaultableConvection.mo) | 具有热导故障的对流传热。 正常参考：`Modelica.Thermal.HeatTransfer.Components.Convection`。 | 连接 solid 与 fluid 热端口，以 W/K 输入 Gc；设置 conductanceScaleFault，查看 Gc_effective 与 Q_flow。 FlowLoss 仅改变换热系数；模型不包含流体流动状态或流体端口。 |
| [`FaultableGeneralHeatFlowToTemperatureAdaptor`](../Thermal/HeatTransfer/Components/FaultableGeneralHeatFlowToTemperatureAdaptor.mo) | 具有温度输出故障的热流适配器。 正常参考：`Modelica.Thermal.HeatTransfer.Components.GeneralHeatFlowToTemperatureAdaptor`。 | 连接 heatPort 与继承的热流输入，对比 temperatureActual、继承的温度输出及启用的导数输出。 故障作用于报告温度；导数输出可能包含故障包络的导数。 |
| [`FaultableHeatCapacitor`](../Thermal/HeatTransfer/Components/FaultableHeatCapacitor.mo) | 具有可变热容的热能储存。 正常参考：`Modelica.Thermal.HeatTransfer.Components.HeatCapacitor`。 | 连接 port，设置 C 与初始 T，再选择正热容目标；同时查看 C_effective、T 和 storedEnergy。 storedEnergy 采用固定 0 K 基准；parameterPower 恒为零。ThermalRunaway 仅改变热容，SensorLag 增加物理热容。 |
| [`FaultableThermalConductor`](../Thermal/HeatTransfer/Components/FaultableThermalConductor.mo) | 具有可配置有限故障目标的热导。 正常参考：`Modelica.Thermal.HeatTransfer.Components.ThermalConductor`。 | 连接两热端口，设置 G 与相应热导目标，查看 G_effective、dT 和 Q_flow。 ConductanceDrift 与 ConductanceLoss 采用同一包络关系；开路与短路模式使用有限热导。 |
| [`FaultableThermalResistor`](../Thermal/HeatTransfer/Components/FaultableThermalResistor.mo) | 具有可配置有限故障目标的热阻。 正常参考：`Modelica.Thermal.HeatTransfer.Components.ThermalResistor`。 | 连接两热端口，设置 R 与相应热阻目标，结合 dT 和 Q_flow 查看 R_effective。 ResistanceDrift 与 ResistanceIncrease 使用同一包络关系；有限目标保留热通路。 |

### 9.26. Thermal.HeatTransfer.Sensors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sensors`](../Thermal/HeatTransfer/Sensors/package.mo) | 具有输出故障的热学测量元件。 | 热流传感器串联接入，温度传感器连接到相应热节点。 |
| [`FaultableHeatFlowSensor`](../Thermal/HeatTransfer/Sensors/FaultableHeatFlowSensor.mo) | 具有输出故障的热流测量。 正常参考：`Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor`。 | 将 port_a 和 port_b 串联接入热通路，以瓦特对比 Q_flow_actual 与 Q_flow。 NoiseIncrease 为确定性正弦信号；Stuck 向 stuckValue 插值，不保存激活时测量值。 |
| [`FaultableRelTemperatureSensor`](../Thermal/HeatTransfer/Sensors/FaultableRelTemperatureSensor.mo) | 具有输出故障的温差测量。 正常参考：`Modelica.Thermal.HeatTransfer.Sensors.RelTemperatureSensor`。 | 将两端口连接到测量节点，以开尔文对比 T_rel_actual 与 T_rel。 NoiseIncrease 为确定性正弦信号；Stuck 向 stuckValue 插值，不保存激活时测量值。 |
| [`FaultableTemperatureSensor`](../Thermal/HeatTransfer/Sensors/FaultableTemperatureSensor.mo) | 具有输出故障的温度测量。 正常参考：`Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor`。 | 将 port 接到被测热节点，以开尔文对比 T_actual 与 T。 NoiseIncrease 为确定性正弦信号；Stuck 向 stuckValue 插值，不保存激活时测量值。 |

### 9.27. Fluid

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Fluid`](../Fluid/package.mo) | 可配置故障的流体元件。 | 选择流体元件，统一 Medium，并检查其公开参数与连接器。 |

### 9.28. Fluid.Machines

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Machines`](../Fluid/Machines/package.mo) | 具有指令与轴端扰动的泵。 | 按控制设定值、规定转速或轴驱动方式选择泵。 |
| [`FaultableControlledPump`](../Fluid/Machines/FaultableControlledPump.mo) | 具有设定值故障的受控泵。 正常参考：`Modelica.Fluid.Machines.ControlledPump`。 | 设置 control_m_flow，选择可选设定值输入，并配置 massFlowBias、pressureBias、gainFault 或卡滞目标；查看 nominal.m_flow_set 或 nominal.p_set。 压力增益与输出损失作用于相对 port_a.p 的压升；模型不修改泵效率曲线。 |
| [`FaultablePrescribedPump`](../Fluid/Machines/FaultablePrescribedPump.mo) | 具有转速指令故障的规定转速泵。 正常参考：`Modelica.Fluid.Machines.PrescribedPump`。 | 选择 N_const 或启用并连接 N_in，配置三点泵曲线，查看 speedCommand、speedEffective 与 nominal.N_in。 SpeedDrift 没有独立漂移计时；实际施加的转速下限为 1e-3 rev/min。 |
| [`FaultablePump`](../Fluid/Machines/FaultablePump.mo) | 具有规定故障力矩的轴驱动泵。 正常参考：`Modelica.Fluid.Machines.Pump`。 | 将 shaft 接到机械驱动端并统一两流体端口的 Medium；设置 faultTorqueAmplitude、damageFrequency 或 seizureDamping，结合轴速查看 faultTorque。 EfficiencyLoss 与 BearingFrictionIncrease 使用相同的规定力矩关系；应在所接驱动系统中核对该力矩源的符号作用。 |

### 9.29. Fluid.Pipes

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Pipes`](../Fluid/Pipes/package.mo) | 具有附加压降关系的管道。 | 先选择储能与换热行为，再设置附加压降系数。 |
| [`FaultableDynamicPipe`](../Fluid/Pipes/FaultableDynamicPipe.mo) | 动态管道及附加二次压降。 正常参考：`Modelica.Fluid.Pipes.DynamicPipe`。 | 设置几何尺寸、nNodes 和 Medium；启用换热时连接 heatPorts。查看 K_fault 与附加压降。 nParallel 已声明但未传给内部 DynamicPipe；堵塞采用有限阻力近似。 |
| [`FaultableStaticPipe`](../Fluid/Pipes/FaultableStaticPipe.mo) | 静态管道及附加二次压降。 正常参考：`Modelica.Fluid.Pipes.StaticPipe`。 | 设置几何尺寸和 Medium，再选择 blockageCoefficient 或 completeBlockageCoefficient；查看 K_fault 与附加压降。 粗糙度与结垢模式选择附加压降系数，正常管道参数保持固定。 |

### 9.30. Fluid.Sensors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sensors`](../Fluid/Sensors/package.mo) | 具有信号故障的流体测量元件。 | 局部状态使用单端口传感器，通流量使用双端口传感器。 |
| [`FaultableDensity`](../Fluid/Sensors/FaultableDensity.mo) | 具有输出故障的密度测量。 正常参考：`Modelica.Fluid.Sensors.Density`。 | 连接 port，统一 Medium，对比 measuredActual 与 d。 NoiseIncrease 采用正弦项，Stuck 使用规定值，均不生成随机噪声。 |
| [`FaultableMassFlowRate`](../Fluid/Sensors/FaultableMassFlowRate.mo) | 具有输出故障的质量流量测量。 正常参考：`Modelica.Fluid.Sensors.MassFlowRate`。 | 将 port_a 与 port_b 串联接入并统一 Medium，对比 measuredActual 与 m_flow。 NoiseIncrease 采用正弦项，Stuck 使用规定值，均不生成随机噪声。 |
| [`FaultablePressure`](../Fluid/Sensors/FaultablePressure.mo) | 具有输出与采样保持故障的压力测量。 正常参考：`Modelica.Fluid.Sensors.Pressure`。 | 连接 port 并对比 measuredActual 与 p；Freeze 保持激活时压力，Stuck 使用 stuckValue。 NoiseIncrease 采用正弦项，Stuck 使用规定值，均不生成随机噪声。 |
| [`FaultableTemperature`](../Fluid/Sensors/FaultableTemperature.mo) | 具有输出故障的温度测量。 正常参考：`Modelica.Fluid.Sensors.Temperature`。 | 连接 port，统一 Medium，以开尔文设置故障幅值，并对比 measuredActual 与 T。 NoiseIncrease 采用正弦项，Stuck 使用规定值，均不生成随机噪声。 |
| [`FaultableVolumeFlowRate`](../Fluid/Sensors/FaultableVolumeFlowRate.mo) | 具有输出故障的体积流量测量。 正常参考：`Modelica.Fluid.Sensors.VolumeFlowRate`。 | 将两流体端口串联接入，统一 Medium，并对比 measuredActual 与 V_flow。 NoiseIncrease 采用正弦项，Stuck 使用规定值，均不生成随机噪声。 |

### 9.31. Fluid.Valves

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Valves`](../Fluid/Valves/package.mo) | 具有指令与通流能力故障的阀。 | 根据介质选择阀，并使用该类声明的指令接口。 |
| [`FaultableValveCompressible`](../Fluid/Valves/FaultableValveCompressible.mo) | 具有开度指令故障的可压缩流阀。 正常参考：`Modelica.Fluid.Valves.ValveCompressible`。 | 连接 opening 与两流体端口，配置正常流量数据，并查看传给内部阀的 opening_effective。 LeakageIncrease 改变最小开度，不增加独立泄漏支路。 |
| [`FaultableValveDiscrete`](../Fluid/Valves/FaultableValveDiscrete.mo) | 具有指令故障的布尔阀。 正常参考：`Modelica.Fluid.Valves.ValveDiscrete`。 | 连接 open，设置正常流量数据并查看 open_effective；severity 需大于 0.5 才能越过覆盖阈值。 severity≤0.5 时指令不变；LeakageIncrease 强制开启指令，而非施加部分泄漏。 |
| [`FaultableValveDiscreteRamp`](../Fluid/Valves/FaultableValveDiscreteRamp.mo) | 具有指令故障的渐变布尔阀。 正常参考：`Modelica.Fluid.Valves.ValveDiscreteRamp`。 | 连接 open，设置正常流量数据并查看 open_effective；severity 需大于 0.5 才能越过覆盖阈值。Topen、Tclose 与 transitionTime 分别设置。 severity≤0.5 时指令不变；LeakageIncrease 强制开启指令，而非施加部分泄漏。 |
| [`FaultableValveLinear`](../Fluid/Valves/FaultableValveLinear.mo) | 具有开度与流量系数故障的线性阀。 正常参考：`Modelica.Fluid.Valves.ValveLinear`。 | 连接 opening 与流体端口，统一 Medium，设置 dp_nominal 与 m_flow_nominal；查看 opening_effective、coefficientScale、leakage 和 m_flow。 opening_nominal 不参与 k 的计算；opening_min 仅在效果强度为正时限制开度。Freeze 保存指令，StuckClosed 使用 stuckOpening。 |
| [`FaultableValveVaporizing`](../Fluid/Valves/FaultableValveVaporizing.mo) | 具有开度指令故障的汽化流阀。 正常参考：`Modelica.Fluid.Valves.ValveVaporizing`。 | 选择兼容的两相 Medium 与受支持的 CvData，连接 opening 并查看 opening_effective；启用 filteredOpening 时同时查看 opening_filtered。 CavitationIncrease 以降低开度近似；正常 ValveVaporizing 不支持 CvData.OpPoint。 |

### 9.32. Magnetic

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Magnetic`](../Magnetic/package.mo) | 磁路故障元件与传感器。 | 按物理接口与故障规律选择具体元件。 |

### 9.33. Magnetic.FluxTubes

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`FluxTubes`](../Magnetic/FluxTubes/package.mo) | 磁通管故障模型。 | 按物理接口与故障规律选择具体元件。 |

### 9.34. Magnetic.FluxTubes.Basic

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Basic`](../Magnetic/FluxTubes/Basic/package.mo) | 支持可配置故障的磁路与转换器。 | 按物理接口与故障规律选择具体元件。 |
| [`FaultableConstantPermeance`](../Magnetic/FluxTubes/Basic/FaultableConstantPermeance.mo) | 支持磁导退化与锁存铁芯开裂效应的恒定磁导。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.ConstantPermeance`。 | 连接 port_p、port_n，设置 G_m 与相应目标，查看 G_m_effective 和 Phi。 Drift 直接随激活包络变化，不包含独立材料老化规律；应选择适合所接磁路的非负磁导。 |
| [`FaultableConstantReluctance`](../Magnetic/FluxTubes/Basic/FaultableConstantReluctance.mo) | 支持磁阻退化与锁存铁芯开裂效应的恒定磁阻。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.ConstantReluctance`。 | 连接磁端口，设置 R_m、R_mFault 或有限开路/短路目标，查看 R_m_effective 和 Phi。 有效磁阻必须保持为正；Drift 没有独立于激活包络的进度状态。 |
| [`FaultableEddyCurrent`](../Magnetic/FluxTubes/Basic/FaultableEddyCurrent.mo) | 通过有效电导变化表示故障的涡流损耗元件。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.EddyCurrent`。 | 选择直接给定 G 或由 rho、l、A 计算电导，设置 conductanceFaultFactor，并按需连接热端口。 通路模式采用有限倍率，不改变拓扑；若要符合模式名所指的增加或减小方向，倍率应大于一。 |
| [`FaultableElectroMagneticConverter`](../Magnetic/FluxTubes/Basic/FaultableElectroMagneticConverter.mo) | 支持匝数及绕组电阻故障的电磁转换器。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverter`。 | 连接 p、n 与两个磁端口，设置 N、NFault 和绕组电阻目标，查看各项功率诊断量。 parameterPower=lossPower-electricalPower-magneticPower 按声明的端口方向记录变匝数功率项；ShortedTurns 是匝数和电阻简化模型，不包含独立的短路绕组回路。 |
| [`FaultableElectroMagneticConverterWithLeakageInductance`](../Magnetic/FluxTubes/Basic/FaultableElectroMagneticConverterWithLeakageInductance.mo) | 支持匝数损失及漏磁变化的电磁转换器。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.ElectroMagneticConverterWithLeakageInductance`。 | 连接电端口与磁端口，配置 N、漏磁几何参数及 NFault 或 leakageFaultFactor，查看 Phi_ind 与 Phi_leak。 电气关系采用 N_effective*der(Phi_ind)，未对 N_effective 求导；ShortedTurns 仅改变有效匝数，本模型未提供独立损耗或参数变化功率诊断量。 |
| [`FaultableLeakageWithCoefficient`](../Magnetic/FluxTubes/Basic/FaultableLeakageWithCoefficient.mo) | 通过有效磁通系数变化表示故障的漏磁通路。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.LeakageWithCoefficient`。 | 连接漏磁通路并输入 R_mUsefulTot，设置 c_usefulFlux 和 cFault 以形成所需耦合变化。 在有效磁路磁阻固定时，有效磁通系数越低，漏磁越大；LeakagePathOpen 接近较大有限磁阻，不断开端口。 |
| [`FaultableVariablePermeance`](../Magnetic/FluxTubes/Basic/FaultableVariablePermeance.mo) | 支持输入故障的信号控制磁导模型。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.VariablePermeance`。 | 以 H 为单位输入 G_m，连接磁端口，并设置卡值或有限开路、短路目标。 Stuck 使用规定目标值，不采集输入快照；GainError 在完整强度下达到零，应检查相应磁路约束与输入范围。 |
| [`FaultableVariableReluctance`](../Magnetic/FluxTubes/Basic/FaultableVariableReluctance.mo) | 支持输入故障的信号控制磁阻模型。 正常参考：`Modelica.Magnetic.FluxTubes.Basic.VariableReluctance`。 | 以 H-1 为单位输入 R_m，连接磁端口，并设置卡值或有限开路、短路目标。 Stuck 使用规定目标值，不采集输入快照；GainError 在完整强度下达到零，应检查相应磁路约束与输入范围。 |

### 9.35. Magnetic.FluxTubes.Sensors

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Sensors`](../Magnetic/FluxTubes/Sensors/package.mo) | 支持输出故障的磁测量。 | 按物理接口与故障规律选择具体元件。 |
| [`FaultableMagneticFluxSensor`](../Magnetic/FluxTubes/Sensors/FaultableMagneticFluxSensor.mo) | 支持输出故障的磁通传感器。 正常参考：`Modelica.Magnetic.FluxTubes.Sensors.MagneticFluxSensor`。 | 将 port_p、port_n 串联接入磁路；以 Wb 为单位读取 Phi，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |
| [`FaultableMagneticPotentialDifferenceSensor`](../Magnetic/FluxTubes/Sensors/FaultableMagneticPotentialDifferenceSensor.mo) | 支持输出故障的磁势差传感器。 正常参考：`Modelica.Magnetic.FluxTubes.Sensors.MagneticPotentialDifferenceSensor`。 | 将 port_p、port_n 跨接在待测磁路两端；以 A 为单位读取 V_m，配置 biasFault、gainFault、stuckValue 或 saturationLimit。 选择 Saturation 后会直接限制到 +/-saturationLimit，不受 enabled、severity、时间窗或触发信号控制。未修改的基准应选择 Normal；其他模式使用激活包络。 |

### 9.36. Utilities

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Utilities`](../Utilities/package.mo) | 激活逻辑与示例支撑。 | 包的用途与内容。 |

### 9.37. Utilities.Internal

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Internal`](../Utilities/Internal/package.mo) | 内部激活逻辑及示例装配模型。 | 包的用途与内容。 |
| [`Activation`](../Utilities/Internal/Activation.mo) | 具有可逆渐变及可选锁存的强度激活模块。 | 绑定 externalTrigger 并配置请求；分别读取 active 和 effectLevel。elapsedActiveTime 在激活时重置，在恢复期间保持已完成的激活时长。 |
| [`CapacitorCircuit`](../Utilities/Internal/CapacitorCircuit.mo) | 具有可替换电容的 RC 充电电路。 | 重新声明 device，并保持正常电容值 C 和初始电压设置；在相同 stepTime 下比较 readout。 |
| [`ConductiveCircuit`](../Utilities/Internal/ConductiveCircuit.mo) | 具有可替换导热通路的受热容体。 | 分别重新声明 capacity 或 heatPath，比较热容或热导变化。 |
| [`ConductorCircuit`](../Utilities/Internal/ConductorCircuit.mo) | 具有可替换电导元件的恒压电路。 | 重新声明 device，将 readout 与正常电流 1.2 A 比较。 |
| [`ConvectiveCircuit`](../Utilities/Internal/ConvectiveCircuit.mo) | 具有可替换对流通路的受热容体。 | 重新声明 heatPath，在相同热导输入和热初始条件下比较 temperature。 |
| [`ElectromagneticCircuit`](../Utilities/Internal/ElectromagneticCircuit.mo) | 可替换电磁转换器的电气激励电路。 | 重新声明 converter，同时观察 current 与 flux。 |
| [`ElectromechanicalCircuit`](../Utilities/Internal/ElectromechanicalCircuit.mo) | 电机、绕组发热与转速反馈装配模型。 | 重新声明 winding 或 feedback，在相同指令下比较 current、speed 和 temperature。 |
| [`InductorCircuit`](../Utilities/Internal/InductorCircuit.mo) | 具有可替换电感的 RL 电路。 | 重新声明 device，保持电流初值设置，并比较 stepTime 之后的电流上升过程。 |
| [`ReluctanceCircuit`](../Utilities/Internal/ReluctanceCircuit.mo) | 具有可替换磁阻的磁路。 | 重新声明 device，将 flux 与正常磁通 0.1 Wb 比较。 |
| [`ResistorCircuit`](../Utilities/Internal/ResistorCircuit.mo) | 具有可替换电阻的恒压电路。 | 重新声明 device，将 readout 与正常电流 1.2 A 比较。 |
| [`RotationalDrive`](../Utilities/Internal/RotationalDrive.mo) | 由转矩驱动且连接接地阻尼器的转动惯量。 | 分别重新声明 rotor 或 damper，区分瞬态响应与稳态转速变化。 |
| [`RotationalSpringPath`](../Utilities/Internal/RotationalSpringPath.mo) | 规定角位移作用下的可替换扭转弹簧。 | 重新声明 spring，在相同规定角位移下比较 torque。 |
| [`SensorCircuit`](../Utilities/Internal/SensorCircuit.mo) | 用于测量故障对照的电路。 | 重新声明传感器，将其读数与电路真实量比较。 |
| [`SignalCircuit`](../Utilities/Internal/SignalCircuit.mo) | 用于增益、一阶与限幅模块对照的公共斜坡输入。 | 重新声明 gain、lag 或 limit，观察 gainValue、lagValue 或 limitValue。 |
| [`TranslationalDrive`](../Utilities/Internal/TranslationalDrive.mo) | 由恒力驱动且连接并联弹簧阻尼器的质量块。 | 依次重新声明 mass、spring 或 damper 中的一个元件，比较 position。 |
| [`ValveCircuit`](../Utilities/Internal/ValveCircuit.mo) | 带边界、可替换阀及压力测量的流体通路。 | 重新声明 valve 或 sensor，保持相同 Medium，并比较 massFlow 与 pressure。 |

### 9.38. UsersGuide

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`UsersGuide`](../UsersGuide/package.mo) | 工具内使用指南。 | 包的用途与内容。 |
| [`Conventions`](../UsersGuide/Conventions.mo) | 故障选择、激活与诊断约定。 | 配置起始条件、强度和恢复前，先查看具体模式的计算关系及诊断量。 |
| [`GettingStarted`](../UsersGuide/GettingStarted.mo) | 加载库并完成首个故障对照。 | 运行 Examples.ResistorReplacement，对照电流输出与激活诊断量。 |
| [`Limitations`](../UsersGuide/Limitations.mo) | 建模假设及元件使用边界。 | 检查接口兼容性、有限故障目标值及具体模式的使用边界。 |
| [`Overview`](../UsersGuide/Overview.mo) | 库的用途与包结构。 | 先选择物理领域与正常接口，再选择故障模式。 |

### 9.39. Examples

| 类 | 用途及正常参考 | 配置与使用边界 |
| --- | --- | --- |
| [`Examples`](../Examples/package.mo) | 元件故障与正常工况对照示例。 | 包的用途与内容。 |
| [`CapacitorDegradation`](../Examples/CapacitorDegradation.mo) | 比较 RC 充电与电容衰减。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`ElectromechanicalFaultEffects`](../Examples/ElectromechanicalFaultEffects.mo) | 比较绕组、反馈与电磁转换故障。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`FluidValveDegradation`](../Examples/FluidValveDegradation.mo) | 比较阀驱动与压力测量故障。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`InductorDegradation`](../Examples/InductorDegradation.mo) | 比较 RL 电流与电感衰减。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`MagneticReluctance`](../Examples/MagneticReluctance.mo) | 比较磁阻增大前后的磁通。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`ResistorReplacement`](../Examples/ResistorReplacement.mo) | 比较可恢复的电阻与电导故障。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`RotationalDamping`](../Examples/RotationalDamping.mo) | 比较阻尼、惯量与扭转刚度变化。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`SensorAndSignalFaults`](../Examples/SensorAndSignalFaults.mo) | 比较输出保持、固定值及测量偏置。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`ThermalDegradation`](../Examples/ThermalDegradation.mo) | 比较热容、导热与对流故障。 | 使用自带实验设置运行，观察要点见第 10 节。 |
| [`TranslationalStiffness`](../Examples/TranslationalStiffness.mo) | 比较刚度、质量与阻尼变化。 | 使用自带实验设置运行，观察要点见第 10 节。 |

## 10. 自带示例

每个示例均包含激励源、边界条件及实验设置。按所需接口选取示例，从表中列出的输出开始观察。

| 示例 | 仿真区间 | 观察要点 |
| --- | --- | --- |
| [`CapacitorDegradation`](../Examples/CapacitorDegradation.mo) | 0–5 s | 绘制 nativeValue、normalValue 和 faultedValue；使用 dynamicLoss 时同时查看 faulted.device 的电荷与能量诊断量。 |
| [`ElectromechanicalFaultEffects`](../Examples/ElectromechanicalFaultEffects.mo) | 0–8 s | 比较电机电流、转速和温度；将转换器电流与磁通作为独立实验查看。 |
| [`FluidValveDegradation`](../Examples/FluidValveDegradation.mo) | 0–5 s | 绘制三路流量和三路压力输出，分别切换 reverseFlow 与 freezeCommand 进行比较。 |
| [`InductorDegradation`](../Examples/InductorDegradation.mo) | 0–0.2 s | 绘制三路 Value 输出，比较电压阶跃后的电流上升过程。 |
| [`MagneticReluctance`](../Examples/MagneticReluctance.mo) | 0–2 s | 绘制 nativeValue、normalValue 和 faultedValue。 |
| [`ResistorReplacement`](../Examples/ResistorReplacement.mo) | 0–5 s | 绘制六路电流输出及 resistorFault.device.effectLevel，每次只改变一个所选模式。 |
| [`RotationalDamping`](../Examples/RotationalDamping.mo) | 0–5 s | 绘制四路转速和三路弹簧转矩输出，分别查看各独立通路。 |
| [`SensorAndSignalFaults`](../Examples/SensorAndSignalFaults.mo) | 0–4 s | 绘制信号与传感器输出，另查看 faultSignal.limit.y 和 faultSensing.actualCurrent，以读取限幅器输出与真实电路电流。 |
| [`ThermalDegradation`](../Examples/ThermalDegradation.mo) | 0–60 s | 绘制七路温度输出，分别比较升温过程与最终温度。 |
| [`TranslationalStiffness`](../Examples/TranslationalStiffness.mo) | 0–10 s | 绘制五路位置输出，分别比较稳定位置、振荡和衰减。 |

## 11. 排错与建模边界

| 现象 | 检查内容 |
| --- | --- |
| 没有明显故障效果 | 检查具体模式、启用状态、强度、时间窗或触发信号、目标幅值以及覆盖阈值。目标与正常值相同时，参数不会变化。 |
| 窗口开始前结果已不同 | 检查初始条件、正常修饰参数、持续应用的饱和限制，以及是否从初始化时就配置了故障。 |
| 取消激活后效果仍然存在 | 检查恢复包络、永久锁存，以及具体模式的保持或漂移关系。 |
| 增加或损失模式使数值向相反方向变化 | 检查数值目标；多种命名不同的模式使用相同插值关系。 |
| 重新声明或连接器匹配失败 | 检查声明的约束类、公开接口、介质包与条件端口。 |
| 物理瞬态难以积分 | 检查有限开路或短路目标、锁止阻力、参数过渡时间、源约束及状态初始条件。 |
| 储能或损耗输出似乎不完整 | 查看元件实际采用的平衡关系和诊断量定义，包括参数变化功率项，以及未计入的碰撞或附加故障力矩做功。 |

目标幅值与有限近似应对实际应用有明确意义。自带示例只覆盖选定配置，应在模型实际使用的 Modelica 环境中比较响应。本指南中的解析值均由所列方程和参数推导得到。

## 12. 文档与延伸阅读

- [英文项目说明](../README.md) / [中文项目说明](../README.zh-CN.md)
- [工具内使用指南](../UsersGuide/package.mo)
- [根包定义](../package.mo)
- [Buildings Library User Guide — Getting Started](https://simulationresearch.lbl.gov/modelica/userGuide/gettingStarted.html)
- [Buildings — SimpleHouse 分步教程](https://simulationresearch.lbl.gov/modelica/releases/v11.1.2/help/Buildings_Examples_Tutorial_SimpleHouse.html)
- [Modelica 文档约定](https://doc.modelica.org/Modelica%204.0.0/Resources/helpDymola/Modelica_UsersGuide_Conventions_Documentation.html)
- [项目许可证](../LICENSE)与[保留的第三方声明](../NOTICE.md)
