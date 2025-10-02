# IL2206 Lab 2: Introduction to Real-Time Operating Systems

本项目仅供参考，KTH 的学生严禁将此代码作为实验报告或作业提交。因违反学术诚信而造成的任何后果，作者概不负责。

This repository is for **reference only**. Submitting this project as a lab report or assignment is **strictly prohibited**. The author does **not** take any responsibility for consequences arising from violations of the KTH honor code.

本文档不支持英文。

**This document is written in Chinese , for English reader, I recommend [Immersive Translator](https://immersivetranslate.com/en/) for translation.**

本文档会简单介绍一下每个task的思路和代码构成。

## Part 1: Introductory Tasks

### Task 1: 交通信号灯

| 时长 | 红灯 | 黄灯 | 绿灯 |
| ---- | ---- | ---- | ---- |
| 3 秒 | On   | Off  | Off  |
| 1秒  | On   | On   | Off  |
| 3 秒 | Off  | Off  | On   |
| 1 秒 | Off  | On   | Off  |



没啥好说的，这是一个简单的裸机任务。要求按照上表顺序亮灯，不能用RTOS。

代码位置在[这里](https://github.com/MosyuSora/FreeRTOSLab/blob/main/ES-Lab-Kit/Software/Projects/task1_1_traffic_light/main.c)。直接While里面用sleep就行了。



### Task 2: 握手亮灯

| 红灯 | 绿灯 |
| ---- | ---- |
| On   | On   |
| On   | Off  |
| Off  | Off  |
| Off  | On   |
| On   | On   |
| ...  | ...  |

题目基于Free-RTOS，要求建立红灯和绿灯两个Task，按照表中的时序要求实现红灯和绿灯的交替闪烁，一行的时间持续2s.

代码位置在[这里](https://github.com/MosyuSora/FreeRTOSLab/blob/main/ES-Lab-Kit/Software/Projects/task1_2_handshake/main.c)。

思路大致如下：首先我们每次循环我们都拆分成四个子状态，红灯依次是亮|亮|暗|暗，绿灯依次是亮|暗|暗|亮。同时我们建立两个信号量`semRed`和`semGreen`。**在每个子状态结束后，他们都互相发给对方所需要的信号量，然后阻塞着等待对面过来的信号量**，也就是握手（Handshake）。



### Task3  共享内存

任务A和任务B同时处理一块共用内存`shareMemory`（可以是全局变量，也可以是堆，这里我就简单写了个全局变量）。A有一个计数器`number`， 代表当前循环次数。 A在每次循环中需要把当前的`number`写入共用内存，然后通知B。 B则需要在收到通知后修改值（乘以-1）。然后通知A打印。效果大概如下：

```raw
Sending   : 1
Receiving : -1
Sending   : 2
Receiving : -2
```



不难看出每次写操作结束后，需要发送信号量来同步。所以时序就是

>  A写`shareMemory`->A发`semB`->A等`semA`->A读`shareMemory`

>  B等`semB` ->B写`shareMemory`->B发`semA`

但这样的共享内存是没有原子性的。我们还需要声明一个互斥锁`mutex`, 在写`shareMemory`前加入取锁操作, 写`shareMemory`后加入上锁操作。

整个程序框图如下图所示，代码在[这里](https://github.com/MosyuSora/FreeRTOSLab/blob/main/ES-Lab-Kit/Software/Projects/task1_3_share_memory/main.c)。

![图片](https://github.com/MosyuSora/FreeRTOSLab/blob/main/ES-Lab-Kit/Software/Projects/task1_3_share_memory/figure.png?raw=true)

## Part 2: Crusie Control

### Task 1~2： 实现基本功能

#### 任务介绍

简单来说，我们需要实现一个有定速巡航功能的玩具车。 

**用户输入**：有3个按键（按下为低电平）和8个开关。 按键从SW5~SW7分别是油门, 巡航使能, 和刹车。八个开关则是模拟一个8bit数的输入，通过输入的大小模拟MCU额外的负载。

**系统输出**：由三个显示组件搭配系统的串口打印组成；

* 环形led(1-24): 负责显示车辆的当前位置(对应position 0-24000m)；
* 红黄绿三个led: 分别代表刹车，巡航使能和油门；
* 四个七段数显管: 低两位负责显示车辆当前的速度(velocity,0-60.0 取整)， 高两位则负责显示油门量(throttle 0-80);
* 串口: 系统应当设立看门狗任务和喂狗任务来监测MCU的负载情况(用前面提到的开关来模拟负载大小)。当系统超载时，超载信息通过串口打印上传。

**预定义的内部信号和接口**：骨架文件给出了一些基本的接口，包括预定义的消息队列，任务和用到的工具函数

* 消息队列（*直接理解成全局变量，因为不涉及生产者消费者问题*）：
  * `xQueueThrottle`: 负责存放油门量，范围0~80, 类型为`uint16_t`
  * `xQueuePosition`,  `xQueueVelocity`: 负责存放车辆的位置和速度,位置的范围是0-24000, 速度范围是0-600(0-60.0), 类型为`uint16_t`
  * `xQueueBrakePedal`, `xQueueGasPedal`和`xQueueCruiseControl `：负责存放刹车，油门和巡航使能三个按钮的状态，类型为`bool`。
* 预定义任务
  * `vVehicleTask`: 一个周期为100的任务,负责读取油门值, 实时计算汽车的位置和速度。 任务文件中描述的诸如风阻，赛道坡度之类的情况都被放在这个任务中处理。作为学生无需修改这个任务。
* 工具函数
  * `uint16_t adjust_position`: `vVehicleTask`用到的工具函数, 用来求位置, 不用改
  * `uint16_t adjust_velocity `: `vVehicleTask`用到的工具函数, 用来求速度, 不用改

#### 代码架构

##### 基本结构

代码部分依次包括

1. 宏定义: 油门增量, PID参数

2. 任务句柄(`TaskHandle_t`)

3. 任务函数的原型
4. 信号量和队列的句柄(`SemaphoreHandle_t` `QueueHandle_t`)
5. 工具函数
6. 任务函数的实现
7. 主函数

##### 任务拆分和优先级

按照题目要求, 除了看门狗和喂狗两个任务,其余函数都应当按照RMS规则规划优先级. 任务的周期是由题目要求给出的.

| 任务                   | 内容                                                         | 周期 | 优先级 |
| ---------------------- | ------------------------------------------------------------ | ---- | ------ |
| vWatchDogTask          | 看门狗任务,每隔一段时间吃掉狗粮(拿走信号量 `xSemaphoreWatchDogFood`), 没有狗粮吃则通过串口告警:"System Overload!" | 1000 | 10     |
| vExtraLoadTask         | 模拟MCU的额外负载, 依次读取8个开关的状态(0-255), 用这个数除以10应当得到一个0-25的时间.然后调用`busy_wait(delay_ms)`执行忙等.<br />因为这个任务的周期是25,且优先级高于其他任务, 如果开关的输入大于250, 相当于该任务在周期内不会进入阻塞态, 其他任务必然会被饿死. 此时系统不会对任何输出产生反应.显示设备也不会工作! | 25   | 9      |
| vButtonTask            | 读取三个按钮的电平状态,把数传给相应的队列.<br />激活定速巡航时, 汽车当前的速度会被写到` xQueueTargetVelocity`这个队列中. | 50   | 8      |
| vVehicleTask           | 项目代码自带, 负责读取油门值, 实时计算汽车的位置和速度,无需修改(前面已介绍). | 100  | 7      |
| vControlTask           | 基于一个简单的PID控制器实现定速巡航功能, 以适应赛道中存在的上下坡和风阻等问题. 输入是车速, 输出是油门量. PID的参数在宏定义中给出. <br />*注: PID参数即使我没怎么调, 有明显的超调问题, 最终也还是落在题目允许的误差范围内* | 200  | 6      |
| vDisplayTask           | 负责和七段数显管, LED阵列和三个led状态灯的交互. 这一段涉及到开发板自带的bsp包接口的使用. 细节问题需要看example程序中的写法.<br />led阵列主要是写了一个工具函数`write_position()`,把位置换算成索引,再用右移运算求出寄存器值(32位整数),最后使用`BSP_ShiftRegWriteAll()`写入寄存器<br />七段数显管主要用到`sprintf()`把值转换为字符串, 然后用`BSP_7SegDispString()`写入寄存器 | 500  | 5      |
| vOverloadDetectionTask | 喂狗任务,优先级最低且没有周期,有空就喂. 喂狗操作就是把二值信号量 `xSemaphoreWatchDogFood`置为一. | NULL | 1      |

#### 代码实现

代码见[这里](https://github.com/MosyuSora/FreeRTOSLab/blob/main/ES-Lab-Kit/Software/Projects/task2_crusie_control/main.c).

需要自己写的, 除了上面讲的任务函数的实现, 还包括里面提到的几个工具函数`write_position()`,`calc_throttle_with_PID()`和`busy_wait()`.



### Task3 附加任务: 基于加速度计实现坡度检测

题目要求三选一实现一个附加任务, 我们这里选择完成加速度计任务. 要求把原来显示油门的高两位的七段数显管变成实时显示当前的坡度.

简单来说, 加速度计有三个轴(x,y,z). 他会返回这个轴读到的加速度大小, 单位是g. 对于我们检测坡度来说, 只需要选择一个轴即可. 这里我们选择x轴. 

思路上非常简单, 首先还是参考example给的例程, 通过 `BSP_GetAxisAcceleration(X_AXIS)`这个函数拿到传感器x轴的值. 此时我们拿到的数应该是一个`float`, 范围从-1.0到+1.0之间, 代表-1g~+1g. 然后我们只需要对它取一个绝对值, 然后把这个数乘以90, 就是当前的坡度值了.

当然在实现上, 我们需要小心处理下面几个问题:

* **传感器信号平滑**: 传感器的信号非常的抖, 我们需要采用平均值滤波(或者任何你喜欢的数字滤波方法)进行去抖. 因为赶时间, 于是我们做一个简单的平均值滤波.
* **数据类型转换**:应当先取绝对值再强转为uint, 不然负值情况下会无法工作
* **坡度任务的优先级问题**: 系统自带的`vVehicleTask`涉及大量数学运算, 占用率非常高. 把坡度任务放在他下面很容易挨饿. 实践上我们把坡度任务放在`vButtonTask`后面, `vVehicleTask`前面, 能起到不错的效果.

下面是程序的任务拆解,具体实现代码在[这里](https://github.com/MosyuSora/FreeRTOSLab/blob/main/ES-Lab-Kit/Software/Projects/task2_conditional_task/main.c#L330)

| 任务                   | 内容                                                         | 周期    | 优先级 |
| ---------------------- | ------------------------------------------------------------ | ------- | ------ |
| vWatchDogTask          | 看门狗任务,每隔一段时间吃掉狗粮(拿走信号量 `xSemaphoreWatchDogFood`), 没有狗粮吃则通过串口告警:"System Overload!" | 1000    | 10     |
| vButtonTask            | 读取三个按钮的电平状态,把数传给相应的队列.<br />激活定速巡航时, 汽车当前的速度会被写到` xQueueTargetVelocity`这个队列中. | 50      | 9      |
| **vSlopeTask**         | **读取加速度传感器x轴的值, 多次采样取平均, 然后传给队列`xQueueSlope`** | **100** | **8**  |
| vVehicleTask           | 项目代码自带, 负责读取油门值, 实时计算汽车的位置和速度,无需修改(前面已介绍). | 100     | 7      |
| vControlTask           | 基于一个简单的PID控制器实现定速巡航功能, 以适应赛道中存在的上下坡和风阻等问题. 输入是车速, 输出是油门量. PID的参数在宏定义中给出. | 200     | 6      |
| vDisplayTask           | 负责和七段数显管, LED阵列和三个led状态灯的交互.              | 500     | 5      |
| vOverloadDetectionTask | 喂狗任务,优先级最低且没有周期,有空就喂. 喂狗操作就是把二值信号量 `xSemaphoreWatchDogFood`置为一. | NULL    | 1      |

