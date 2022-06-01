本项目需要设计一个电机驱动板，控制**8个CAN协议的伺服电机**。为了锻炼我对RT-Thread的使用能力同时加快开发进度，减少花费在驱动代码上的时间，电机驱动板采用了RT-Thread操作系统。
# 资料连接
本项目的所有资料全部开源：

**硬件工程**：[https://lceda.cn/FranHawk/485tocan_motor_controller](https://lceda.cn/FranHawk/485tocan_motor_controller)
**软件工程**：[https://github.com/FranHawk/RT-Thread-485toCAN](https://github.com/FranHawk/RT-Thread-485toCAN)
# 需求分析
 - PC上位机由串口通过485协议发送**电机转矩电流指令**到电机驱动板，电机驱动板对指令解码并通过CAN总线发送转矩控制指令
 - PC上位机由串口通过485协议发送**状态查询指令**到电机驱动板，电机驱动板对指令解码并通过CAN总线发送**状态查询指令**，电机通过CAN总线返回包括转矩电流和编码器位置在内的状态，由电机驱动板发送回上位机
 - 电机驱动板为电机供电，每个电机最大电流1.5A，则总共需提供最大12A的电流
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021060409532269.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70#pic_center)

# 协议分析
 - CAN是物理层与链路层协议，一般微控制器(MCU)均带有CAN链路层外设，需要在硬件上对CAN物理层的支持
 - 485是物理层协议，一般将UART作为链路层与微控制器(MCU)通讯，需在硬件上提供对485物理层的支持
# 硬件设计
## 硬件需求
 - 控制：微控制器
 - 通讯：485转UART芯片，CAN物理层芯片
 - 调试：SWD烧写接口，复位按键，晶振，调试串口，LED*2，按键*2
 - 电源：直流12V输入，提供3.3V，5V，12V供电。12V需满足电机最大电流要求。
 - 接口：CAN电机接口x8，485接口x2，12V电源接口

## 硬件选型
### 伺服电机
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604101117980.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)

采用光毓机电的**RMDL伺服电机**，RMD-L 伺服电机是一款高集成度的动力输出模组。集成高性能 FOC 驱动器，高功率密度无刷电机，高精度绝对位置编码器三大功能部件。突破传统分布式设计，使得终端产品结构设计更加简洁，产品内部走线更加便捷，整机性能更稳定。
### MCU
**STM32F103RET6**
由于开发时间比较短，我采用了我最常用的STM32F103RET6，它拥有一个CAN控制器，64kRAM，256kROM，满足需求，实际上RAM用不到这么大，毕竟现在芯片这么贵，其实完全可以采用相同功能但是RAM更小更便宜的芯片
### 485
**SP3485**
参照了正点原子战舰开发板上的设计
### CAN
**TJA1050**
参照了正点原子战舰开发板上的设计,5V供电
### 12V供电
采用程控电源直接给电机驱动板提供12V供电，但是要保证电机驱动板可以通过足够大的电流，预计在PCB上采用开窗的方式加大电流承载量。
### 5V,3.3V供电
- **TPS5430**开关电源将12V降至7.3V保证供电效率
- **AMS1117-5.0**将7.3V降至5V
- **AMS1117-3.3**将5V降至3.3V

## 硬件整体结构
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604113617491.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70#pic_center)
采用了两个SP3485芯片，其实一片就够用了，为了提高硬件的可扩展性，又另外加了一片
## 硬件设计流程
采用免费的EDA软件**立创EDA**，可以直接从立创商城导入封装，同时可以采用浏览器编辑器，全部云端化处理，免去下载软件的麻烦。
完整硬件工程链接如下
[https://oshwhub.com/FranHawk/485tocan_motor_controller](https://oshwhub.com/FranHawk/485tocan_motor_controller)
接下来展示各个硬件部分的原理图。
### STM32主控部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604114546562.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604114610832.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### STM32调试部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604114653723.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### 7.3V开关电源部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604114744386.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604115400387.png)
 - 我参照了数据手册的参考设计，数据手册可以在立创商城上找。
 - 其中R1，R2的计算公式如上，R1为10k，R2为2k，计算得到输出电压为7.3V。
 - 这里设计的时候我设计出现了一个问题，板子到了后发现7.3V电压无法正常输出，通过查看数据手册发现ENA浮空就好了。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604115615139.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### 5V电源部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604132526989.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### 3.3V电源部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604132603237.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### 485电路部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604132744121.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021060413280728.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### CAN电路部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604132852650.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### 12V及CAN接口电路部分
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604133050957.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604133121655.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)
### PCB
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604133322708.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70)

## 硬件制作流程
采用嘉立创的PCB制版和SMT贴片，贴片的时候的BOM表和有极性的芯片贴片方向一定要仔细调整。从投板，贴片，到收到货总共花费5天时间。最终成品图如下。
### 正面图
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021060413351316.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70#pic_center)
### 背面图
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210604133536599.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjQ4NzkwNg==,size_16,color_FFFFFF,t_70#pic_center)









