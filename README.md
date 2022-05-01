#网络遥控器开源项目
## 项目介绍
本项目使用ESP8266开发的4G原成遥控车改装套件， 通过接受遥控器原厂接收机的PWM信号解码为各通道的数字信号，使用MQTT通讯以发布订阅的形式将数据发布到MQTT服务器，就受端订阅发射端ID后，将数字信号还原为舵机使用的PWM信号以提供遥控车使用。
## 发射端使用说明
### 硬件连接
网络遥控器发射端
![1(1)](https://user-images.githubusercontent.com/63451966/166128316-c81e6f5b-4071-46fe-ae6f-bdcacde18795.png)
使用5V2A以上的TYPEC供电 或者 使用电池供电 电压在8-24V 
两种供电方式二选一即可

遥控器接收机按照顺序 1 到 5顺序接上即可（注意不要插反）
![1](https://user-images.githubusercontent.com/63451966/166128323-008add55-cdd8-43f8-bfb1-e0b45f2f1a77.png)
### 配网
1.设备上电 用电脑连接 NetRc_Tx 的 WIFI
2.打开网页输入 192.168.4.1 访问配网界面
![3](https://user-images.githubusercontent.com/63451966/166128331-43996b64-3da9-4c11-b3fb-c35509c98a14.png)
3.输入要连接的wifi和对应的密码 (只能连接2.4G wifi)
4.点击确定，完成配网
5.记录设备设备MAC，此ID需要在接收端中填写进行配对
### 设备状态

| LED状态 | 含义 | 备注 |
| --- | --- | --- |
| 长亮 | 未连接网络 |  |
| 慢闪 | 未接入接收机信号 |  |
| 熄灭 | 正常工作 |  |
-------------------------------------
# NetRc相关开发说明
基础工程仓库地址： https://gitee.com/xuhongv/AiThinkerProjectForESP.git

使用UBUNTU1804虚拟机搭建开发环境具体可参考安信可官方文档 LINUX下SDK开发环境搭建 本方案基于ESP8266的RTOS SDK3.0版本开发而来,在xClouds开源架构下进行二次开发，下边时关于xClouds 开源架构的介绍

# 关于  xClouds 开源架构

&nbsp;&nbsp;&nbsp;&nbsp; 当我们日复一日年复一年的搬砖的时候，你是否曾想过提升一下开发效率，如果一个  **IOT智能家具物联网**  模板的项目摆在你的面前，你还会选择自己搭架构么？

 &nbsp;&nbsp;&nbsp;&nbsp; 但是搭建出一个好的架构并非易事，有多少人愿意选择去做，还有多少人选择努力去做好，可能寥寥无几，但是你今天看到的，正是你所想要的，一个真正能解决你开发新项目时最大痛点的架构工程，你不需要再麻木 ``Copy`` 原有旧项目的代码，只需改动少量代码就能得到想要的效果，你会发现开发新项目其实是一件很快乐的事；

&nbsp;&nbsp;&nbsp;&nbsp; 很幸运的是，在2018年底时候，我已经自学打通了天猫精灵音箱和小爱同学音箱同时控制一个 ESP8266，那时候我就开始专研服务器开发，自学攻破了数据库、php语言，再到了微信公众号的生态，我打通了这一整个的控制；

&nbsp;&nbsp;&nbsp;&nbsp;  直到目前为止，我依然在走这条道路，但是随着开源其趋势不断影响，很多组织或个人奋力地做一些开源，源源不断的架构和代码封装，加快了我们程序员开发的效率，比如前端的三大框架的 **Angular**，**React**，**Vue**：


&nbsp;&nbsp;&nbsp;&nbsp;  国内AI智能音箱一大崛起，也有很多优秀的企业一步一步崛起，像涂鸦就是典型的例子，一站式开发，对于很多小型家电传统公司接入AI音箱等智能控制，无疑就是一个 “菩萨” 般存在；

&nbsp;&nbsp;&nbsp;&nbsp;  话说回来，类似这种平台，对于我们开发者有什么值得学习的地方呢？ 最熟悉宣传语就是：**全球部署，一接入可对接多语音平台。**

&nbsp;&nbsp;&nbsp;&nbsp;  于是乎，我有个大胆的想法能否单独做这样的架构出来并开源出来：

- **跨平台、开发快、全开源**的宗旨；
- 微信公众号内 `airkiss` 配网和 `MQTT` 控制；
- 微信小程序内 `smartConfig` 配网和 `MQTT` 控制；
- 设备端要求：支持 `airkiss` 配网和 `MQTT` 协议即可；
- 服务器端：有微信公众号业务和各云平台对接的业务，支持但不限于 天猫精灵、小爱同学、小度音箱、Alexa音箱等；
- 扩展性强，支持用户二次开发，可私定义协议；
- 三端开源：设备端、服务器端、前端；

&nbsp;&nbsp;&nbsp;&nbsp;  现在，我有能力地公开这个架构，我想对于那些想学系统化的，或者电子diy爱好者，抑或是在校大学生，可参考学习，我的愿景就是：

&nbsp;&nbsp;&nbsp;&nbsp; **xClouds能在国内物联网一枝独秀，助国内物联网发展；**

&nbsp;&nbsp;&nbsp;&nbsp; **但愿人长久，搬砖不再有！**



# xClouds-devices for  ESP8266  开发框架


&nbsp;&nbsp;&nbsp;&nbsp;  认识我的人都知道，我和乐鑫 ESP8266 芯片有一个解不开的缘分，我通过此芯片认识很多志同道合的的小伙伴，也让我逐渐地找到了自己的人生价值，所以，我会以自己擅长的 Wi-Fi模块二次开发，带领大家快速体验和商业化使用此物美价廉的芯片；


&nbsp;&nbsp;&nbsp;&nbsp;  本框架基于**ESP8266_RTOS_SDK v3.0** 上二次开发，采用C语言开发，并非 **arduino** 开发；

## 例程简介

* [1.SmartConfig_AirKiss_To_MQTT](./Ai-examples/1.SmartConfig_AirKiss_To_MQTT) : 简单模板，实现 smartconfig 一键配网 和 微信公众号 airkiss 配网，并实现连接 MQTT 服务器的通讯实现上下发数据的例子；

---

## 开发入门

### 环境搭建

&nbsp;&nbsp;&nbsp;&nbsp;  因为此为基于 RTOS 系统上开发，需要一定的C基础或 freeRtos 开发基础，强烈建议使用 Linux 开发；

* [Windows](https://aithinker.blog.csdn.net/article/details/104390508)
* [Linux](https://xuhong.blog.csdn.net/article/details/104736261)

&nbsp;&nbsp;&nbsp;&nbsp;然后跑起 demo， 比如 [1.SmartConfig_AirKiss_To_MQTT](./Ai-examples/1.SmartConfig_AirKiss_To_MQTT)

```
cd /Ai-examples/1.SmartConfig_AirKiss_To_MQTT
make menuconfig
make -j8 flash 
make monitor
```

# 感谢：

- PHP微信对接：https://github.com/zoujingli/WeChatDeveloper
- PHP Oauth2.0：https://github.com/bshaffer/oauth2-server-php
- PHP 框架：http://www.thinkphp.cn
- 乐鑫物联网操作系统：https://github.com/espressif/esp-idf

项目遵循协议： [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0.html)

服务器开源地址：https://github.com/xuhongv/xClouds-php

设备端开源地址：https://github.com/xuhongv/xClouds-device

**1、额外说明，架构中提到的对公司或组织的观点，如有争议，请联系我；**

**2、架构中涉及到的技术点，我会一一公布出来以表感谢；**

**3、同时，也欢迎大家支持我，或一起壮大这个框架，奉献你代码项目；**
