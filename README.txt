项目名称：基于机械臂的笔迹模拟

项目说明：本项目借助 https://github.com/dailenson/SDT 进行对于笔迹的深度学习，使用数
张（约20张）笔迹照片即能生成众多（6763个）汉字的轨迹。本项目基于这些轨迹，完成摄像头视觉
引导（目前主要功能仅为检测纸张边界）下的机械臂运动，进行汉字书写。

文件说明：
- user_generate_new.py 配合https://github.com/dailenson/SDT使用的文件
- results_one 笔迹数据库
- example.txt 待书写文本
- calibrate.py 标定摄像头，获取内参矩阵
- calibate_images 用于存储标定所用照片
- patter.pdf 标定所需棋盘格
- initialize.py 初始化机械臂，完成坐标系设定
- c.jpg 初始化机械臂完成后拍摄的照片，用于用户检验摄像头能否完整拍摄整个纸张
- move_x.py,move_y.py 控制机械臂分别向x坐标轴正方向，y坐标轴正方向运动，用户需基于此与
c.jpg判断设定的坐标轴与摄像头视场的坐标轴是否一致
- main.py 主程序，包含语音识别，纸张边缘检测，坐标系变幻及控制机械臂移动进行书写
- audio.mp3 语音识别所用提示音
- 20250425134342A024 JAKA提供的SDK，含多系统版本
- jakaAPI.dll, jaksAPI.exp, jakaAPI.lib, jkrc.exp, jkrc.lib, jkrc.pyd 在64位Windows
电脑上运行所需SDK

运行方式：
1.按requirements_SDT.txt配置环境
2.从 https://github.com/dailenson/SDT 下载项目文件，得到文件夹SDT-master，将此文件夹
下的user_generate.py换为user_generate_new.py，依照user_generate.py的运行方式（参阅
SDT项目文档）运行user_generate_new.py，得到results_one，将results_one复制至本项目根目录下。
3.按requirements.txt配置环境
4.使用摄像头拍摄打印的pattern.pdf的各角度多张图片,存入calibrate_images，运行calibrate.py,
依照所得矩阵修改main.py中writing()中fx fy cx cy数据，后将摄像头固定至机械臂上
5.在桌面上固定一张白纸（受限于机械臂最大抬升高度，本程序中将拍照高度固定为200mm，此高度
较低，故为使白纸能被拍全，此纸尺寸应较小）
6.手动移动机械臂，直至固定在机械臂上的摄像头紧贴白纸
7.运行initialize.py，机械臂抬升200mm，新建坐标系并拍照
8.运行move_x.py move_y.py观察行进方向，以此确定新建坐标系的坐标轴朝向
9.打开c.jpg，确保纸张被拍全且照片坐标系（从照片左上角到右上角为x轴正方向，从左上角到左下
角为y轴正方向）与新建坐标系重合，否则回到第6步
10.运行main.py，根据提示音指定数据（需取消main.py中第305行的注释），固定笔在机械臂上，开始书写

！注意：也可略去1-2步直接用本项目自带的results_one中数据书写
