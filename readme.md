# sztu_hand ros 仓库
## 文件架构
'''
├── DynamixelSDK //舵机sdk
└── sztu_hand
    ├── CMakeLists.txt
    ├── hand_urdf
    ├── launch
    ├── package.xml
    ├── rviz
    └── src
'''
## 编译
由于sztu_hand 功能包依赖sdk,故需要先编译sdk。
## 目前实现功能
rviz 实时控制🖕
## 如何启动
roslaunch sztu_hand display.launch 
