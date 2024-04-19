# -세계 쵝오의 ros2 카메라-

### 세계 쵝오의 ROS2 카메라 만들기<br/>
:heavy_exclamation_mark:학습 및 이해 수준 판단을 위한 프로젝트입니다. (수정 및 추가 구현 할 예정! launch, parameter, 여러 기능들...):heavy_exclamation_mark:<br/><br/>
[![My Skills](https://skillicons.dev/icons?i=ubuntu,vscode,cpp,ros&perline=4)](https://skillicons.dev)
<br/>
### 1. 구성 <br/>
```
├── camera
│   ├── CMakeLists.txt
│   ├── include
│   │   └── camera
│   ├── launch
│   │   └── CannyEdgeNode.launch.py
│   ├── package.xml
│   ├── param
│   │   ├── filter.yaml
│   │   ├── path.yaml
│   │   └── teststring.yaml
│   └── src
│       ├── CannyEdgeNode.cpp
│       ├── ImageServiceServer.cpp
│       ├── StillShotClient.cpp
│       ├── TestClientNode.cpp
│       └── WebcamPub.cpp
└── msgspack
    ├── action
    ├── CMakeLists.txt
    ├── include
    │   └── msgspack
    ├── msg
    ├── package.xml
    ├── src
    └── srv
        ├── Camerashot.srv
        └── Testsrv.srv
```
### 2.기본 세팅 <br/>
:heavy_exclamation_mark: ros2 launch로 실행 방식 변경 예정
<br/>


* Terminal 1:<br/>

```
ros2 run camera WebcamPub
```
* Terminal 2:<br/>

```
ros2 run camera ImageServiceServer
```
* Terminal 3:<br/>

```
rqt
```
* Terminal 4,5...:<br/>
```
ros2 run camera [...]
```
### 3.기능 사용 예시 <br/>
* CannyEdgeNode: 
