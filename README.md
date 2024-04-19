# -세계 쵝오의 ros2 카메라-

### 세계 쵝오의 ROS2 카메라 만들기<br/>
:heavy_exclamation_mark:(수정 및 추가 구현 할 예정! launch, parameter, 여러 기능들...):heavy_exclamation_mark:<br/><br/>
[![My Skills](https://skillicons.dev/icons?i=ubuntu,vscode,cpp,ros&perline=4)](https://skillicons.dev)
<br/>
1. 구성 <br/>
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
...
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
