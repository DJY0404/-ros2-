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
![CannyEdgeNode](https://github.com/DJY0404/-ros2-/assets/55430286/d55d73b0-426b-4191-88a6-800037d946a3) <br/>

* StillShotClient
```
ros2 run camera StillShotClient
```
![CaptureSuceess](https://github.com/DJY0404/-ros2-/assets/55430286/36db822a-1552-4584-bcbc-9e7b4159a7b3) <br/>
(Home/capture/) <br/>
![CaptureInFolder](https://github.com/DJY0404/-ros2-/assets/55430286/16f58913-f356-4f0d-90b6-13511e4fb4fb) <br/>

* TestClientNode (service call 테스트 용 노드)<br/>

```
# msgspack/srv/Testsrv
# Request t1: int64
# Response t2: int64
ros2 service call /test_node msgspack/srv/Testsrv "{t1: 3}"
```
![TestNodeCall](https://github.com/DJY0404/-ros2-/assets/55430286/802e9d70-405a-45bb-87ff-0d543b4dc083) <br/>
![TestNodeResponse](https://github.com/DJY0404/-ros2-/assets/55430286/f75642ec-9656-4e78-98f9-41a475544283) <br/>

