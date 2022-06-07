# Vehicle Control Using ROS and CAN

## 필요 라이브러리
rospy\
numpy\
attrdict\
can\
cantools\
pygame\
geopy\
novatel_oem7_msgs.msg -> novatel_oem7_driver

## 설치 해야 할것
ROS melodic\
python3\
novatel_oem7_driver\
can-utils

## 실행 전
1. dbc 파일을 작성한다.
2. scripts/can_module.py에서 dbc에 맞추어 메세지 및 시그널을 조정한다.
3. path를 설정하거나, scripts/final_can_control.py에서 시작점 및 끝점을 조정한다(직선).

```python
class Controller:
    def __init__(self, conf=None, param=None):
        
        """ 시작점, 끝점 """
        self.start_pos = (35.22510137, 126.83975056)
        self.target_pos = (35.22545213, 126.83870505)
        
        """ path init """
        # self.init_path()
        
    """ load path or get update from other module """
    def init_path(self):
        pass
```
4. GPS를 RTK모드로 변경한다. 
## 실행
0. ros를 실행한다  ```roscore```
1. car_alive.py를 실행한다 ```python3 scripts/car_alive.py```
2. GPS 드라이버 실행 ```roslaunch novatel_oem7_driver oem7_net.launch```
4. 제어 프로그램 실행 ```rosrun can_control final_can_control```





