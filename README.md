# :taxi: 무인 자율주행 택시 알파카

![image](https://user-images.githubusercontent.com/89068148/196036636-99cb424a-fd81-4d8c-aec3-f246a2559992.png)

<br>

```
모빌리티(자율주행) 도메인을 통해, 자율주행 시스템을 구현하고, 
해당 시스템을 택시 서비스에 접목한 프로젝트 입니다.

이번 프로젝트의 핵심은 자율주행 도메인에 대한 이해와 
인지, 판단, 제어 구현 입니다.

무인 자율주행 택시 알파카는, 해당 자율주행 기술의 발전으로
미래에 있을법한 서비스의 예시 입니다.
```

역할 : 제어 및 판단 / 전체 단계 통합

기술 : python, ROS(우분투), m사 시뮬레이터

<br>

<br>

## :computer: 핵심 구현 기능

<br>

### 1. 터치 한번으로 새로운 주행 시작 !

모바일에서 터치 한번으로 시뮬레이터의 차량이 새로운 목적지로 주행한다.

명세서는 한 번 작동을 시키면 단 한번의 주행만을 진행한다. 

하지만 모바일의 택시 서비스를 진행하기 위해서는 한 번의 작동으로 여러 번 주행할 필요가 있었다. 이 때문에 세부적인 구조는 

유지가 되었지만, 전체적인 흐름은 크게 바뀌었다.

특히, 파이어 베이스를 통해 모바일과 시뮬레이터 간에 명령을 주고 받는다.

그럼 시뮬레이터가 한번 실행되고 나선, 모바일을 통해 주행하고 정차하고 다시 주행하는 모습이 가능하다.

[코드 및 주석](https://github.com/windy825/Alpah_car/blob/master/catkin_ws/src/alpha_car/scripts/firebase.py)

 <br>

### 2. 경로 판단을 위해, 차선 변경 정보 반영

기존의 json 파일을 수정하여 다익스트라 비용 계산시 차선 변경 요소를 고려하여 판단한다.

예시를 보면 다익스트라 비용 테이블 초기화시, 출발노드의 앞 링크에서 차선변경이 가능하면 

출발노드의 갈 수 있는 노드 리스트에 새 노드가 추가된다.

다만 비용을 단순하게 지금 직진보다 조금 더 주는 방식이기 때문에 정교 하지 않다. 개선이 필요

```python
# init : possible left changeable node, cost
for x, left_link in enumerate(next_link.get_all_left_links(), 1):
	weight_from_this_node[left_link.to_node.idx] = [left_link.idx, len(left_link.points) + 20*x]
        # 이 노드에서 갈 수 있는 노드 리스트         =   [링크 IDX, 직진보다 차선변경에 좀더 가중치를 둔 비용]

# init : possible right changeable node, cost
for x, right_link in enumerate(next_link.get_all_right_links(), 1):
	weight_from_this_node[right_link.to_node.idx] = [right_link.idx, len(right_link.points) + 20*x] 
```

<br>

### 3. 기타, 명세서 상의 gpsimu, acc, advanced_pure_pursuit, pid 등등 모두 구현

늘 그렇지만,,,, 코드는 항상 작성 및 진행이 된다.

하지만 이 VM환경에선 도저히 돌아가지 않는다.. 절망적이었지만, 

다행히 프로젝트 중반에 팀원과 같이 노트북 1곳에서 돌리던 환경을 노트북 2개로 분리하여 시연하게 되었다.

![제목 없음](https://user-images.githubusercontent.com/89068148/197391006-66aa4700-d234-4ac0-873c-ddd65bc99cf4.png)

<br>

<br>

## :gun: 서비스 전체 흐름

<br>

**[1] 자율주행 도메인**

![image](https://user-images.githubusercontent.com/89068148/197387685-6b5886ed-cfb6-4c6a-9b2c-475f886ade7f.png)

<br>

**[2] 안드로이드 도메인**

![image](https://user-images.githubusercontent.com/89068148/197387763-45c6720f-34b7-4168-b5e9-76a6696b159f.png)

<br>

**[3] 전체 흐름**

![image](https://user-images.githubusercontent.com/89068148/197387800-567f5e51-8613-486b-b9a4-2cc1f29998f6.png)

