multirotor_emergency_landing
==============================
Multirotor control allocation and emergency landing
-----------------------------------------------------
  >본 연구에서는 컨벡스 최적화 기법을 이용하여 다중프로펠러 비행체의 고장 대응 알고리듬을 설계한다.
 
## 개요
고장대응 알고리듬은 비행체의 고장 검출이 완료되었다는 가정하에 설계되었으며, 고장 검출 직후 실행된다.
알고리듬은 비행체 구동을 위한 roll, pitch, yaw 자세각 명령과 추력 명령을 출력하여 비행체가 고장 위치에서 착륙 가능 조건을 만족하는 가장 가까운 위치로 최소 추력을 사용하여 착륙하도록 한다.
알고리듬에는 다음의 식을 컨벡스화한 식이 사용되었다.

<img src = "https://user-images.githubusercontent.com/70250834/99491238-bacd5800-29ae-11eb-81f3-6c9ea2601a35.png"  width="60%" height="60%">


추력 크기의 차이를 줄이기 위하여 아래 식 1)를 제한조건으로 사용하였으며, 비행체의 자세를 적당히 유지하기 위하여 식 2)를 적용하였다. 또한, 착륙이 가능한 위치를 제한하기 위해 식 3)을 추가하였다.

1. <img src="https://user-images.githubusercontent.com/70250834/99487289-4cd26200-29a9-11eb-8d37-3ae403aaa1f2.png" width="20%" height="10%">

2. <img src = "https://user-images.githubusercontent.com/70250834/99487049-d7669180-29a8-11eb-9637-4005c1341631.png" width="40%" height="30%">

3. <img src =  "https://user-images.githubusercontent.com/70250834/99487470-c66a5000-29a9-11eb-9b0b-5bb0b1f65191.png"   width="14%" height="7%">

## 비상착륙 알고리듬
1. 고장 검출이 끝난 시점에서 비행체의 위치와 속도를 초기값으로 한다.
2. Bisection Method를 이용하여 최적 N을 정한다.
3. 컨벡스 문제를 N번 풀어 3축 추력 명령을 얻는다.
4. 가장 첫 번째 step의 추력 명령을 자세각(roll, pitch, yaw)명령과 추력 명령으로 변환한다.
5. 4에서 계산한 값을 비행체의 자세 명령으로 반환한다.

## 시뮬레이션 결과
1. simulation 1
* 총 시뮬레이션 시간은 10초이며 고장 발생 시간은 3초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* mission

| |0~3.5s|
|--|--|
|x| 3 |
|y| 1 |
|Altitude| 2 | 
|yaw| 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![sim1_thrust](https://user-images.githubusercontent.com/70250834/99490812-edc31c00-29ad-11eb-8386-d809ed6f1005.png)
![sim1_Euler](https://user-images.githubusercontent.com/70250834/99490826-f287d000-29ad-11eb-9f9c-41b98efddacf.png)
![sim1_rotor](https://user-images.githubusercontent.com/70250834/99490841-fb78a180-29ad-11eb-91b4-cda7b942680f.png)
![sim1_NED](https://user-images.githubusercontent.com/70250834/99490846-fd426500-29ad-11eb-9b07-a2e84da06fbb.png)
![sim1_3D](https://user-images.githubusercontent.com/70250834/99490858-029faf80-29ae-11eb-8b0f-22e12a4e600a.png)
![AnyConv com__Multirotor_lin_sim2](https://user-images.githubusercontent.com/70250834/100186481-73544800-2f29-11eb-9157-c806d0f3188a.png)

2. simulation 2
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* mission

| |0~3.5s|0~6.2s|
|--|--|--|
|x| 3 | 5 |
|y| 2 | 6 |
|Altitude| 4 | 4 |
|yaw| 10 | 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![sim2_thrust](https://user-images.githubusercontent.com/70250834/99490861-05020980-29ae-11eb-83e5-9aaa9e8a5326.png)
![sim2_Euler](https://user-images.githubusercontent.com/70250834/99490866-07fcfa00-29ae-11eb-91d9-29856e938fd9.png)
![sim2_rotor](https://user-images.githubusercontent.com/70250834/99490998-4db9c280-29ae-11eb-940d-4d19f6257f8f.png)
![sim2_NED](https://user-images.githubusercontent.com/70250834/99490882-0df2db00-29ae-11eb-8b39-152253b34f5a.png)
![sim2_3D](https://user-images.githubusercontent.com/70250834/99490888-10553500-29ae-11eb-9b1f-4809b2193a52.png)
![AnyConv com__Multirotor_lin_sim1](https://user-images.githubusercontent.com/70250834/100186485-73ecde80-2f29-11eb-95ef-8a83c715af0a.png)

