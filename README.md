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
* 착륙 위치 제한은 위의 식 3)과 같다.
* mission

| |0~3.5s|
|--|--|
|x| 3 |
|y| 1 |
|Altitude| 2 | 
|yaw| 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![sim1_thrust](https://user-images.githubusercontent.com/70250834/100834114-a5782380-34ae-11eb-89f7-482069d514e5.png)
![sim1_Euler](https://user-images.githubusercontent.com/70250834/100834115-a610ba00-34ae-11eb-81e3-4acf04701ee4.png)
![sim1_rotor](https://user-images.githubusercontent.com/70250834/100834116-a610ba00-34ae-11eb-8a14-29b905910206.png)
![sim1_NED](https://user-images.githubusercontent.com/70250834/100834117-a6a95080-34ae-11eb-84e2-d826638b052c.png)
![sim1_angle](https://user-images.githubusercontent.com/70250834/100834118-a741e700-34ae-11eb-9256-defdc5d3ffe3.png)
![sim1_2D](https://user-images.githubusercontent.com/70250834/100834120-a7da7d80-34ae-11eb-8e9e-98e1fad4e11a.png)
![sim1_3D](https://user-images.githubusercontent.com/70250834/100834119-a741e700-34ae-11eb-9127-0a03f815e18c.png)

![Multirotor_lin_sim2](https://user-images.githubusercontent.com/70250834/100188979-bbc23480-2f2e-11eb-902f-f4350447f637.gif)

2. simulation 2
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 위의 식 3)과 같다.
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

![Multirotor_lin_sim1](https://user-images.githubusercontent.com/70250834/100188977-ba910780-2f2e-11eb-8df9-9f6dfc970f66.gif)

3. simulation 3
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = "https://user-images.githubusercontent.com/70250834/100188059-c4b20680-2f2c-11eb-95b6-4b481a7c2480.png" width="14%" height="7%">

* mission

| |0~3.5s|0~6.2s|
|--|--|--|
|x| 3 | 5 |
|y| 2 | 6 |
|Altitude| 4 | 4 |
|yaw| 10 | 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![P1_Thrust](https://user-images.githubusercontent.com/70250834/100186549-91ba4380-2f29-11eb-9269-449a702f1993.png)
![p1_Euler](https://user-images.githubusercontent.com/70250834/100186536-8ebf5300-2f29-11eb-8f74-42effed2781a.png)
![p1_Rotor](https://user-images.githubusercontent.com/70250834/100186539-8f57e980-2f29-11eb-8180-21b06203d807.png)
![p1_NED](https://user-images.githubusercontent.com/70250834/100186537-8ebf5300-2f29-11eb-955a-3bbe103ee261.png)
![p1_3D](https://user-images.githubusercontent.com/70250834/100186530-8d8e2600-2f29-11eb-84be-8bc102133211.png)

![Multirotor1](https://user-images.githubusercontent.com/70250834/100188844-6c7c0400-2f2e-11eb-8cab-5448e5a779e3.gif)

4. simulation 4
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188060-c54a9d00-2f2c-11eb-93fe-008e9d6bc158.png width="14%" height="7%">

* mission

| |0~3.5s|0~6.2s|
|--|--|--|
|x| 3 | 5 |
|y| 2 | 6 |
|Altitude| 4 | 4 |
|yaw| 10 | 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![p2_thrust](https://user-images.githubusercontent.com/70250834/100186547-91ba4380-2f29-11eb-9f78-0d5d9d87fbe6.png)
![p2_Euler](https://user-images.githubusercontent.com/70250834/100186543-90891680-2f29-11eb-80ee-dc02c0d9540e.png)
![p2_rotor](https://user-images.githubusercontent.com/70250834/100186546-9121ad00-2f29-11eb-98c0-10b5776e559e.png)
![p2_NED](https://user-images.githubusercontent.com/70250834/100186545-90891680-2f29-11eb-9c02-41604ffea594.png)
![p2_3D](https://user-images.githubusercontent.com/70250834/100186542-8ff08000-2f29-11eb-95fb-03c4e230933d.png)

![Multirotor2](https://user-images.githubusercontent.com/70250834/100189023-cf6d9b00-2f2e-11eb-812d-c262d2dbdf6b.gif)

5. simulation 5
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188061-c5e33380-2f2c-11eb-9324-d7dba8e8a3b9.png width="18%" height="9%">

* mission

| |0~3.5s|0~6.2s|
|--|--|--|
|x| 3 | 5 |
|y| 2 | 6 |
|Altitude| 4 | 4 |
|yaw| 10 | 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![u1_thrust](https://user-images.githubusercontent.com/70250834/100186555-93840700-2f29-11eb-8dd9-5b98a30dc546.png)
![u1_Euler](https://user-images.githubusercontent.com/70250834/100186551-9252da00-2f29-11eb-9ad4-f8c73bbc80ed.png)
![u1_rotor](https://user-images.githubusercontent.com/70250834/100186554-93840700-2f29-11eb-9bda-2c1c8471a347.png)
![u1_NED](https://user-images.githubusercontent.com/70250834/100186552-92eb7080-2f29-11eb-9b70-3e579987050f.png)
![u1_3D](https://user-images.githubusercontent.com/70250834/100186550-9252da00-2f29-11eb-9ed8-dd8b1860c324.png)

![Multirotor3](https://user-images.githubusercontent.com/70250834/100189022-ce3c6e00-2f2e-11eb-8353-92fbcc015cce.gif)

6. simulation 6
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188063-c67bca00-2f2c-11eb-9e2b-1b6a8386be8b.png width="18%" height="9%">

* mission

| |0~3.5s|0~6.2s|
|--|--|--|
|x| 3 | 5 |
|y| 2 | 6 |
|Altitude| 4 | 4 |
|yaw| 10 | 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![u2_thrust](https://user-images.githubusercontent.com/70250834/100186565-95e66100-2f29-11eb-990a-f9e8b3e1fc4c.png)
![u2_Euler](https://user-images.githubusercontent.com/70250834/100186559-94b53400-2f29-11eb-8937-973278615499.png)
![u2_rotor](https://user-images.githubusercontent.com/70250834/100186564-954dca80-2f29-11eb-8033-41d60fa5741e.png)
![u2_NED](https://user-images.githubusercontent.com/70250834/100186561-94b53400-2f29-11eb-9802-b70fa17ad524.png)
![u2_3D](https://user-images.githubusercontent.com/70250834/100186557-941c9d80-2f29-11eb-9425-61ba5b933444.png)

![Multirotor4](https://user-images.githubusercontent.com/70250834/100189062-e0b6a780-2f2e-11eb-97d9-09329506218c.gif)

7. simulation 7
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188064-c67bca00-2f2c-11eb-8f49-3351ea68d565.png width="14%" height="7%">

* mission

| |0~3.5s|0~6.2s|
|--|--|--|
|x| 3 | 5 |
|y| 2 | 6 |
|Altitude| 4 | 4 |
|yaw| 10 | 10 |

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![u3_thrust](https://user-images.githubusercontent.com/70250834/100186575-97b02480-2f29-11eb-8615-c0ad861aefdc.png)
![u3_Euler](https://user-images.githubusercontent.com/70250834/100186571-967ef780-2f29-11eb-9389-452adae1c66f.png)
![u3_rotor](https://user-images.githubusercontent.com/70250834/100187459-7d774600-2f2b-11eb-97a2-03ba71da8351.png)
![u3_NED](https://user-images.githubusercontent.com/70250834/100186572-97178e00-2f29-11eb-8c10-2e262242167c.png)
![u3_3D](https://user-images.githubusercontent.com/70250834/100186566-95e66100-2f29-11eb-8803-0de997915f2a.png)

![Multirotor5](https://user-images.githubusercontent.com/70250834/100189058-deece400-2f2e-11eb-8575-b3daca77d1db.gif)
