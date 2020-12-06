multirotor_emergency_landing
==============================
Multirotor control allocation and emergency landing
-----------------------------------------------------
  >본 연구에서는 컨벡스 최적화 기법을 이용하여 다중프로펠러 비행체의 고장 대응 알고리듬을 설계한다.
 
## 개요
고장대응 알고리듬은 비행체의 고장 검출이 완료되었다는 가정하에 설계되었으며, 고장 검출 직후 실행된다.
알고리듬은 비행체 구동을 위한 roll, pitch, yaw 자세각 명령과 추력 명령을 출력하여 비행체가 고장 위치에서 착륙 가능 조건을 만족하는 가장 가까운 위치로 최소 추력을 사용하여 착륙하도록 한다.
알고리듬에는 다음의 식이 사용되었다.

<img src = "https://user-images.githubusercontent.com/70250834/101276230-aed6f800-37ee-11eb-9850-f8a17c04a0e7.png"  width="60%" height="60%">


추력 크기의 차이를 줄이기 위하여 위의 식 (7)을 제한조건으로 사용하였으며, 비행체의 자세를 적당히 유지하여 급기동을 방지하고자 식 (8)을 적용하였다. 


## 비상착륙 알고리듬
1. 고장 검출이 끝난 시점에서 비행체의 위치와 속도를 초기값으로 한다.
2. Bisection Method를 이용하여 최적 N을 정한다.
3. 컨벡스 문제를 N번 풀어 3축 추력 명령을 얻는다.
4. 가장 첫 번째 step의 추력 명령을 자세각(roll, pitch, yaw)명령과 추력 명령으로 변환한다.
5. 4에서 계산한 값을 비행체의 자세 명령으로 반환한다.

## 시뮬레이션 결과
1. simulation 1
* 총 시뮬레이션 시간은 10초이며 고장 발생 시간은 3초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src =  "https://user-images.githubusercontent.com/70250834/99487470-c66a5000-29a9-11eb-9b0b-5bb0b1f65191.png"   width="14%" height="7%">

* Mission

|time(s)|Altitude(m)|x(m)|y(m)|yaw|
|--|--|--|--|--|
|0~3.5|2|3|1|10|

![sim1_2D](https://user-images.githubusercontent.com/70250834/101276182-6fa8a700-37ee-11eb-881a-73724e81351d.png)

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![sim1_thrust](https://user-images.githubusercontent.com/70250834/101276189-71726a80-37ee-11eb-8fa7-4b829a95798b.png)
![sim1_euler](https://user-images.githubusercontent.com/70250834/101276187-70d9d400-37ee-11eb-8155-42b849338272.png)
![sim1_rotor](https://user-images.githubusercontent.com/70250834/101276188-71726a80-37ee-11eb-9060-22f3c29db74b.png)
![sim1_body](https://user-images.githubusercontent.com/70250834/101276185-70d9d400-37ee-11eb-94b6-e0904bec0b9f.png)
![sim1_3D](https://user-images.githubusercontent.com/70250834/101276183-6fa8a700-37ee-11eb-898f-68d6dd3dee99.png)

착륙시 기체가 급기동하는 것을 방지하기 위하여 착륙 직전의 일부 스텝의 추력 명령이 Z축과 이루는 각을 6도로 제한하였다.
![sim1_angle](https://user-images.githubusercontent.com/70250834/101276184-70413d80-37ee-11eb-8d69-cef27dcd42e2.png)

![Multirotor_lin_sim1](https://user-images.githubusercontent.com/70250834/101276212-84853a80-37ee-11eb-8d97-bacb8228967f.gif)

2. simulation 2
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 위의 식 3)과 같다.
* Mission
|time(s)|Altitude(m)|x(m)|y(m)|yaw|
|--|--|--|--|--|
|0~3.5|4|3|2|10|
|3.5~6.2|4|5|6|10|

![sim2_2D](https://user-images.githubusercontent.com/70250834/101276190-720b0100-37ee-11eb-9351-b4f466fc7d1e.png)

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![sim2_Thrust](https://user-images.githubusercontent.com/70250834/100834087-9ee9ac00-34ae-11eb-8bdd-5c308fb826ea.png)
![sim2_euler](https://user-images.githubusercontent.com/70250834/101276139-5ef83100-37ee-11eb-8599-47626b92aff2.png)
![sim2_rotor](https://user-images.githubusercontent.com/70250834/101276142-60295e00-37ee-11eb-8391-1f0d01d38654.png)
![sim2_NED](https://user-images.githubusercontent.com/70250834/101276140-5f90c780-37ee-11eb-8311-8b0135f0421d.png)
![sim2_3D](https://user-images.githubusercontent.com/70250834/101276191-72a39780-37ee-11eb-9fa3-a70bbc4dc444.png)

착륙시 기체가 급기동하는 것을 방지하기 위하여 착륙 직전의 일부 스텝의 추력 명령이 Z축과 이루는 각을 6도로 제한하였다.
![sim2_angle](https://user-images.githubusercontent.com/70250834/101276138-5d2e6d80-37ee-11eb-865e-1985b816593d.png)

![Multirotor_lin_sim2](https://user-images.githubusercontent.com/70250834/101276210-83540d80-37ee-11eb-9fd6-e5b517eae4ba.gif)

3. simulation 3
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = "https://user-images.githubusercontent.com/70250834/100188059-c4b20680-2f2c-11eb-95b6-4b481a7c2480.png" width="14%" height="7%">

* Mission
|time(s)|Altitude(m)|x(m)|y(m)|yaw|
|--|--|--|--|--|
|0~3.5|4|3|2|10|
|3.5~6.2|4|5|6|10|

![pol1_2D](https://user-images.githubusercontent.com/70250834/101276165-69b2c600-37ee-11eb-9780-10c24f7ab169.png)

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![pol1_thrust](https://user-images.githubusercontent.com/70250834/101276173-6c152000-37ee-11eb-84f4-8a2f13244845.png)
![pol1_euler](https://user-images.githubusercontent.com/70250834/101276170-6ae3f300-37ee-11eb-9102-3aad6022dc8a.png)
![pol1_rotor](https://user-images.githubusercontent.com/70250834/101276172-6b7c8980-37ee-11eb-80f5-b455eb4cb077.png)
![pol1_NED](https://user-images.githubusercontent.com/70250834/101276171-6b7c8980-37ee-11eb-8c3d-53f0345e8b0b.png)
![pol1_3D](https://user-images.githubusercontent.com/70250834/101276167-6a4b5c80-37ee-11eb-9058-cab67239a8ac.png)

착륙시 기체가 급기동하는 것을 방지하기 위하여 착륙 직전의 일부 스텝의 추력 명령이 Z축과 이루는 각을 6도로 제한하였다.
![pol1_angle](https://user-images.githubusercontent.com/70250834/101276169-6a4b5c80-37ee-11eb-8121-624f306a6942.png)

![Multirotor1 (1)](https://user-images.githubusercontent.com/70250834/101276216-85b66780-37ee-11eb-8d37-702dc7975472.gif)

4. simulation 4
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188060-c54a9d00-2f2c-11eb-93fe-008e9d6bc158.png width="14%" height="7%">

* Mission
|time(s)|Altitude(m)|x(m)|y(m)|yaw|
|--|--|--|--|--|
|0~3.5|4|3|2|10|
|3.5~6.2|4|5|6|10|

![pol2_2D](https://user-images.githubusercontent.com/70250834/101276175-6cadb680-37ee-11eb-8478-a8121431e6c5.png)

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![pol2_thrust](https://user-images.githubusercontent.com/70250834/101276181-6f101080-37ee-11eb-909e-323abaa70cf5.png)
![pol2_euler](https://user-images.githubusercontent.com/70250834/101276178-6ddee380-37ee-11eb-9d41-7565f674e110.png)
![pol2_rotor](https://user-images.githubusercontent.com/70250834/101276180-6e777a00-37ee-11eb-94c9-eb29b666be29.png)
![pol2_NED](https://user-images.githubusercontent.com/70250834/101276179-6ddee380-37ee-11eb-8789-4e83f3328796.png)
![pol2_3D](https://user-images.githubusercontent.com/70250834/101276176-6cadb680-37ee-11eb-8ddc-645491831ce8.png)

착륙시 기체가 급기동하는 것을 방지하기 위하여 착륙 직전의 일부 스텝의 추력 명령이 Z축과 이루는 각을 6도로 제한하였다.
![pol2_angle](https://user-images.githubusercontent.com/70250834/101276177-6d464d00-37ee-11eb-824c-0f4bf85034f9.png)

![Multirotor2](https://user-images.githubusercontent.com/70250834/101276209-82bb7700-37ee-11eb-9777-9fc910344f5a.gif)

5. simulation 5
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188061-c5e33380-2f2c-11eb-9324-d7dba8e8a3b9.png width="18%" height="9%">

* Mission
|time(s)|Altitude(m)|x(m)|y(m)|yaw|
|--|--|--|--|--|
|0~3.5|4|3|2|10|
|3.5~6.2|4|5|6|10|

![ball1_2D](https://user-images.githubusercontent.com/70250834/101276144-60c1f480-37ee-11eb-9316-728a11d17ccf.png)

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![ball1_thrust](https://user-images.githubusercontent.com/70250834/101276150-63bce500-37ee-11eb-96a3-5cbfb1b928de.png)
![ball1_euler](https://user-images.githubusercontent.com/70250834/101276147-61f32180-37ee-11eb-993e-5bb747b1fe37.png)
![ball1_rotor](https://user-images.githubusercontent.com/70250834/101276149-63244e80-37ee-11eb-837a-a78a31f35b4f.png)
![ball1_NED](https://user-images.githubusercontent.com/70250834/101276148-628bb800-37ee-11eb-8f57-81265b382565.png)
![ball1_3D](https://user-images.githubusercontent.com/70250834/101276145-615a8b00-37ee-11eb-9581-791b8ceb555b.png)

착륙시 기체가 급기동하는 것을 방지하기 위하여 착륙 직전의 일부 스텝의 추력 명령이 Z축과 이루는 각을 6도로 제한하였다.
![ball1_angle](https://user-images.githubusercontent.com/70250834/101276146-61f32180-37ee-11eb-9253-eded9f0366a0.png)

![Multirotor3](https://user-images.githubusercontent.com/70250834/101276213-851dd100-37ee-11eb-8493-c3fd3ab32692.gif)

6. simulation 6
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188063-c67bca00-2f2c-11eb-9e2b-1b6a8386be8b.png width="18%" height="9%">

* Mission
|time(s)|Altitude(m)|x(m)|y(m)|yaw|
|--|--|--|--|--|
|0~3.5|4|3|2|10|
|3.5~6.2|4|5|6|10|

![ball2_2D](https://user-images.githubusercontent.com/70250834/101276151-63bce500-37ee-11eb-83bb-71997ddd2281.png)

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![ball2_thrust](https://user-images.githubusercontent.com/70250834/101276157-661f3f00-37ee-11eb-9210-19c783f9e36e.png)
![ball2_euler](https://user-images.githubusercontent.com/70250834/101276154-64ee1200-37ee-11eb-92dd-c6e9812d1f43.png)
![ball2_rotor](https://user-images.githubusercontent.com/70250834/101276156-661f3f00-37ee-11eb-9dcc-bf2d435b30f0.png)
![ball2_NED](https://user-images.githubusercontent.com/70250834/101276155-6586a880-37ee-11eb-89f9-7162113857d3.png)
![ball2_3D](https://user-images.githubusercontent.com/70250834/101276152-64557b80-37ee-11eb-8e55-b36f1edbddea.png)

착륙시 기체가 급기동하는 것을 방지하기 위하여 착륙 직전의 일부 스텝의 추력 명령이 Z축과 이루는 각을 6도로 제한하였다.
![ball2_angle](https://user-images.githubusercontent.com/70250834/101276153-64ee1200-37ee-11eb-82a3-7ec83b55a566.png)

![Multirotor4](https://user-images.githubusercontent.com/70250834/101276206-7f27f000-37ee-11eb-836b-f1f28a63a142.gif)

7. simulation 7
* 총 시뮬레이션 시간은 12초이며 고장 발생 시간은 6초, 고장 검출에 걸린 시간은 0.2초로 하였다.
* 착륙 위치 제한은 아래 식과 같다.

<img src = https://user-images.githubusercontent.com/70250834/100188064-c67bca00-2f2c-11eb-8f49-3351ea68d565.png width="14%" height="7%">

* Mission
|time(s)|Altitude(m)|x(m)|y(m)|yaw|
|--|--|--|--|--|
|0~3.5|4|3|2|10|
|3.5~6.2|4|5|6|10|

![ball3_2D](https://user-images.githubusercontent.com/70250834/101276158-66b7d580-37ee-11eb-82fb-d73a55aba942.png)

* graph
붉은 점선은 고장 시점, 푸른 점선은 알고리듬 적용 시점, 노란 점선은 착륙 시점이다.

![ball3_thrust](https://user-images.githubusercontent.com/70250834/101276164-691a2f80-37ee-11eb-9c9e-64653b1e300e.png)
![ball3_euler](https://user-images.githubusercontent.com/70250834/101276161-67e90280-37ee-11eb-81e7-52af77c7e642.png)
![ball3_rotor](https://user-images.githubusercontent.com/70250834/101276163-691a2f80-37ee-11eb-829b-79ecf58d5ccc.png)
![ball3_NED](https://user-images.githubusercontent.com/70250834/101276162-67e90280-37ee-11eb-93f4-89296230f27c.png)
![ball3_3D](https://user-images.githubusercontent.com/70250834/101276159-66b7d580-37ee-11eb-8307-ed3a108500dc.png)

착륙시 기체가 급기동하는 것을 방지하기 위하여 착륙 직전의 일부 스텝의 추력 명령이 Z축과 이루는 각을 6도로 제한하였다.
![ball3_angle](https://user-images.githubusercontent.com/70250834/101276160-67506c00-37ee-11eb-8c8c-f625997e8771.png)

![Multirotor5](https://user-images.githubusercontent.com/70250834/101276208-818a4a00-37ee-11eb-89db-a0b8595f3c64.gif)
