multirotor_emergency_landing
==============================
Multirotor control allocation and emergency landing
-----------------------------------------------------
  >본 연구에서는 컨벡스 최적화 기법을 이용하여 다중프로펠러 비행체의 고장 대응 알고리듬을 설계한다.
 
## 개요
고장대응 알고리듬은 비행체의 고장 검출이 완료되었다는 가정하에 설계되었으며, 고장 검출 직후 실행된다.
알고리듬은 비행체 구동을 위한 roll, pitch, yaw 자세각 명령과 추력 명령을 출력하여 비행체가 고장 위치에서 착륙 가능 조건을 만족하는 가장 가까운 위치로 최소 추력을 사용하여 착륙하도록 한다.
알고리듬에는 다음의 식을 컨벡스화한 식이 사용되었다.

추력 크기의 차이를 줄이기 위하여 아래 식 a를 제한조건으로 사용하였으며, 비행체의 자세를 적당히 유지하기 위하여 식 b를 적용하였다. 또한, 착륙이 가능한 위치를 제한하기 위해 식 3을 추가하였다.



위 식에서 

## 비상착륙 알고리듬
1. 고장 검출이 끝난 시점에서 비행체의 위치와 속도를 초기값으로 한다.
2. Bisection Method를 이용하여 최적 N을 정한다.
3. 컨벡스 문제를 N번 풀어 3축 추력 명령을 얻는다.
4. 가장 첫 번째 step의 추력 명령을 자세각(roll, pitch, yaw)명령과 추력 명령으로 변환한다.
5. 4에서 계산한 값을 비행체의 자세 명령으로 반환한다.

## 시뮬레이션 결과
1. simulation 1
2. simulation 2
3. simulation 3


