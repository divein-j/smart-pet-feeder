# SMART-PET-FEEDER
졸업작품-반려동물용 사료 자동 배급기

📌프로젝트 소개

외출 시에도 정해진 시간에 반려동물에게 자동으로 사료와 물을 제공하는 IoT 기반 자동 급식기


🗓️개발 기간

2025.07 ~ 2025.11 (4개월)


🛠️사용 기술

**하드웨어**
- 마이크로컨트롤러 : ATmega128 AVR (JMOD-128-1)
- SG90 서보모터 2개
- RTC 실시간 시계 모듈 (DS3231)
- HC-05 블루투스 모듈
- 1602 LCD I2C 모듈
- 워터펌프 모터
- 수위 센서
- 릴레이 모듈
- 전원 어댑터 5V/2A, 터미널블록 to DC 커넥터

**소프트웨어**
- 언어 : C
- 개발환경 : Microchip Studio
- 앱 개발 : Android Studio (블루투스 제어 앱)
- 통신 프로토콜 : UART(블루투스), I2C (LCD, RTC)


✨주요 기능

- 자동 급식 : 설정된 시간에 자동으로 사료 배급 (하루 2회)
- 자동 급수 : 수위 센서로 물 부족 감지 시 자동 급수
- 센서 모니터링 : 사료 잔량 및 물 수위 실시간 감지
- LCD 디스플레이 : 현재 시간, 급식 일정, 센서 상태 표시
- 긴급 정지 : 긴급 상황 시 모든 모터 즉시 정지
**블루투스 원격 제어**
- 급식 / 급수 시간 설정
- 수동 급식 / 급수
- 실시간 상태 조회
- 급식 이력 조회


💻시스템 구조

- Main Controller : ATmega128에서 모든 센서 및 actuator 제어
- 통신 : HC-05 블루투스로 스마트폰 앱과 양방향 통신
- 타이머 : DS3231 RTC로 정확한 시간 기반 자동 급식
- 모터 제어 : 서보모터 PWM 제어로 정밀한 사료 배급
- 펌프 제어 : 릴레이 모듈로 워터펌프 ON / OFF 제어 


👥나의 역할 (2인 팀 프로젝트)

- 하드웨어 회로 설계 및 조립
- ATmega128 펌웨어 개발 (C/AVR)
- Android 블루투스 제어 앱 개발
- 블루투스 통신 프로토콜 설계
- I2C 통신(LCD,RTC) 구현
- 센서 데이터 처리 및 모터 제어 시스템 구현


🧪 트러블슈팅

### 워터펌프 동작 시 서보모터 오작동 문제

**문제**  
워터펌프 가동 중 서보모터가 의도치 않게 오작동하는 현상 발생

**원인**  
펌프 구동 시 전원 간섭으로 PWM 신호가 불안정해짐

**해결**  
펌프 동작 전 PWM 타이머(TCCR1A, TCCR1B)를 완전히 차단하고,  
서보 핀을 입력 모드로 전환한 뒤 펌프 종료 후 재초기화하는 방식으로 해결

**배운 점**  
하드웨어 간 전원 간섭을 소프트웨어 타이머 제어로 해결하는  
하드웨어-소프트웨어 통합 디버깅 경험

## ✅ 동작 검증

- 자동 급식 시간 동작 확인 (06:00 / 18:00 기준, 시간 변경 가능)
- 수위 센서 5회 샘플링 중 3회 이상 감지 시 Empty 판정 → 자동 급수 동작 확인
- 블루투스 앱 연동 수동 제어(급식/급수) 및 상태·이력 조회 확인
- 긴급 정지 명령 수신 시 모터 즉시 정지 및 복구(EMERGENCY_RESET) 확인
- 펌프 동작 중 긴급 정지 명령 수신 시 즉시 중단 처리 확인


💡개발 과정에서 배운 점

- ATmega128 데이터시트 분석 및 레지스터 제어
- I2C, UART 통신 프로토콜 이해 및 구현
- 하드웨어 PWM을 이용한 정밀 모터 제어
- 실시간 시스템에서의 타이밍 처리
- 하드웨어-소프트웨어 통합 디버깅 경험


📸 실행 화면

#완성된 시스템

<img width="986" height="825" alt="image" src="https://github.com/user-attachments/assets/d8da6c13-70d1-4e75-adb3-d1280128d77c" />
<img width="1150" height="862" alt="image" src="https://github.com/user-attachments/assets/446206de-8e56-4329-869c-ef9d64e909c7" />


#LCD 디스플레이

<img width="314" height="315" alt="image" src="https://github.com/user-attachments/assets/98734d5d-a4b3-4390-8743-5c082773d5dd" />
<img width="314" height="315" alt="image" src="https://github.com/user-attachments/assets/c9dd8c3a-a542-41e8-984b-40669807d3a6" />
<img width="314" height="315" alt="image" src="https://github.com/user-attachments/assets/dbaf46ee-5740-4df4-a8b9-fa1e6a03afa0" />
<img width="314" height="315" alt="image" src="https://github.com/user-attachments/assets/2c4bdcea-0311-42c4-b7c2-e389ec78db19" />


#블루투스 앱

<img width="600" height="977" alt="image" src="https://github.com/user-attachments/assets/7a5d3254-143a-4252-a047-d4df29514788" />


🔗 관련 링크
