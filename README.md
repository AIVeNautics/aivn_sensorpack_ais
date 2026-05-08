# AIS Serial ROS 2 Driver Guide

**패키지명:** `aivn_sensorpack_ais`  
**버전:** `v0.1.0`  
**작성일:** 2026-05-08  
**작성자:** 박재원  
**배포등급:** 내부전용 (Confidential)

---

## 개요 (Overview)

`aivn_sensorpack_ais`는 시리얼 포트로 수신되는 표준 AIS NMEA 0183 문장(`!AIVDM`, `!AIVDO`)을 파싱하여 ROS 2 토픽으로 퍼블리시하는 Python 기반 AIS 전용 드라이버 패키지입니다.

이 패키지는 AIS 6-bit armoring payload를 디코딩하고, 위치 보고 메시지와 정적 선박 정보 메시지를 `aivn_interfaces/msg/AisShip` 메시지로 변환합니다.

- **메인 노드:** `AisSerialNode`
- **메인 모듈:** `aivn_sensorpack_ais.ais_serial_node:main`
- **실행 진입점:** `console_scripts: ais_serial_node`
- **런치 파일:** `launch/ais_serial.launch.py`
- **기본 파라미터 파일:** `config/config.yaml`

---

## 사전 준비 (Prerequisites)

- ROS 2 Humble
- Python 3.10 환경
- 워크스페이스 예시: `~/your_ws`
- 시리얼 AIS 장비 또는 AIS 문장을 송출하는 가상 포트
- 기본 시리얼 설정: `38400 bps`, `8N1`

### 의존성

`package.xml` 기준 실행 의존성은 다음과 같습니다.

- `rclpy`
- `pyserial`
- `aivn_interfaces`
- `ros2launch`

> 주의: Python 코드에서 `import serial`을 사용하므로 실제 Python 실행 환경에 `pyserial`이 설치되어 있어야 합니다.  
> `ModuleNotFoundError: No module named 'serial'` 발생 시 `python3 -m pip install pyserial` 또는 `sudo apt install python3-serial`로 설치하세요.

---

## 구성 요소 (Nodes & Files)

| 파일/경로 | 설명 |
|---|---|
| `aivn_sensorpack_ais/ais_serial_node.py` | 시리얼 포트 오픈/재연결, AIS 문장 추출, 파싱 결과 퍼블리시를 담당하는 메인 ROS 2 노드 |
| `aivn_sensorpack_ais/ais_nmea_parser.py` | `!AIVDM`, `!AIVDO` 문장 검증, checksum 처리, fragment assembly, AIS payload 디코딩 |
| `aivn_sensorpack_ais/ais_sixbit.py` | AIS 6-bit armoring payload 변환 및 bit field 추출 유틸리티 |
| `launch/ais_serial.launch.py` | ROS 2 launch 파일. 포트와 보레이트 launch 인자 지원 |
| `config/config.yaml` | 기본 노드 파라미터 YAML |
| `setup.py` | Python 패키지 설치 및 console script entry point 설정 |
| `package.xml` | ROS 2 패키지 메타데이터 및 의존성 정의 |
| `test/test_ais_nmea_parser.py` | AIS type 1 위치 보고 파싱 단위 테스트 |

---

## 입력 문장 (Supported Sentences)

| Sentence ID | 설명 | 비고 |
|---|---|---|
| `!AIVDM` | AIS VHF Data-link Message | AIS 수신국에서 수신한 타 선박 AIS 메시지 |
| `!AIVDO` | AIS Own-vessel Data | 자선 AIS 송신 메시지 |

### 지원 AIS Message Type

| AIS Message Type | 설명 | 처리 내용 |
|---:|---|---|
| 1 | Position Report Class A | MMSI, navigation status, lat/lon, SOG, COG, heading |
| 2 | Position Report Class A Assigned Schedule | Type 1과 동일한 위치 필드 처리 |
| 3 | Position Report Class A Response to Interrogation | Type 1과 동일한 위치 필드 처리 |
| 5 | Static and Voyage Related Data | 선박명, 호출부호, 선박 타입 |
| 18 | Standard Class B CS Position Report | MMSI, lat/lon, SOG, COG, heading |
| 19 | Extended Class B Equipment Position Report | Type 18 위치 정보 + 선박명/선박 타입 일부 |
| 24 | Static Data Report | Part A: 선박명, Part B: 선박 타입/호출부호 |

지원하지 않는 AIS Message Type은 현재 `None`으로 처리되어 퍼블리시하지 않습니다.

---

## 체크섬 처리

AIS NMEA 문장은 다음 방식으로 checksum을 검증합니다.

1. 시작 문자 `!` 다음부터 `*` 직전까지의 문자열을 대상으로 XOR 계산
2. 계산 결과를 `*HH`의 16진수 checksum과 비교
3. `ais_checksum_required=true`인 경우 checksum이 없거나 불일치하면 파싱 에러 처리

예시:

```text
!AIVDM,1,1,,A,<payload>,0*CS
```

- `body`: `AIVDM,1,1,,A,<payload>,0`
- `checksum`: `CS`

---

## Fragment Assembly

AIS NMEA 문장은 긴 payload를 여러 fragment로 나누어 전송할 수 있습니다.

예시 필드 구조:

```text
!AIVDM,<total>,<number>,<sequence_id>,<channel>,<payload>,<fill_bits>*CS
```

이 패키지는 다음 key 기준으로 fragment를 조립합니다.

```python
(sentence_id, sequence_id, channel)
```

- 모든 fragment가 도착하면 payload를 순서대로 결합하여 디코딩합니다.
- 마지막 fragment의 `fill_bits`를 사용합니다.
- 오래된 fragment는 `fragment_ttl_sec` 기준으로 정리됩니다.
- 현재 `fragment_ttl_sec` 기본값은 parser 내부 기본값 `8.0초`입니다.

---

## 퍼블리시 토픽 (ROS 2 Outputs)

### 메인 출력

| 항목 | 값 |
|---|---|
| Topic | `ais_topic_name` 파라미터 값 |
| 기본 Topic | `/edge_server/external/ais/ship` |
| Type | `aivn_interfaces/msg/AisShip` |
| Queue depth | `100` |

---

## `AisDecoded` → `AisShip` 필드 매핑

노드는 parser의 내부 디코딩 결과인 `AisDecoded`를 `aivn_interfaces/msg/AisShip`으로 변환하여 퍼블리시합니다.

| `AisShip` 필드 | 값/의미 |
|---|---|
| `header.stamp` | ROS clock 기준 현재 시간 |
| `header.frame_id` | `ais_frame_id` 파라미터 값. 기본값: `ais` |
| `ais_message_id` | AIS message type ID |
| `mmsi` | AIS MMSI 번호 |
| `ship_id` | 현재 구현에서는 MMSI 문자열 기본 사용 |
| `ship_name` | Type 5, 19, 24 등에서 획득한 선박명 |
| `call_sign` | Type 5, 24 Part B에서 획득한 호출부호 |
| `ship_type` | Type 5, 19, 24 Part B에서 획득한 선박 타입 코드 |
| `navigation_status` | Class A 위치 보고의 항해 상태. 알 수 없으면 `15` |
| `lat` | 위도, degree |
| `lon` | 경도, degree |
| `sog` | Speed Over Ground, knot |
| `cog` | Course Over Ground, degree |
| `heading` | True heading, degree. AIS unavailable 값은 `511` |
| `position_valid` | 위도/경도가 정상 범위이면 `true` |
| `static_valid` | 선박명/호출부호/선박 타입 등 정적 정보가 유효하면 `true` |
| `receiving_time_unix` | ROS clock 기준 UNIX timestamp 초 단위 |
| `original_sentence` | 원본 AIS NMEA 문장. multi-fragment는 줄바꿈으로 결합된 원문 |
| `source_port` | 수신한 시리얼 포트 경로 |

---

## 정적 선박 정보 캐시

AIS의 위치 보고 메시지(Type 1/2/3/18)는 보통 선박명이나 호출부호를 포함하지 않습니다.  
이 패키지는 MMSI 기준으로 정적 정보를 캐시하여 이후 위치 보고 메시지에 보강합니다.

- 캐시 key: `mmsi`
- 캐시 대상: `ship_name`, `call_sign`, `ship_type`
- 기본 유효 시간: `ais_stale_static_info_sec = 600.0초`
- 캐시 정보가 유효하면 위치 보고 메시지에도 정적 정보를 채워 퍼블리시합니다.

---

## 파라미터 (Parameters)

노드 내부에서 선언되는 런타임 파라미터는 다음과 같습니다.

| 이름 | 타입/기본값 | 설명 |
|---|---:|---|
| `ais_serial_port_name` | string `/dev/ttyUSB0` | AIS 시리얼 포트 경로 |
| `ais_baud_rate` | int `38400` | AIS 시리얼 보레이트 |
| `ais_topic_name` | string `/edge_server/external/ais/ship` | `AisShip` 퍼블리시 토픽 |
| `ais_frame_id` | string `ais` | 메시지 `header.frame_id` |
| `ais_verbose` | bool `false` | 원문 출력 및 요약 로그 활성화 |
| `ais_checksum_required` | bool `true` | AIS NMEA checksum 필수 여부 |
| `ais_poll_period_sec` | double `0.005` | 시리얼 polling timer 주기 |
| `ais_read_size` | int `8192` | 1회 read 최대 byte 수 |
| `ais_reconnect_sec` | double `2.0` | 시리얼 open 실패 후 재시도 간격 |
| `ais_stale_static_info_sec` | double `600.0` | MMSI별 정적 정보 캐시 유효 시간 |

### 기본 YAML

`config/config.yaml`

```yaml
ais_serial_node:
  ros__parameters:
    ais_serial_port_name: "/dev/ttyUSB0"
    ais_baud_rate: 38400
    ais_topic_name: "/edge_server/external/ais/ship"
    ais_frame_id: "ais"
    ais_verbose: false
    ais_checksum_required: true
    ais_poll_period_sec: 0.005
    ais_read_size: 8192
    ais_reconnect_sec: 2.0
    ais_stale_static_info_sec: 600.0
```

### Launch 인자

`launch/ais_serial.launch.py`에서 직접 override 가능한 launch argument는 다음 두 가지입니다.

| Launch argument | 기본값 | 설명 |
|---|---:|---|
| `ais_serial_port_name` | `/dev/ttyUSB0` | AIS 시리얼 포트 경로 |
| `ais_baud_rate` | `38400` | AIS 시리얼 보레이트 |

예시:

```bash
ros2 launch aivn_sensorpack_ais ais_serial.launch.py \
  ais_serial_port_name:=/dev/ttyUSB0 \
  ais_baud_rate:=38400
```

---

## 실행 (Build & Run)

### 1) 빌드

```bash
cd ~/your_ws
colcon build --packages-select aivn_sensorpack_ais
source install/setup.bash
```

### 2) Launch 실행

```bash
ros2 launch aivn_sensorpack_ais ais_serial.launch.py
```

포트와 보레이트를 지정하려면:

```bash
ros2 launch aivn_sensorpack_ais ais_serial.launch.py \
  ais_serial_port_name:=/dev/ttyUSB0 \
  ais_baud_rate:=38400
```

### 3) 직접 실행 (Debug)

```bash
ros2 run aivn_sensorpack_ais ais_serial_node
```

파라미터를 직접 지정하려면:

```bash
ros2 run aivn_sensorpack_ais ais_serial_node --ros-args \
  -p ais_serial_port_name:=/dev/ttyUSB0 \
  -p ais_baud_rate:=38400 \
  -p ais_topic_name:=/edge_server/external/ais/ship \
  -p ais_frame_id:=ais \
  -p ais_verbose:=true
```

가상 포트 또는 테스트 입력을 사용하는 경우:

```bash
ros2 run aivn_sensorpack_ais ais_serial_node --ros-args \
  -p ais_serial_port_name:=/tmp/ais_in \
  -p ais_topic_name:=/edge_server/external/ais/ship \
  -p ais_verbose:=true
```

---

## 모니터링 (Monitoring)

### 토픽 확인

```bash
ros2 topic list | grep ais
```

### 퍼블리시 데이터 확인

```bash
ros2 topic echo /edge_server/external/ais/ship
```

### 메시지 타입 확인

```bash
ros2 interface show aivn_interfaces/msg/AisShip
```

### 노드 파라미터 확인

```bash
ros2 param list /ais_serial_node
ros2 param get /ais_serial_node ais_serial_port_name
ros2 param get /ais_serial_node ais_baud_rate
ros2 param get /ais_serial_node ais_topic_name
```

---

## 예시 출력

### AIS 위치 보고 수신 시 (Type 1/2/3 또는 Type 18)

```yaml
header:
  stamp:
    sec: 1710000000
    nanosec: 123456789
  frame_id: ais
ais_message_id: 1
mmsi: 440123456
ship_id: "440123456"
ship_name: ""
call_sign: ""
ship_type: 0
navigation_status: 0
lat: 35.123456
lon: 129.123456
sog: 12.3
cog: 45.6
heading: 90
position_valid: true
static_valid: false
receiving_time_unix: 1710000000
original_sentence: "!AIVDM,1,1,,A,...*CS"
source_port: "/dev/ttyUSB0"
```

### AIS 정적 정보 수신 시 (Type 5 또는 Type 24)

```yaml
header:
  frame_id: ais
ais_message_id: 5
mmsi: 440123456
ship_id: "440123456"
ship_name: "AIVEN"
call_sign: "D7ABC"
ship_type: 70
navigation_status: 15
lat: 0.0
lon: 0.0
sog: 0.0
cog: 0.0
heading: 511
position_valid: false
static_valid: true
source_port: "/dev/ttyUSB0"
```

---

## 예외/로깅 (Errors & Logs)

### 시리얼 오픈 실패

```text
AIS serial open failed: ...
```

주요 원인:

- 포트 경로가 존재하지 않음
- 권한 문제 (`dialout` 그룹 미포함 등)
- 이미 다른 프로세스가 포트를 사용 중

### 시리얼 read 실패

```text
AIS serial read error: ...
```

read 중 예외가 발생하면 serial 객체를 닫고, 다음 polling cycle에서 재연결을 시도합니다.

### AIS 파싱 실패

```text
AIS parse error: ... / raw=...
```

주요 원인:

- checksum 불일치
- 잘못된 fragment 번호
- payload 길이 부족
- 잘못된 AIS 6-bit payload 문자

### Verbose 모드

`ais_verbose=true`이면 원문과 요약 로그를 출력합니다.

원문 출력:

```text
ais_original_sentence: '!AIVDM,...*CS'
```

5초 주기 요약 로그:

```text
AIS summary: bytes=... sentences=... decoded_ok=... decoded_err=... published=...
```

---

## 테스트 (Tests)

현재 포함된 단위 테스트는 AIS Type 1 위치 보고 문장의 lat/lon, SOG, COG, heading 디코딩을 검증합니다.

```bash
cd ~/your_ws
colcon test --packages-select aivn_sensorpack_ais
colcon test-result --verbose
```

또는 패키지 디렉터리에서 pytest를 직접 실행할 수 있습니다.

```bash
python3 -m pytest test/test_ais_nmea_parser.py
```

---

## 개발 참고 사항

### 현재 구현상 특징

- 입력은 `!`로 시작하고 `*HH` checksum을 포함하는 문장만 추출합니다.
- `!AIVDM`, `!AIVDO` 외 문장은 무시합니다.
- `\r\n` 또는 `\n` 라인 종료 문자를 제거하고 처리합니다.
- `use_sim_time`은 별도로 선언하지 않지만 ROS 2 기본 시간 파라미터로 사용할 수 있습니다.
- launch 파일에는 현재 `respawn=True`가 설정되어 있지 않습니다.

### 확인된 개선 권장 사항

1. `setup.py`의 `install_requires`에 `pyserial` 추가 권장

현재 `setup.py`는 다음처럼 되어 있습니다.

```python
install_requires=["setuptools"]
```

Python 패키지 설치 관점에서는 아래처럼 `pyserial`을 추가하는 것이 안전합니다.

```python
install_requires=["setuptools", "pyserial"]
```

2. 런치 자동 재시작이 필요하면 `Node(..., respawn=True, respawn_delay=2.0)` 추가 검토

현재 런치 파일에는 respawn 옵션이 없습니다. 운영 환경에서 시리얼 장치 장애 또는 노드 크래시 대응이 필요하면 다음 옵션을 검토하세요.

```python
Node(
    package="aivn_sensorpack_ais",
    executable="ais_serial_node",
    name="ais_serial_node",
    output="screen",
    respawn=True,
    respawn_delay=2.0,
    parameters=[...],
)
```

3. checksum 에러 통계 분리 검토

현재 통계는 `decoded_err`로 통합 집계됩니다. 운영 모니터링을 위해 checksum mismatch, unsupported message type, decode error 등을 분리하면 장애 분석에 유리합니다.

---

## 관련 메시지

- `aivn_interfaces/msg/AisShip.msg`

---

## 관련 명령 요약

```bash
# 빌드
cd ~/your_ws
colcon build --packages-select aivn_sensorpack_ais
source install/setup.bash

# 런치 실행
ros2 launch aivn_sensorpack_ais ais_serial.launch.py \
  ais_serial_port_name:=/dev/ttyUSB0 \
  ais_baud_rate:=38400

# 직접 실행
ros2 run aivn_sensorpack_ais ais_serial_node --ros-args \
  -p ais_serial_port_name:=/dev/ttyUSB0 \
  -p ais_baud_rate:=38400 \
  -p ais_topic_name:=/edge_server/external/ais/ship \
  -p ais_verbose:=true

# 토픽 확인
ros2 topic echo /edge_server/external/ais/ship

# 메시지 타입 확인
ros2 interface show aivn_interfaces/msg/AisShip
```
