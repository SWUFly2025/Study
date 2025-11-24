---
name: "🚀 Feature / 🐞 Bug Issue"
about: "드론(PX4 / Gazebo / MAVSDK / Python) 프로젝트 기능 개발 및 버그 보고 템플릿"
title: ""
labels: ""
assignees: ""
---

## 🚀 이슈 설명 (Issue Description)
- 이슈에 대해 설명해주세요.

## 🔍 구현 상세 (Implementation Details)
- 구현해야 할 기능 또는 버그의 원인을 구체적으로 설명해주세요.
- PX4, Gazebo, MAVSDK, Python 등의 관련 모듈이 있으면 명시해주세요.
- 센서, 토픽(subscription), 제어(set_velocity, offboard 등) 관련 처리도 포함해 주세요.

## 📋 체크리스트 (Checklist)
- [ ] PX4 SITL에서 재현 가능한가?
- [ ] Gazebo 환경(world, sdf, sensor) 수정이 필요한가?
- [ ] Offboard 모드 또는 MAVSDK API 동작 점검 완료
- [ ] 시뮬레이터(Gazebo / JMAVSim) 모두에서 테스트 완료
- [ ] Python 스크립트 로그 출력/에러 핸들링 확인
- [ ] 성능/안전성(altitude stability, drift, velocity overshoot 등) 검증

## 📎 관련 자료 (Related Resources)
- 참고할 링크가 있다면 여기에 추가해주세요.
  - PX4 문서  
  - MAVSDK Python API  
  - 사용 중인 world / 회로도 / UI mockup  

## 📝 추가 정보 (Additional Information)
- 하드웨어 연동 계획 여부
- 향후 확장(waypoint, tracking, obstacle avoidance 등) 고려사항
