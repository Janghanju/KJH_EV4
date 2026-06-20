@echo off
echo ========================================
echo  KJH_EV4 열화상 모니터 패키지 설치
echo ========================================
echo.

REM python -m pip 방식 사용 (pip 명령이 없어도 동작)
python -m pip install --upgrade pip
python -m pip install pyserial matplotlib numpy Pillow

echo.
echo ========================================
echo  설치 완료! 아래 명령으로 실행하세요:
echo    cd tools
echo    python thermal_monitor.py
echo.
echo  자동 연결:
echo    python thermal_monitor.py --port COM3 --mode "ESP32 릴레이"
echo ========================================
pause
