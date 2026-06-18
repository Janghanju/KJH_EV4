#!/bin/bash
echo "========================================"
echo " KJH_EV4 열화상 모니터 패키지 설치"
echo "========================================"
echo ""

# python3 또는 python 중 존재하는 것 사용
if command -v python3 &>/dev/null; then
    PY="python3"
elif command -v python &>/dev/null; then
    PY="python"
else
    echo "[ERROR] Python이 설치되어 있지 않습니다."
    echo "        https://www.python.org 에서 설치 후 다시 시도하세요."
    exit 1
fi

echo "Python: $($PY --version)"
echo ""

$PY -m pip install --upgrade pip
$PY -m pip install pyserial matplotlib numpy

echo ""
echo "========================================"
echo " 설치 완료! 아래 명령으로 실행하세요:"
echo "   $PY thermal_monitor.py"
echo "========================================"
