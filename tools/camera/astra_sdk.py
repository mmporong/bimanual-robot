"""Astra SDK bin 폴더 찾기. 노트북마다 압축을 푼 위치가 달라 한 곳에서 찾는다.

우선순위: 환경 변수 ASTRA_SDK_BIN -> 아래 후보 중 astra.dll이 있는 첫 폴더.
새 노트북에서 다른 곳에 풀었다면 `setx ASTRA_SDK_BIN "<...>\bin"`으로 지정한다.
"""
import os
from pathlib import Path

SDK_NAME = "AstraSDK-v2.1.3-94bca0f52e-20210608T034051Z-vs2015-win64"
CANDIDATE_ROOTS = (
    Path.home() / "Downloads",
    Path("D:/astra"),
    Path("C:/astra"),
    Path.home() / "astra",
)


def find_sdk_bin() -> str:
    env = os.environ.get("ASTRA_SDK_BIN")
    if env:
        return env
    for root in CANDIDATE_ROOTS:
        # 압축을 풀면 같은 이름 폴더가 한 겹 더 생기는 경우와 안 생기는 경우를 모두 본다
        for sub in (Path(SDK_NAME) / SDK_NAME / "bin", Path(SDK_NAME) / "bin"):
            if (root / sub / "astra.dll").is_file():
                return str(root / sub)
    return str(CANDIDATE_ROOTS[0] / SDK_NAME / SDK_NAME / "bin")  # 없으면 기존 기본 경로(오류 메시지에 표시)
