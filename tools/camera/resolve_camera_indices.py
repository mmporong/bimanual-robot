"""물리적 USB 장치 경로로 카메라 인덱스를 매번 다시 찾아낸다.

윈도우 DirectShow 인덱스는 세션마다(브리지 재시작, USB 재연결, 재부팅마다)
흔들린다 - 이건 못 막는다. 대신 각 장치의 DirectShow DevicePath(USB 포트
위치가 인코딩된 문자열)는 같은 물리 포트에 꽂혀있는 한 안 바뀐다. 손목캠
2대는 모델이 같아서 이름만으로는 구분이 안 되지만 DevicePath는 장치별로
고유하므로, "이 경로를 가진 장치가 지금 몇 번 인덱스인지"를 실행 직전에
매번 다시 찾으면 인덱스 번호 자체가 흔들려도 항상 올바른 인덱스를 쓸 수
있다.

LEFT_DEVPATH_HINT / RIGHT_DEVPATH_HINT는 2026-10-20 세션에서 사용자가
"1=왼쪽, 2=오른쪽"이라고 직접 확인한 순간 그 장치들의 DevicePath에서 뽑은
고유 부분이다. 손목캠을 물리적으로 다른 USB 포트에 옮기면 이 값도 다시
확인해서 갱신해야 한다 - capture_indexed.py로 확인 후 enum_device_paths
방식으로 재추출.

사용법:
    python tools/camera/resolve_camera_indices.py
    -> left=N / right=N / depth=N 출력
"""
import comtypes
from comtypes.persist import IPropertyBag
from pygrabber.dshow_graph import SystemDeviceEnum
from pygrabber.dshow_ids import DeviceCategories

LEFT_DEVPATH_HINT = "268b1cad"
RIGHT_DEVPATH_HINT = "30e3ab42"
DEPTH_NAME = "OBS Virtual Camera"


def enumerate_devices():
    sde = SystemDeviceEnum()
    filter_enumerator = sde.system_device_enum.CreateClassEnumerator(
        comtypes.GUID(DeviceCategories.VideoInputDevice), dwFlags=0
    )
    devices = []
    moniker, count = filter_enumerator.Next(1)
    idx = 0
    while count > 0:
        pb = moniker.BindToStorage(0, 0, IPropertyBag._iid_).QueryInterface(IPropertyBag)
        try:
            name = pb.Read("FriendlyName", pErrorLog=None)
        except Exception:
            name = ""
        try:
            devpath = pb.Read("DevicePath", pErrorLog=None)
        except Exception:
            devpath = ""
        devices.append((idx, name, devpath))
        idx += 1
        moniker, count = filter_enumerator.Next(1)
    return devices


def resolve():
    devices = enumerate_devices()
    result = {}
    for idx, name, devpath in devices:
        if LEFT_DEVPATH_HINT in devpath:
            result["left"] = idx
        elif RIGHT_DEVPATH_HINT in devpath:
            result["right"] = idx
        elif name == DEPTH_NAME:
            result["depth"] = idx
    missing = {"left", "right", "depth"} - result.keys()
    if missing:
        detail = "\n".join(f"  slot{i}: name={n!r} devpath={d!r}" for i, n, d in devices)
        raise RuntimeError(f"다음 카메라를 못 찾음: {missing}.\n현재 전체 장치 목록:\n{detail}")
    return result


if __name__ == "__main__":
    r = resolve()
    print(f"left={r['left']}")
    print(f"right={r['right']}")
    print(f"depth={r['depth']}")
