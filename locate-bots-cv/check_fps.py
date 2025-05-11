import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
# import cscore
import time

# 웹캠 열기 (0번 장치)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) # CAP_DSHOW, CAP_MSMF
# cap = cscore.VideoCapture(0)

vid_fmt = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
cap.set(cv2.CAP_PROP_FOURCC, vid_fmt)

# 해상도
width = cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 가로 해상도
height = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 세로 해상도

# fps
fps = cap.get(cv2.CAP_PROP_FPS)  # FPS
print(f"   ▶ 변경 전 FPS: {fps}")
cap.set(cv2.CAP_PROP_FPS, 60)  # FPS
print(f"   ▶ 변경 후 FPS: {cap.get(cv2.CAP_PROP_FPS)}")

# 자동 노출 끄기 (0으로 설정)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()
    
# 📷 웹캠 기본 정보
#    ▶ 해상도: 640x480
#    ▶ FPS: 30.0
#    ▶ 밝기(Brightness): 128.0
#    ▶ 대비(Contrast): 128.0
#    ▶ 채도(Saturation): 128.0
#    ▶ 노출(Exposure): -5.0


print(f"   ▶ 해상도: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")



# 밝기 (0~1)
brightness = cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)  # 밝기
print(f"   ▶ 밝기(Brightness): {cap.get(cv2.CAP_PROP_BRIGHTNESS)}")

# 대비 (0~1)
# contrast = cap.set(cv2.CAP_PROP_CONTRAST, 0)  # 대비
print(f"   ▶ 대비(Contrast): {cap.get(cv2.CAP_PROP_CONTRAST)}")

saturation = cap.get(cv2.CAP_PROP_SATURATION)  # 채도
exposure = cap.get(cv2.CAP_PROP_EXPOSURE)  # 노출

# ▶ 웹캠 정보 출력

print(f"   ▶ 채도(Saturation): {saturation}")
print(f"   ▶ 노출(Exposure): {exposure}")

vid_fmt = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
cap.set(cv2.CAP_PROP_FOURCC, vid_fmt)

# FPS를 수동으로 계산할 변수들
fps = 0
frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break
    
    # FPS 수동 계산
    frame_count += 1
    elapsed_time = time.time() - start_time  # 경과 시간
    if elapsed_time >= 1:
        fps = frame_count / elapsed_time  # FPS 계산
        start_time = time.time()  # 시간 초기화
        frame_count = 0  # 카운트 초기화

    # 화면에 FPS 정보 표시
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Logitech Webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()