import cv2
import numpy as np

def detect_objects(image):
    # この関数は、画像内のオブジェクトを検出し、その距離を推定します
    height, width = image.shape[:2]
    lower_half = image[height//2:, :]
    
    gray = cv2.cvtColor(lower_half, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    
    # 輪郭を検出
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    objects = []
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # 小さすぎる輪郭は無視
            x, y, w, h = cv2.boundingRect(contour)
            y += height // 2  # 下半分の画像での位置を元の画像での位置に変換
            
            # 距離の推定 (この例では、画像の下端からの距離を使用)
            distance = (height - (y + h)) / height * 10  # 10mを最大距離と仮定
            
            objects.append({
                'bbox': (x, y, w, h),
                'distance': distance
            })
    
    return objects