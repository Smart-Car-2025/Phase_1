import cv2
import numpy as np

class TrafficLightDetector:
    def __init__(self):
        # Khởi tạo các giá trị mặc định
        self.brightness = 100
        self.contrast = 25

        # Không gian màu HSV
        self.colors = {
            'Red': {
                'lower1': [0, 100, 100],
                'upper1': [10, 255, 255],
                'lower2': [160, 100, 100],
                'upper2': [180, 255, 255]
            },
            'Yellow': {
                'lower': [20, 100, 100],
                'upper': [35, 255, 255]
            },
            'Green': {
                'lower': [40, 100, 100],
                'upper': [80, 255, 255]
            }
        }

    def adjust_image(self, image):
        # Điều chỉnh độ sáng và độ tương phản
        alpha = self.contrast / 50.0  # Độ tương phản
        beta = self.brightness - 50  # Độ sáng

        adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        return adjusted

    def detect_traffic_light(self, frame):
        # Điều chỉnh độ sáng và độ tương phản
        frame = self.adjust_image(frame)

        # Chuyển sang không gian màu HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Kernel để khử nhiễu
        kernel = np.ones((5, 5), np.uint8)

        # Theo dõi số lượng pixel của từng màu
        color_pixel_counts = {'Red': 0, 'Yellow': 0, 'Green': 0}

        # Theo dõi màu đèn
        dominant_color = None
        dominant_contour = None

        for color, color_range in self.colors.items():
            if color == 'Red':
                # Xử lý đặc biệt cho màu đỏ (2 phạm vi)
                mask1 = cv2.inRange(hsv,
                                    np.array(color_range['lower1']),
                                    np.array(color_range['upper1']))
                mask2 = cv2.inRange(hsv,
                                    np.array(color_range['lower2']),
                                    np.array(color_range['upper2']))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv,
                                   np.array(color_range['lower']),
                                   np.array(color_range['upper']))

            # Khử nhiễu
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Đếm số pixel
            color_pixel_counts[color] = cv2.countNonZero(mask)

            # Tìm contour
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Lọc contour lớn nhất
            for contour in contours:
                if cv2.contourArea(contour) > 50:
                    area = cv2.contourArea(contour)
                    if dominant_contour is None or area > cv2.contourArea(dominant_contour):
                        dominant_contour = contour
                        dominant_color = color

        # Vẽ đèn giao thông duy nhất
        if dominant_contour is not None:
            (x, y), radius = cv2.minEnclosingCircle(dominant_contour)
            center = (int(x), int(y))
            radius = int(radius)

            # Chọn màu viền dựa trên màu đèn
            if dominant_color == 'Red':
                border_color = (0, 0, 255)  # Đỏ
                # Thêm cảnh báo dừng xe khi đèn đỏ
                cv2.rectangle(frame, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 255), 10)
                cv2.putText(frame, "STOP!", (frame.shape[1] // 3 - 50, frame.shape[0] // 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            elif dominant_color == 'Yellow':
                border_color = (0, 255, 255)  # Vàng
            else:
                border_color = (0, 255, 0)  # Xanh

            # Vẽ hình tròn
            cv2.circle(frame, center, radius, border_color, 2)

            # Vẽ văn bản nhãn và thông tin pixel
            label_text = f"{dominant_color} Light ({color_pixel_counts[dominant_color]} px)"
            cv2.putText(frame, label_text,
                        (int(x), int(y - radius - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        border_color, 2)

        return frame, dominant_color, color_pixel_counts
