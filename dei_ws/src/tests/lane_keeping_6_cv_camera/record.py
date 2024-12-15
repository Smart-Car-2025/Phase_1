import cv2

def record_video(output_filename="output_video.mp4", camera_index=0, fps=30):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Không th? m? camera.")
        return

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

    print("Nh?n 'q' d? d?ng ghi hình.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Không th? d?c khung hình.")
                break

            out.write(frame)
            cv2.imshow("Camera", frame)

    except KeyboardInterrupt:
        print("Ðã d?ng ghi hình.")

    cap.release()
    out.release()
    cv2.destroyAllWindows()

