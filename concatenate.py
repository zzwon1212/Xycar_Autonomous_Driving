import sys
import argparse
import cv2
import time
# from cv_bridge import CvBridge
# import rospy
# from rosbag import Bag

# python3 /workspace/Programmers/xycar_ws/src/concatenate.py

def parse_args_():
    parser = argparse.ArgumentParser(description="CONVERT")
    parser.add_argument(
        "--input1", dest="input1", help="input video1 path", type=str,
        default="/workspace/Programmers/xycar_ws/src/phone.mp4"
    )
    parser.add_argument(
        "--input2", dest="input2", help="input video2 path", type=str,
        default="/workspace/Programmers/xycar_ws/src/xycar.mp4"
    )
    parser.add_argument(
        "--output", dest="output", help="output video path", type=str,
        default="/workspace/Programmers/xycar_ws/src/video_concatenated.mp4"
    )

    # if len(sys.argv) != 3:
        # parser.print_help()
        # sys.exit(1)

    args = parser.parse_args()

    return args

def get_video(path):
    cap = cv2.VideoCapture(path)

    if not cap.isOpened():
        print("Error opening video file")
        sys.exit()

    return cap

def convert_mp4_to_rosbag(cap, output_path):
    bridge = CvBridge()
    fps = cap.get(cv2.CAP_PROP_FPS)

    with Bag(output_path, "w") as bag:
        while cap.isOpened():
            ret, frame = cap.read()
            # frame = cv2.resize(frame, (570, 1012))
            if not ret:
                break
            img = bridge.cv2_to_imgmsg(frame, "bgr8")
            img.header.stamp = rospy.Time.now()
            bag.write("/video/image_raw", img)
            rospy.sleep(1 / fps)

    cap.release()

if __name__ == "__main__":
    args = parse_args_()
    cap1 = get_video(args.input1)
    cap2 = get_video(args.input2)

    fps1 = cap1.get(cv2.CAP_PROP_FPS)
    fps2 = cap2.get(cv2.CAP_PROP_FPS)

    writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'), fps1, (1920, 1012))

    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not (ret1 and ret2):
            print("Error reading VideoCapture")
            sys.exit()

        frame1 = cv2.resize(frame1, (570, 1012))
        frame2 = cv2.resize(frame2, (1350, 1012))

        frame_concatenated = cv2.hconcat([frame1, frame2])
        writer.write(frame_concatenated)

        # cv2.imshow("Result", frame_concatenated)
        # cv2.waitKey(1)

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()
