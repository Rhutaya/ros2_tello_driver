import socket
import cv2
import h264decoder
import numpy as np

decoder = h264decoder.H264Decoder()

def _h264_decode(packet_data, decoder):
    res_frame_list = []
    frames = decoder.decode(packet_data)
    for framedata in frames:
        (frame, w, h, ls) = framedata
        if frame is not None:

            frame = np.fromstring(frame, dtype=np.ubyte, count=len(frame), sep='')
            frame = (frame.reshape((h, int(ls / 3), 3)))
            frame = frame[:, :w, :]
            res_frame_list.append(frame)

    return res_frame_list

video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_socket.bind(('0.0.0.0', 11111))
video_socket.settimeout(10)

packet_data = b''

while True:
    try:
        res_string, ip = video_socket.recvfrom(2048)

        packet_data += res_string
        # end of frame
        if len(res_string) != 1460:
            for frame in _h264_decode(packet_data, decoder):
                cv2.imshow("video", frame)
                cv2.waitKey(1)
            packet_data = b''

    except socket.error as exc:
        print ("Caught exception socket.error : %s" % exc)


