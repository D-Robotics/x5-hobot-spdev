// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import sys, os, time

import numpy as np
import cv2
from hobot_vio import libsrcampy

def get_nalu_pos(byte_stream):
    size = byte_stream.__len__()
    nals = []
    retnals = []

    startCodePrefixShort = b"\x00\x00\x01"

    pos = 0
    while pos < size:
        is4bytes = False
        retpos = byte_stream.find(startCodePrefixShort, pos)
        if retpos == -1:
            break
        if byte_stream[retpos - 1] == 0:
            retpos -= 1
            is4bytes = True
        if is4bytes:
            pos = retpos + 4
        else:
            pos = retpos + 3
        val = hex(byte_stream[pos])
        val = "{:d}".format(byte_stream[pos], 4)
        val = int(val)
        fb = (val >> 7) & 0x1
        nri = (val >> 5) & 0x3
        type = val & 0x1f
        nals.append((pos, is4bytes, fb, nri, type))
    for i in range(0, len(nals) - 1):
        start = nals[i][0]
        if nals[i + 1][1]:
            end = nals[i + 1][0] - 5
        else:
            end = nals[i + 1][0] - 4
        retnals.append((start, end, nals[i][1], nals[i][2], nals[i][3], nals[i][4]))
    start = nals[-1][0]
    end = byte_stream.__len__() - 1
    retnals.append((start, end, nals[-1][1], nals[-1][2], nals[-1][3], nals[-1][4]))
    return retnals

def get_h264_nalu_type(byte_stream):
    nalu_types = []
    nalu_pos = get_nalu_pos(byte_stream)

    for idx, (start, end, is4bytes, fb, nri, type) in enumerate(nalu_pos):
        # print("NAL#%d: %d, %d, %d, %d, %d" % (idx, start, end, fb, nri, type))
        nalu_types.append(type)
    return nalu_types

def test_camera():
    print("[INFO] Running test_camera")
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, -1, 30, 1920, 1080)
    print("Camera open_cam return:%d" % ret)
    time.sleep(3)
    img = cam.get_img(2)
    if img is not None:
        #save file
        fo = open("output.yuv", "wb")
        fo.write(img)
        fo.close()
        print("camera save img file success")
    else:
        print("camera save img file failed")
    cam.close_cam()
    print("test_camera done!!!")

def test_camera_vps():
    print("[INFO] Running test_camera_vps")
    vps = libsrcampy.Camera()
    ret = vps.open_vps(1, 1, 1920, 1080, 512, 512)
    print("Camera vps return:%d" % ret)

    fin = open("output.yuv", "rb")
    img = fin.read()
    fin.close()
    ret = vps.set_img(img)
    print ("Process set_img return:%d" % ret)

    fo = open("output_vps.img", "wb+")
    img = vps.get_img()
    if img is not None:
        fo.write(img)
        print("encode write image success")
    else:
        print("encode write image failed")
    fo.close()

    vps.close_cam()
    print("test_camera_vps done!!!")

def test_encode():
    print("[INFO] Running test_encode")
    #encode file
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)

    #save file
    fo = open("encode.h264", "wb+")
    a = 0
    fin = open("output.yuv", "rb")
    input_img = fin.read()
    fin.close()
    while a < 100:
        ret = enc.encode_file(input_img)
        print("Encoder encode_file return:%d" % ret)
        img = enc.get_img()
        if img is not None:
            fo.write(img)
            print("encode write image success count: %d" % a)
        else:
            print("encode write image failed count: %d" % a)
        a = a + 1

    enc.close()
    print("test_encode done!!!")

def test_decode():
    print("[INFO] Running test_decode")
    #decode start
    dec = libsrcampy.Decoder()

    ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    img = dec.get_img()
    if img is not None:
        #save file
        fo = open("output.yuv", "wb")
        fo.write(img)
        fo.close()
        print("decode save img file success")
    else:
        print("decode save img file failed")

    dec.close()
    print("test_decode done!!!")

def test_display():
    print("[INFO] Running test_display")
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)
    ret = disp.display(2)
    print ("Display display 2 return:%d" % ret)
    ret = disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
    print ("Display set_graph_rect return:%d" % ret)
    string = "horizon"
    ret = disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
    print ("Display set_graph_word return:%d" % ret)

    fo = open("output.yuv", "rb")
    img = fo.read()
    fo.close()
    ret = disp.set_img(img)
    print ("Display set_img return:%d" % ret)

    time.sleep(3)

    disp.close()
    print("test_display done!!!")

def test_camera_bind_encode():
    print("[INFO] Running test_camera_bind_encode")
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, -1, 30, [1920, 1280], [1080, 720])
    print("Camera open_cam return:%d" % ret)

    #encode start
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)
    ret = libsrcampy.bind(cam, enc)
    print("libsrcampy bind return:%d" % ret)

    enc1 = libsrcampy.Encoder()
    ret = enc1.encode(1, 1, 1280, 720)
    print("Encoder encode return:%d" % ret)
    ret = libsrcampy.bind(cam, enc1)
    print("libsrcampy bind return:%d" % ret)

    #save file
    fo = open("encode.h264", "wb+")
    fo1 = open("encode1.h264", "wb+")
    a = 0
    while a < 100:
        img = enc.get_img()
        img1 = enc1.get_img()
        if img is not None:
            fo.write(img)
            fo1.write(img1)
            print("encode write image success count: %d" % a)
        else:
            print("encode write image failed count: %d" % a)
        a = a + 1
    fo.close()
    fo1.close()

    print("save encode file success")
    ret = libsrcampy.unbind(cam, enc)
    print("libsrcampy unbind return:%d" % ret)
    ret = libsrcampy.unbind(cam, enc1)
    print("libsrcampy unbind return:%d" % ret)

    enc1.close()
    enc.close()
    cam.close_cam()
    print("test_camera_bind_encode done!!!")

def test_camera_bind_display():
    print("[INFO] Running test_camera_bind_display")
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, -1, 30, 1280, 720)
    print("Camera open_cam return:%d" % ret)

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1, chn_width = 1280, chn_height = 720)
    print ("Display display 0 return:%d" % ret)
    ret = disp.display(2, chn_width = 1280, chn_height = 720)
    print ("Display display 2 return:%d" % ret)
    disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
    string = "horizon"
    disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
    ret = libsrcampy.bind(cam, disp)
    print("libsrcampy bind return:%d" % ret)

    time.sleep(10)

    ret = libsrcampy.unbind(cam, disp)
    print("libsrcampy unbind return:%d" % ret)
    disp.close()
    cam.close_cam()
    print("test_camera_bind_display done!!!")

def test_decode_bind_display():
    print("[INFO] Running test_decode_bind_display")
    #decode start
    dec = libsrcampy.Decoder()
    ret = dec.decode("encode.h264", 0, 1, 1920, 1080)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    dec1 = libsrcampy.Decoder()
    ret = dec1.decode("encode1.h264", 1, 1, 1280, 720)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)
    ret = disp.display(2)
    print ("Display display 2 return:%d" % ret)
    disp.set_graph_rect(100, 100, 1920, 200, chn = 2, flush = 1,  color = 0xffff00ff)
    string = "horizon"
    disp.set_graph_word(300, 300, string.encode('gb2312'), 2, 0, 0xff00ffff)
    ret = libsrcampy.bind(dec, disp)
    print("libsrcampy bind return:%d" % ret)

    time.sleep(5)

    ret = libsrcampy.unbind(dec, disp)
    print("libsrcampy unbind return:%d" % ret)
    disp.close()
    dec1.close()
    dec.close()
    print("test_decode_bind_display done!!!")

def test_cam_bind_encode_decode_bind_display():
    print("[INFO] Running test_cam_bind_encode_decode_bind_display")
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, -1, 30, [1920, 1280], [1080, 720])
    print("Camera open_cam return:%d" % ret)

    #encode file
    enc = libsrcampy.Encoder()
    ret = enc.encode(0, 1, 1920, 1080)
    print("Encoder encode return:%d" % ret)

    #decode start
    dec = libsrcampy.Decoder()
    ret = dec.decode("", 0, 1, 1920, 1080)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    ret = libsrcampy.bind(cam, enc)
    print("libsrcampy bind return:%d" % ret)
    ret = libsrcampy.bind(dec, disp)
    print("libsrcampy bind return:%d" % ret)

    a = 0
    while a < 100:
        img = enc.get_img()
        if img is not None:
            dec.set_img(img)
            print("encode get image success count: %d" % a)
        else:
            print("encode get image failed count: %d" % a)
        a = a + 1

    ret = libsrcampy.unbind(cam, enc)
    ret = libsrcampy.unbind(dec, disp)
    disp.close()
    dec.close()
    enc.close()
    cam.close_cam()
    print("test_cam_bind_encode_decode_bind_display done!!!")

def test_cam_vps_display():
    print("[INFO] Running test_cam_vps_display")
    #camera start
    cam = libsrcampy.Camera()
    ret = cam.open_cam(0, -1, 30, [1920, 1280], [1080, 720])
    print("Camera open_cam return:%d" % ret)

    #vps start
    vps = libsrcampy.Camera()
    ret = vps.open_vps(1, 1, 1920, 1080, 512, 512)
    print("Camera vps return:%d" % ret)

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    a = 0
    while a < 100:
        img = cam.get_img()
        if img is not None:
            vps.set_img(img)
            print("camera get image success count: %d" % a)
        else:
            print("camera get image failed count: %d" % a)

        img = vps.get_img(2, 1920, 1080)
        if img is not None:
            disp.set_img(img)
            print("vps get image success count: %d" % a)
        else:
            print("vps get image failed count: %d" % a)
        a = a + 1

    disp.close()
    vps.close_cam()
    cam.close_cam()
    print("test_cam_vps_display done!!!")

def test_rtsp_decode_bind_vps_bind_disp():
    rtsp_url = "rtsp://127.0.0.1/3840x2160.264"
    print(f"[INFO] Running test_rtsp_decode_bind_vps_bind_disp with URL: {rtsp_url}")
    find_pps_sps = 0

    #rtsp start
    cap = cv2.VideoCapture(rtsp_url)
    cap.set(cv2.CAP_PROP_FORMAT, -1) # get stream
    if not cap.isOpened():
        print("fail to open rtsp: {}".format(rtsp_url))
        return -1
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    #decode start
    dec = libsrcampy.Decoder()
    ret = dec.decode("", 0, 1, width, height)
    print ("Decoder return:%d frame count: %d" %(ret[0], ret[1]))

    #camera start
    vps = libsrcampy.Camera()
    ret = vps.open_vps(0, 1, width, height, [1920, 512], [1080, 512])
    print("Camera open_cam return:%d" % ret)

    #display start
    disp = libsrcampy.Display()
    ret = disp.display(0, 1920, 1080, 0, 1)
    print ("Display display 0 return:%d" % ret)

    ret = libsrcampy.bind(dec, vps)
    print("libsrcampy bind return:%d" % ret)
    ret = libsrcampy.bind(vps, disp)
    print("libsrcampy bind return:%d" % ret)

    a = 0
    while a < 500:
        ret, stream_frame = cap.read()
        if not ret:
            print("decode rtsp get frame failed")
            continue
        nalu_types = get_h264_nalu_type(stream_frame.tobytes())

        # first frame need be pps，sps
        if (nalu_types[0] in [1, 5]) and find_pps_sps == 0:
            continue

        find_pps_sps = 1
        if stream_frame is not None:
            ret = dec.set_img(stream_frame.tobytes(), 0)
            if ret != 0:
                print("decode set image failed")
            a = a + 1

    ret = libsrcampy.unbind(dec, vps)
    ret = libsrcampy.unbind(vps, disp)
    disp.close()
    dec.close()
    vps.close_cam()
    cap.release()
    print("test_rtsp_decode_bind_vps_bind_disp done!!!")


def main(selected_tests):
    tests = {
        1: test_camera,
        2: test_camera_vps,
        3: test_encode,
        4: test_decode,
        5: test_display,
        6: test_camera_bind_encode,
        7: test_camera_bind_display,
        8: test_decode_bind_display,
        9: test_cam_bind_encode_decode_bind_display,
        10: test_cam_vps_display,
        11: test_rtsp_decode_bind_vps_bind_disp,
    }
    for test_num in selected_tests:
        if test_num in tests:
            tests[test_num]()
        else:
            print(f"Invalid test number: {test_num}")

if __name__ == "__main__":
    print("Available tests:")
    print("1: test_camera")
    print("2: test_camera_vps")
    print("3: test_encode")
    print("4: test_decode")
    print("5: test_display")
    print("6: test_camera_bind_encode")
    print("7: test_camera_bind_display")
    print("8: test_decode_bind_display")
    print("9: test_cam_bind_encode_decode_bind_display")
    print("10: test_cam_vps_display")
    print("11: test_rtsp_decode_bind_vps_bind_disp")
    print("Enter the numbers of tests to run (separated by spaces), or 'all' to run all tests:")

    user_input = input().strip().lower()

    if user_input == 'all':
        selected_tests = range(1, 12)
    else:
        selected_tests = [int(num) for num in user_input.split()]

    main(selected_tests)
