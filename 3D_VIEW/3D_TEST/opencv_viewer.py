#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2021 Bitcraze AB
#
#  AI-deck demo
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc., 51
#  Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
#  Demo for showing streamed JPEG images from the AI-deck example.
#
#  By default this demo connects to the IP of the AI-deck example when in
#  Access point mode.
#
#  The demo works by opening a socket to the AI-deck, downloads a stream of
#  JPEG images and looks for start/end-of-frame for the streamed JPEG images.
#  Once an image has been fully downloaded it's rendered in the UI.
#
#  Note that the demo firmware is continously streaming JPEG files so a single
#  JPEG image is taken from the stream using the JPEG start-of-frame (0xFF 0xD8)
#  and the end-of-frame (0xFF 0xD9).

import argparse
import time
import socket, os, struct, time
import numpy as np
import cv2
from rembg import remove


# self.Args for setting IP/port of AI-deck. Default settings are for when
# AI-deck is in AP mode.
class AIDECK:
    def __init__(self, folder, deck_port, deck_ip):
        self.save = True

        print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print("Socket connected")

        imgdata = None
        data_buffer = bytearray()
        self.count = 0
        self.path = "images/" + folder
        try:
            os.makedirs(self.path)
        except:
            print("Folder already exist")

    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size - len(data)))
        return data

    def remove_bg(self, img):
        # input_path = glob.glob(folder+"/*.JPG")
        # k=0
        # self.image_list_BG = []

        # output_path = folder+"BG/result"+str(10+k)+".jpg"
        # input = cv2.imread(img)

        output = remove(img)

        # eliminel la 4 component ja que es el valor alhpa que sobra en les nostres imatges
        output = np.delete(output, 3, axis=2)

        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        # Threshold the image to create a mask
        _, mask = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)

        # Invert the mask
        mask_inv = cv2.bitwise_not(mask)

        # Apply the mask to the image
        foreground = cv2.bitwise_and(output, output, mask=mask)

        # Create a white background image
        background = np.full(output.shape, 255, dtype=np.uint8)

        # Apply the inverted mask to the background
        background = cv2.bitwise_and(background, background, mask=mask_inv)

        # Combine the foreground and background
        result = cv2.add(foreground, background)

        # self.image_list_BG.append(result)
        # k+=1
        return result

    def send_data(self):
        try:
            packed_length = struct.pack("<H", len("take"))
            self.client_socket.sendall(packed_length)
            self.client_socket.sendall(b"take")
            print("Datos enviados exitosamente")
        except Exception as e:
            print("Error al enviar datos:", str(e))

    def take_photo(self, factor):
        print("Take Photo")
        self.send_data()
        start = time.time()

        # First get the info
        packetInfoRaw = self.rx_bytes(4)
        # print(packetInfoRaw)
        [length, routing, function] = struct.unpack("<HBB", packetInfoRaw)
        # print("Length is {}".format(length))
        # print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
        # print("Function is 0x{:02X}".format(function))

        imgHeader = self.rx_bytes(length - 2)
        # print(imgHeader)
        # print("Length of data is {}".format(len(imgHeader)))
        [magic, width, height, depth, format, size] = struct.unpack(
            "<BHHBBI", imgHeader
        )

        if magic == 0xBC:
            # print("Magic is good")
            # print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
            # print("Image format is {}".format(format))
            # print("Image size is {} bytes".format(size))

            # Now we start rx the image, this will be split up in packages of some size
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = self.rx_bytes(4)
                [length, dst, src] = struct.unpack("<HBB", packetInfoRaw)
                # print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = self.rx_bytes(length - 2)
                imgStream.extend(chunk)

            self.count = self.count + 1

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
                bayer_img.shape = (244, 324)
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
                hsv[:, :, 2] = np.clip(hsv[:, :, 2] * factor, 0, 255)
                bayer_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                """cv2.imshow('Raw', bayer_img)"""
                scale_factor = 2.0  # Cambia el factor al valor deseado

                # Calcula las nuevas dimensiones
                new_width = int(bayer_img.shape[1] * scale_factor)
                new_height = int(bayer_img.shape[0] * scale_factor)

                bayer_img_rez = cv2.resize(
                    bayer_img, (new_width, new_height), interpolation=cv2.INTER_LINEAR
                )
                bayer_img_rez = self.remove_bg(bayer_img_rez)
                cv2.imwrite(
                    f"{self.path}3/img_k{self.count:06d}.png",
                    bayer_img_rez,
                    [cv2.IMWRITE_JPEG_QUALITY, 95],
                )

                # cv2.imshow('Color', color_img)
                if self.save:
                    cv2.imwrite(f"{self.path}/img_{self.count:06d}.png", bayer_img)

                    # cv2.imwrite(f"stream_out/debayer/img_{self.count:06d}.png", color_img)
                cv2.waitKey(1)
            else:
                with open("img.jpeg", "wb") as f:
                    f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                cv2.imshow("JPEG", decoded)
                cv2.waitKey(1)


if __name__ == "__main__":
    cam = AIDECK("Test1", 5000, "172.20.10.2")
    for i in range(60):
        print(i + 1)
        cam.take_photo(1)
        time.sleep(2)
