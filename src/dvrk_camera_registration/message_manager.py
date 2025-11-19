# Author: Brendan Burkhart
# Date: 2025-11-19

# (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import collections
import cv2
import threading

class MessageManager:
    class Level(Enum):
        INFO = 0
        WARNING = 1
        ERROR = 2

    def __init__(self, buffer_size=100, font_size=0.5):
        self.messages = collections.deque(maxlen=buffer_size)
        self.messages_lock = threading.Lock()

        self.padding = 15
        self.font_size = font_size
        self.font = cv2.FONT_HERSHEY_DUPLEX

        self.current_progress = 0
        self.in_progress = False

    def _add(self, level, message):
        print(message)

        messages = message.split("\n")

        with self.messages_lock:
            self.in_progress = False
            for message in messages:
                self.messages.appendleft((level, message))

    def info(self, message):
        self._add(MessageManager.Level.INFO, message)

    def warn(self, message):
        self._add(MessageManager.Level.WARNING, message)

    def error(self, message):
        self._add(MessageManager.Level.ERROR, message)

    def line_break(self):
        self._add(MessageManager.Level.INFO, "")

    def progress(self, progress):
        self.current_progress = progress
        with self.messages_lock:
            if self.in_progress:
                self.messages.popleft()

        percent = int(100 * self.current_progress)

        with self.messages_lock:
            self.messages.appendleft(
                (MessageManager.Level.INFO, "Progress: {}%".format(percent))
            )
            self.in_progress = self.current_progress != 1.0

    def _message_color(self, level):
        if level == MessageManager.Level.INFO:
            return (255, 255, 255)  # white
        elif level == MessageManager.Level.WARNING:
            return (80, 255, 255)  # yellow
        else:
            return (80, 80, 255)  # red

    def render(self, image, area):
        start = area[0] + area[2] - self.padding

        with self.messages_lock:
            for level, line in self.messages:
                size, _ = cv2.getTextSize(line, self.font, self.font_size, 1)
                location = (area[1] + self.padding, start)
                cv2.putText(
                    image,
                    line,
                    location,
                    fontFace=self.font,
                    fontScale=self.font_size,
                    color=self._message_color(level),
                    thickness=1,
                    lineType=cv2.LINE_AA,
                )
                start -= size[1] + self.padding
