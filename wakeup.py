import platform
import os
import sys
sys.path.append("/home/orangepi/.local/lib/python3.8/site-packages")
from lcd import  Lcd
from emoji import read_img_dict,do_emoji

class Logger(object):
    def __init__(self, filename="log.txt"):
        self.terminal = sys.stdout
        self.log_file = open(filename, "w") #覆盖上次开机的log,防止log文件过大

    def write(self, message):
        self.terminal.write(message)
        self.log_file.write(message)
        self.flush() #调用flush立即写入，相当于printf(flush=True)，默认是False

    def flush(self):
        self.terminal.flush()
        self.log_file.flush()

#sys.stdout = Logger()

link_file="_snowboydetect.so"
if os.path.exists(link_file):
    os.remove(link_file)
if platform.machine() == "x86_64":
    os.system("ln -s snowboy/_snowboydetect_x86.so _snowboydetect.so")
else:
    os.system("ln -s snowboy/_snowboydetect_arm.so _snowboydetect.so")

from snowboy import snowboydecoder
import signal

interrupted = False

def signal_handler(signal, frame):
    global interrupted
    exit(0)
    interrupted = True

# capture SIGINT signal, e.g., Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

#Returns True if the main loop needs to stop.
def interrupt_callback():
    global interrupted
    if interrupted == True:
        exit(0)
    return interrupted


lcd = Lcd()
imgs_dict = read_img_dict()

def detectedCallback1():
    print("已检测到关键词1")
    do_emoji(lcd, imgs_dict, "左上看")

def detectedCallback2():
    print("已检测到关键词2")
    do_emoji(lcd, imgs_dict, "右上看")

def detectedCallback3():
    print("已检测到关键词3")
    do_emoji(lcd, imgs_dict, "兴奋")

def detectedCallback4():
    print("已检测到关键词4")
    do_emoji(lcd, imgs_dict, "眨眼")


if __name__ == "__main__":
    print('Listening... Press Ctrl+C to exit')
    do_emoji(lcd, imgs_dict, "图片")

    #model = "snowboy/resources/xiaoaitongxue.pmdl"
    # detector = snowboydecoder.HotwordDetector(models, sensitivity=0.45)

    models = [
    "snowboy/resources/models/snowboy.umdl",
    "snowboy/resources/xiaobai.pmdl",
    "snowboy/resources/wei_xiao.pmdl",
    "snowboy/resources/zha_yan.pmdl"]
    detector = snowboydecoder.HotwordDetector(models, sensitivity=0.59)

    detectedCallbacks = [
        detectedCallback1,
        detectedCallback2,
        detectedCallback3,
        detectedCallback4
    ]

    # main loop
    detector.start(detected_callback=detectedCallbacks,
                   audio_recorder_callback=None,
                   interrupt_check=interrupt_callback,
                   sleep_time=0.03, #0.03s=30ms检查一次pyaudio录制是否OK
                   silent_count_threshold=5, #10 沉默等待时间
                   recording_timeout=60)

    detector.terminate()


