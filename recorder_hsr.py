import datetime
import os
import wave
import rospy
from audio_common_msgs.msg import AudioData

# 音声設定
CHANNELS = 1
RATE = 16000

def callback(msg, queue):
    queue.append(msg.data)

###HSRで動くように書き換える
def record_audio():
    print("recording...")
    #rospy.init_node('listener', anonymous=True)
    while True:
        start_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        frames = []
        rospy.Subscriber("audio/audio", AudioData, callback, frames)
        print("start record")
        rospy.sleep(5)
        print("Finished recording.")

        if not os.path.exists("data/audio"):
            os.makedirs("data/audio")

        # ファイル名を現在の日時に基づいて設定
        end_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S.%f")
        filename = "data/audio/" + start_time + "-" + end_time + ".wav"

        # 音声データを保存
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()

if __name__ == "__main__":
    record_audio()
