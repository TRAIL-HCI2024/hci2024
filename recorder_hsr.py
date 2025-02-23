import datetime
import os
import wave
import rospy
from audio_common_msgs.msg import AudioData
from typing import List

# 音声設定
CHANNELS = 1
RATE = 16000


def callback(msg, queue):
    queue.append(msg.data)

# HSRで動くように書き換える


def record_audio(file_path_list: List[str]):
    print("recording...")
    # rospy.init_node('listener', anonymous=True)
    while True:
        # floatに変更
        start_time = datetime.datetime.now().timestamp()

        frames: List[bytes] = []
        rospy.Subscriber("audio/audio", AudioData, callback, frames)
        print("start record")
        rospy.sleep(5)
        print("Finished recording.")

        if not os.path.exists("data/audio"):
            os.makedirs("data/audio")

        print(f"frames len: {len(frames)}")

        # ファイル名を現在の日時に基づいて設定
        filename = f"data/audio/{str(start_time)}.wav_temp"

        # 音声データを保存
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()

        # ファイル名を変更
        os.rename(filename, filename.replace("_temp", ""))
        new_name = filename.replace("_temp", "")
        file_path_list.append(new_name)
        # ファイルサイズを表示
        print(f"file size: {os.path.getsize(new_name)}")


if __name__ == "__main__":
    record_audio()
