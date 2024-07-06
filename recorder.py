import pyaudio
import wave
import datetime
import os

# 音声設定
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 2048
RECORD_SECONDS = 5

###HSRで動くように書き換える
def record_audio():
    audio = pyaudio.PyAudio()
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                          rate=RATE, input=True,
                          frames_per_buffer=CHUNK)
    
    while True:
      start_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

      print("Recording...")

      frames = []

      for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
          data = stream.read(CHUNK)
          frames.append(data)

      print("Finished recording.")

      if not os.path.exists("data"):
            os.makedirs("data")

      # ファイル名を現在の日時に基づいて設定
      end_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S.%f")
      filename = "data/" + start_time + "-" + end_time + ".wav"

      # 音声データを保存
      wf = wave.open(filename, 'wb')
      wf.setnchannels(CHANNELS)
      wf.setsampwidth(audio.get_sample_size(FORMAT))
      wf.setframerate(RATE)
      wf.writeframes(b''.join(frames))
      wf.close()

if __name__ == "__main__":
    record_audio()

