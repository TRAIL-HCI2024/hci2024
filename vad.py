import os
import torch
from pydub import AudioSegment
from speech_recognizer import whisper_make_transcription
from speech_interpreter import generate_response
import time
torch.set_num_threads(1)

RATE = 16000

def vad(file_dir: str):
  model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad')
  (get_speech_timestamps, _, read_audio, _, _) = utils

  while True:
    for filename in os.listdir(file_dir):
      if filename.endswith(".wav"):
        file_path = os.path.join(file_dir, filename)
        print(file_path)
        wav = read_audio(file_path)
        speech_timestamps = get_speech_timestamps(wav, model, sampling_rate=RATE)
        print(speech_timestamps)

        if len(speech_timestamps) > 0:
            audio = AudioSegment.from_wav(file_path)

            for i, timestamp in enumerate(speech_timestamps):
                start_ms = (timestamp['start'] / RATE) * 1000
                end_ms = (timestamp['end'] / RATE) * 1000
                segment = audio[start_ms:end_ms]
                output_path = os.path.join(file_dir, f"{filename[:-4]}_segment_{i}.wav")
                segment.export(output_path, format="wav")

                transcription = whisper_make_transcription(output_path)
                print(generate_response(transcription))

                os.remove(output_path)

        os.remove(file_path)
    time.sleep(1)

if __name__=="__main__":
    vad("data/audio")
