import os
from action.action import Action
import openai
import time
from speech_interpreter import generate_response
from vision.my_typing import Direction
from vision.vision import Vision
from typing import List


WHISPER_MODEL_NAME = "small"
WHISPER_DEVICE = "cpu"
UPLOAD_FOLDER = "uploads/audio"


def whisper_make_transcription(filename: str) -> str:
    OPEN_API_KEY = os.environ.get("OPENAI_API_KEY")
    client = openai.OpenAI(api_key=OPEN_API_KEY)

    try:
        with open(filename, "rb") as audio_file:
            # fileサイズを取得
            file_size = os.path.getsize(filename)
            print(f"file size from whisper: {file_size}")
            transcript = client.audio.transcriptions.create(
                model="whisper-1", file=audio_file, language="ja")
        return transcript.text
    except Exception as e:
        raise e


def whisper(file_dir: str, action: Action, vision: Vision, file_path_list: List[str]):
    flag = 0
    if not os.path.exists(file_dir):
        os.makedirs(file_dir)
    # clear folder
    for filename in os.listdir(file_dir):
        file_path = os.path.join(file_dir, filename)
        os.remove(file_path)
    action.register_initial_position()
    time.sleep(6)

    while True:
        for file_path in file_path_list:
            if file_path.endswith(".wav"):
                print(file_path)
                transcription = whisper_make_transcription(file_path)

                response = generate_response(transcription)
                if response["isOrder"]:
                    filename = os.path.basename(file_path)
                    timestamp, _ = os.path.splitext(filename)
                    direction = vision.search_direction_at(timestamp)

                    action.speak(response["response"])
                    action.look(direction)
                    action.find_and_pick(response["object"])
                    action.bring()
                    flag = 1
        file_path_list.clear()
        if flag:
            break
        time.sleep(1)


if __name__ == "__main__":
    with open("こんにちは.wav", "rb") as audio_file:
        audio_data = audio_file.read()
        print(whisper_make_transcription(audio_data))
