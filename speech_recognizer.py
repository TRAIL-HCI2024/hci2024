import os
from action.action import Action
import openai 
import time
from speech_interpreter import generate_response
from vision.vision import Vision
from vision.typing import Direction


WHISPER_MODEL_NAME = "small"
WHISPER_DEVICE = "cpu"
UPLOAD_FOLDER = "uploads/audio"


def whisper_make_transcription(filename: str) -> str:
    OPEN_API_KEY = os.environ.get("OPENAI_API_KEY")
    client = openai.OpenAI(api_key=OPEN_API_KEY)

    try:
        with open(filename, "rb") as audio_file:
            transcript = client.audio.transcriptions.create(model="whisper-1", file=audio_file, language="ja")
        return transcript.text
    except Exception as e:
        raise e
    
def whisper(file_dir: str, action: Action):
    flag = 0
    while True:
        for filename in os.listdir(file_dir):
            if filename.endswith(".wav"):
                file_path = os.path.join(file_dir, filename)
                print(file_path)
                transcription = whisper_make_transcription(file_path)
                
                response = generate_response(transcription)
                if response["isOrder"]:
                    timestamp, _ = os.path.splitext(filename)
                    direction: Direction = vision.search_direction_at(timestamp)

                    action.speak(response["response"])
                    action.look(direction)
                    action.find_and_pick(response["object"])
                    flag = 1
                    break
                    
                os.remove(file_path)
        if flag:
            break
        time.sleep(1)

if __name__=="__main__":
    with open("こんにちは.wav", "rb") as audio_file:
        audio_data = audio_file.read()
        print(whisper_make_transcription(audio_data))
