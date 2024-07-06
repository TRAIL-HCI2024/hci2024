import os

import openai 

WHISPER_MODEL_NAME = "small"
WHISPER_DEVICE = "cpu"
UPLOAD_FOLDER = "uploads/audio"


def whisper_make_transcription(filename: str) -> str:
    client = openai.OpenAI(api_key="sk-proj-d5M3eeV1P0wAYBU2eoQpT3BlbkFJkbaBKB2jFv1QRCq3ya9E")

    try:
        with open(filename, "rb") as audio_file:
            transcript = client.audio.transcriptions.create(model="whisper-1", file=audio_file, language="ja")
        return transcript.text
    except Exception as e:
        raise e

if __name__=="__main__":
    with open("こんにちは.wav", "rb") as audio_file:
        audio_data = audio_file.read()
        print(whisper_make_transcription(audio_data))
