import threading
from vad import vad
from recorder_hsr import record_audio as record_audio_hsr
from recorder import record_audio 
from speech_recognizer import whisper

def main():
    file_dir = "data/audio"
    vad_thread = threading.Thread(target=whisper, args=(file_dir,))
    vad_thread.start()

    record_audio()
    vad_thread.join()

if __name__ == "__main__":
    main()
