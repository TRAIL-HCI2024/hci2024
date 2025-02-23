import threading
from recorder_hsr import record_audio as record_audio_hsr
# from recorder import record_audio
from speech_recognizer import whisper
from vision import vision
from action import action


def main():
    vis = vision.Vision()
    vis.start()
    act = action.Action()
    act.start()

    act.speak("こんにちは！")

    file_path_list = []
    file_dir = "data/audio"
    vad_thread = threading.Thread(
        target=whisper, args=(file_dir, act, vis, file_path_list))
    vad_thread.start()

    record_audio_hsr(file_path_list)
    vad_thread.join()


if __name__ == "__main__":
    main()
