import threading
from vad import vad
from recorder_hsr import record_audio

def main():
    file_dir = "data/audio"
    vad_thread = threading.Thread(target=vad, args=(file_dir,))
    vad_thread.start()

    record_audio()
    vad_thread.join()

if __name__ == "__main__":
    main()
