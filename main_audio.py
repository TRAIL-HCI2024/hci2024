import threading
from vad import vad
from recorder import record_audio 

def main():
    file_dir = "data"

    vad_thread = threading.Thread(target=vad, args=(file_dir,))
    record_thread = threading.Thread(target=record_audio)

    vad_thread.start()
    record_thread.start()

    vad_thread.join()
    record_thread.join()

if __name__ == "__main__":
    main()
