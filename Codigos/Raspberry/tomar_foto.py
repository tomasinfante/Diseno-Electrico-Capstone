from picamera2 import Picamera2
import sys
import time
from datetime import datetime


def tomar_foto(ruta=None):
    if ruta is None:
        ruta = datetime.now().strftime("foto_%Y%m%d_%H%M%S.jpg")

    picam2 = Picamera2()
    config = picam2.create_still_configuration()
    picam2.configure(config)

    picam2.start()
    time.sleep(2)  # tiempo de estabilización del sensor (AE/AWB)

    picam2.capture_file(ruta)
    picam2.close()

    print(f"Foto guardada en {ruta}")
    return ruta


if __name__ == "__main__":
    ruta = sys.argv[1] if len(sys.argv) > 1 else None
    tomar_foto(ruta)
