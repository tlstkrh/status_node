import cv2
import logging
from time import sleep

class SIYIRTSP:
    def __init__(self, rtsp_url="rtsp://192.168.144.25:8554/main.264", debug=False):
        """
        RTSP client for video stream.
        """
        self._rtsp_url = rtsp_url
        self._debug = debug

        # Configure logging
        self._logger = logging.getLogger(self.__class__.__name__)
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(
            logging.Formatter("[%(levelname)s] %(asctime)s [SIYIRTSP]: %(message)s")
        )
        self._logger.addHandler(console_handler)
        self._logger.setLevel(logging.DEBUG if self._debug else logging.INFO)

        self._stream = None
        self._stopped = False

    def start(self):
        """
        Start receiving RTSP stream.
        """
        self._logger.info("Connecting to RTSP stream: %s", self._rtsp_url)
        self._stream = cv2.VideoCapture(self._rtsp_url, cv2.CAP_FFMPEG)

        if not self._stream.isOpened():
            self._logger.error("Failed to open RTSP stream: %s", self._rtsp_url)
            return False

        self._logger.info("RTSP stream opened successfully.")
        return True

    def show_stream(self):
        """
        Display RTSP stream using OpenCV.
        """
        if not self.start():
            return

        while not self._stopped:
            ret, frame = self._stream.read()
            if not ret:
                self._logger.warning("Failed to read frame from RTSP stream.")
                break

            cv2.imshow("RTSP Stream", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                self._logger.info("Stopping stream...")
                self.stop()
                break

            sleep(0.01)

    def stop(self):
        """
        Stop the RTSP stream.
        """
        self._stopped = True
        if self._stream:
            self._stream.release()
        cv2.destroyAllWindows()
        self._logger.info("RTSP stream stopped.")

# Test function to run RTSP client
def test_rtsp():
    rtsp_client = SIYIRTSP(rtsp_url="rtsp://192.168.144.25:8554/main.264", debug=True)
    rtsp_client.show_stream()

if __name__ == "__main__":
    test_rtsp()
