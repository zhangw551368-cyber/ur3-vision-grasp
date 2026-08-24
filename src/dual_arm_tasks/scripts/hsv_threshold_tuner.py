#!/usr/bin/python3

import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class HSVThresholdTuner:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param(
            "~image_topic", "/kinect_1/kinect2/qhd/image_color"
        )
        self.input_is_compressed = bool(
            rospy.get_param(
                "~input_is_compressed", self.image_topic.endswith("/compressed")
            )
        )
        self.window_name = rospy.get_param("~window_name", "kinect2_1_hsv_tuner")
        self.preview_scale = float(rospy.get_param("~preview_scale", 0.55))
        self.save_path = rospy.get_param(
            "~save_path", "/tmp/kinect2_1_hsv_thresholds.yaml"
        )
        self.show_windows = bool(rospy.get_param("~show_windows", True))
        if self.show_windows and not os.environ.get("DISPLAY"):
            rospy.logwarn("DISPLAY is not set, disabling OpenCV windows")
            self.show_windows = False

        low_1 = self.get_hsv_param("~hsv_low_1", [0, 80, 40])
        high_1 = self.get_hsv_param("~hsv_high_1", [12, 255, 255])
        low_2 = self.get_hsv_param("~hsv_low_2", [168, 80, 40])
        high_2 = self.get_hsv_param("~hsv_high_2", [180, 255, 255])
        self.initial_values = {
            "h1_low": low_1[0],
            "h1_high": high_1[0],
            "h2_low": low_2[0],
            "h2_high": high_2[0],
            "s_low": low_1[1],
            "s_high": high_1[1],
            "v_low": low_1[2],
            "v_high": high_1[2],
            "use_h2": int(rospy.get_param("~use_second_range", True)),
            "reject_glare": int(rospy.get_param("~reject_glare", True)),
            "glare_s_max": int(rospy.get_param("~glare_s_max", 70)),
            "glare_v_min": int(rospy.get_param("~glare_v_min", 220)),
            "open_kernel": int(rospy.get_param("~open_kernel", 3)),
            "close_kernel": int(rospy.get_param("~close_kernel", 5)),
        }

        self.latest_bgr = None
        self.latest_hsv = None
        self.trackbars_ready = False
        self.last_saved_text = ""

        self.mask_pub = rospy.Publisher("hsv_threshold/mask", Image, queue_size=1)
        self.filtered_pub = rospy.Publisher(
            "hsv_threshold/filtered", Image, queue_size=1
        )
        self.debug_pub = rospy.Publisher("hsv_threshold/debug", Image, queue_size=1)
        self.glare_pub = rospy.Publisher(
            "hsv_threshold/glare_mask", Image, queue_size=1
        )

        msg_type = CompressedImage if self.input_is_compressed else Image
        self.sub = rospy.Subscriber(
            self.image_topic,
            msg_type,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "HSV tuner listening on %s compressed=%s",
            self.image_topic,
            self.input_is_compressed,
        )

    def get_hsv_param(self, name, default):
        value = rospy.get_param(name, default)
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            rospy.logwarn("Invalid %s=%s, using %s", name, value, default)
            value = default
        return [
            self.clamp_int(value[0], 0, 180),
            self.clamp_int(value[1], 0, 255),
            self.clamp_int(value[2], 0, 255),
        ]

    @staticmethod
    def clamp_int(value, minimum, maximum):
        return max(minimum, min(maximum, int(value)))

    @staticmethod
    def odd_kernel(value):
        value = int(value)
        if value <= 1:
            return 0
        if value % 2 == 0:
            value += 1
        return value

    @staticmethod
    def ordered_pair(a, b):
        a = int(a)
        b = int(b)
        return (a, b) if a <= b else (b, a)

    def create_trackbars(self):
        if not self.show_windows or self.trackbars_ready:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.createTrackbar(
            "H1 low", self.window_name, self.initial_values["h1_low"], 180, self.noop
        )
        cv2.createTrackbar(
            "H1 high",
            self.window_name,
            self.initial_values["h1_high"],
            180,
            self.noop,
        )
        cv2.createTrackbar(
            "Use H2", self.window_name, self.initial_values["use_h2"], 1, self.noop
        )
        cv2.createTrackbar(
            "H2 low", self.window_name, self.initial_values["h2_low"], 180, self.noop
        )
        cv2.createTrackbar(
            "H2 high",
            self.window_name,
            self.initial_values["h2_high"],
            180,
            self.noop,
        )
        cv2.createTrackbar(
            "S low", self.window_name, self.initial_values["s_low"], 255, self.noop
        )
        cv2.createTrackbar(
            "S high", self.window_name, self.initial_values["s_high"], 255, self.noop
        )
        cv2.createTrackbar(
            "V low", self.window_name, self.initial_values["v_low"], 255, self.noop
        )
        cv2.createTrackbar(
            "V high", self.window_name, self.initial_values["v_high"], 255, self.noop
        )
        cv2.createTrackbar(
            "Reject glare",
            self.window_name,
            self.initial_values["reject_glare"],
            1,
            self.noop,
        )
        cv2.createTrackbar(
            "Glare S max",
            self.window_name,
            self.initial_values["glare_s_max"],
            255,
            self.noop,
        )
        cv2.createTrackbar(
            "Glare V min",
            self.window_name,
            self.initial_values["glare_v_min"],
            255,
            self.noop,
        )
        cv2.createTrackbar(
            "Open k",
            self.window_name,
            self.initial_values["open_kernel"],
            21,
            self.noop,
        )
        cv2.createTrackbar(
            "Close k",
            self.window_name,
            self.initial_values["close_kernel"],
            21,
            self.noop,
        )
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.trackbars_ready = True

    @staticmethod
    def noop(_value):
        pass

    def get_trackbar(self, name):
        return cv2.getTrackbarPos(name, self.window_name)

    def current_config(self):
        if self.show_windows and self.trackbars_ready:
            values = {
                "h1_low": self.get_trackbar("H1 low"),
                "h1_high": self.get_trackbar("H1 high"),
                "use_h2": self.get_trackbar("Use H2"),
                "h2_low": self.get_trackbar("H2 low"),
                "h2_high": self.get_trackbar("H2 high"),
                "s_low": self.get_trackbar("S low"),
                "s_high": self.get_trackbar("S high"),
                "v_low": self.get_trackbar("V low"),
                "v_high": self.get_trackbar("V high"),
                "reject_glare": self.get_trackbar("Reject glare"),
                "glare_s_max": self.get_trackbar("Glare S max"),
                "glare_v_min": self.get_trackbar("Glare V min"),
                "open_kernel": self.get_trackbar("Open k"),
                "close_kernel": self.get_trackbar("Close k"),
            }
        else:
            values = dict(self.initial_values)

        values["h1_low"], values["h1_high"] = self.ordered_pair(
            values["h1_low"], values["h1_high"]
        )
        values["h2_low"], values["h2_high"] = self.ordered_pair(
            values["h2_low"], values["h2_high"]
        )
        values["s_low"], values["s_high"] = self.ordered_pair(
            values["s_low"], values["s_high"]
        )
        values["v_low"], values["v_high"] = self.ordered_pair(
            values["v_low"], values["v_high"]
        )
        values["glare_s_max"] = self.clamp_int(values["glare_s_max"], 0, 255)
        values["glare_v_min"] = self.clamp_int(values["glare_v_min"], 0, 255)
        values["open_kernel"] = self.odd_kernel(values["open_kernel"])
        values["close_kernel"] = self.odd_kernel(values["close_kernel"])
        return values

    def decode_image(self, msg):
        if self.input_is_compressed:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is None:
                raise CvBridgeError("cv2.imdecode returned None")
            return image
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def make_mask(self, hsv, config):
        low_1 = np.array(
            [config["h1_low"], config["s_low"], config["v_low"]], dtype=np.uint8
        )
        high_1 = np.array(
            [config["h1_high"], config["s_high"], config["v_high"]], dtype=np.uint8
        )
        mask = cv2.inRange(hsv, low_1, high_1)
        if config["use_h2"]:
            low_2 = np.array(
                [config["h2_low"], config["s_low"], config["v_low"]], dtype=np.uint8
            )
            high_2 = np.array(
                [config["h2_high"], config["s_high"], config["v_high"]],
                dtype=np.uint8,
            )
            mask = mask | cv2.inRange(hsv, low_2, high_2)

        glare_mask = np.zeros(mask.shape, dtype=np.uint8)
        if config["reject_glare"]:
            saturation = hsv[:, :, 1]
            value = hsv[:, :, 2]
            glare_mask[
                (saturation <= config["glare_s_max"])
                & (value >= config["glare_v_min"])
            ] = 255
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(glare_mask))

        if config["open_kernel"]:
            kernel = np.ones(
                (config["open_kernel"], config["open_kernel"]), dtype=np.uint8
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if config["close_kernel"]:
            kernel = np.ones(
                (config["close_kernel"], config["close_kernel"]), dtype=np.uint8
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask, glare_mask

    def annotate(self, bgr, mask, glare_mask, config):
        overlay = bgr.copy()
        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[-2]
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                overlay,
                "largest area {:.0f}".format(area),
                (x, max(24, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

        if config["reject_glare"]:
            glare_pixels = int(cv2.countNonZero(glare_mask))
            cv2.putText(
                overlay,
                "glare pixels {}".format(glare_pixels),
                (20, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 220, 255),
                2,
            )
        return overlay

    def publish_image(self, pub, cv_image, encoding, header):
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        msg.header = header
        pub.publish(msg)

    def labeled_panel(self, image, label, scale):
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if abs(scale - 1.0) > 1e-6:
            image = cv2.resize(
                image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        cv2.rectangle(image, (0, 0), (240, 32), (0, 0, 0), thickness=cv2.FILLED)
        cv2.putText(
            image,
            label,
            (10, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        return image

    def make_preview(self, bgr, mask, glare_mask, filtered, overlay):
        scale = max(0.1, min(1.5, self.preview_scale))
        mask_view = mask
        if cv2.countNonZero(glare_mask) > 0:
            mask_view = cv2.addWeighted(mask, 1.0, glare_mask, 0.45, 0)

        panels = [
            self.labeled_panel(bgr.copy(), "source", scale),
            self.labeled_panel(mask_view.copy(), "mask + glare", scale),
            self.labeled_panel(filtered.copy(), "filtered", scale),
            self.labeled_panel(overlay.copy(), "debug", scale),
        ]
        top = np.hstack([panels[0], panels[1]])
        bottom = np.hstack([panels[2], panels[3]])
        return np.vstack([top, bottom])

    def save_thresholds(self, config):
        low_1 = [config["h1_low"], config["s_low"], config["v_low"]]
        high_1 = [config["h1_high"], config["s_high"], config["v_high"]]
        low_2 = [config["h2_low"], config["s_low"], config["v_low"]]
        high_2 = [config["h2_high"], config["s_high"], config["v_high"]]
        text = "\n".join(
            [
                "# Saved by hsv_threshold_tuner.py",
                "red_low_1: {}".format(low_1),
                "red_high_1: {}".format(high_1),
                "red_low_2: {}".format(low_2),
                "red_high_2: {}".format(high_2),
                "hsv_tuner:",
                "  use_second_range: {}".format(bool(config["use_h2"])),
                "  reject_glare: {}".format(bool(config["reject_glare"])),
                "  glare_s_max: {}".format(config["glare_s_max"]),
                "  glare_v_min: {}".format(config["glare_v_min"]),
                "  open_kernel: {}".format(config["open_kernel"]),
                "  close_kernel: {}".format(config["close_kernel"]),
                "",
            ]
        )
        directory = os.path.dirname(os.path.abspath(self.save_path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.save_path, "w") as output:
            output.write(text)
        self.last_saved_text = text
        rospy.loginfo("Saved HSV thresholds to %s\n%s", self.save_path, text)

    def log_thresholds(self, config):
        low_1 = [config["h1_low"], config["s_low"], config["v_low"]]
        high_1 = [config["h1_high"], config["s_high"], config["v_high"]]
        low_2 = [config["h2_low"], config["s_low"], config["v_low"]]
        high_2 = [config["h2_high"], config["s_high"], config["v_high"]]
        rospy.loginfo(
            "HSV low1=%s high1=%s low2=%s high2=%s reject_glare=%s glare=(S<=%d,V>=%d)",
            low_1,
            high_1,
            low_2,
            high_2,
            bool(config["reject_glare"]),
            config["glare_s_max"],
            config["glare_v_min"],
        )

    def handle_key(self, config):
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("s"), ord("S")):
            self.save_thresholds(config)
        elif key in (ord("p"), ord("P")):
            self.log_thresholds(config)
        elif key in (ord("q"), ord("Q"), 27):
            rospy.signal_shutdown("HSV tuner window closed by keyboard")

    def mouse_callback(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.latest_bgr is None or self.latest_hsv is None:
            return
        scale = max(0.1, min(1.5, self.preview_scale))
        panel_width = int(round(self.latest_bgr.shape[1] * scale))
        panel_height = int(round(self.latest_bgr.shape[0] * scale))
        if x >= panel_width or y >= panel_height:
            return
        image_x = self.clamp_int(int(x / scale), 0, self.latest_bgr.shape[1] - 1)
        image_y = self.clamp_int(int(y / scale), 0, self.latest_bgr.shape[0] - 1)
        bgr = self.latest_bgr[image_y, image_x].tolist()
        hsv = self.latest_hsv[image_y, image_x].tolist()
        rospy.loginfo("Pixel x=%d y=%d BGR=%s HSV=%s", image_x, image_y, bgr, hsv)

    def image_callback(self, msg):
        try:
            bgr = self.decode_image(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(3.0, "Image conversion failed: %s", exc)
            return

        self.create_trackbars()
        config = self.current_config()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask, glare_mask = self.make_mask(hsv, config)
        filtered = cv2.bitwise_and(bgr, bgr, mask=mask)
        overlay = self.annotate(bgr, mask, glare_mask, config)

        self.latest_bgr = bgr
        self.latest_hsv = hsv

        self.publish_image(self.mask_pub, mask, "mono8", msg.header)
        self.publish_image(self.filtered_pub, filtered, "bgr8", msg.header)
        self.publish_image(self.debug_pub, overlay, "bgr8", msg.header)
        self.publish_image(self.glare_pub, glare_mask, "mono8", msg.header)

        if self.show_windows:
            preview = self.make_preview(bgr, mask, glare_mask, filtered, overlay)
            cv2.imshow(self.window_name, preview)
            self.handle_key(config)

    def shutdown(self):
        if self.show_windows and self.trackbars_ready:
            cv2.destroyWindow(self.window_name)


if __name__ == "__main__":
    rospy.init_node("hsv_threshold_tuner")
    HSVThresholdTuner()
    rospy.spin()
