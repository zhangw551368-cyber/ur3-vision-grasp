#!/usr/bin/python3

"""Service-based control interface for the right Robotiq 2F gripper."""

import threading
import time

import rospy
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


def clamp_byte(value):
    return max(0, min(255, int(value)))


class RightArmGripperControl:
    def __init__(self):
        self.command_topic = rospy.get_param(
            "~command_topic", "/right_arm/Robotiq2FGripperRobotOutput"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/right_arm/Robotiq2FGripperRobotInput"
        )
        self.open_position = clamp_byte(rospy.get_param("~open_position", 0))
        self.close_position = clamp_byte(rospy.get_param("~close_position", 210))
        self.speed = clamp_byte(rospy.get_param("~speed", 120))
        self.force = clamp_byte(rospy.get_param("~force", 80))
        self.command_repeats = max(1, int(rospy.get_param("~command_repeats", 5)))
        self.command_interval = max(
            0.01, float(rospy.get_param("~command_interval", 0.05))
        )
        self.driver_timeout = max(
            0.1, float(rospy.get_param("~driver_timeout", 5.0))
        )
        self.motion_timeout = max(
            0.1, float(rospy.get_param("~motion_timeout", 5.0))
        )
        self.position_tolerance = max(
            0, int(rospy.get_param("~position_tolerance", 5))
        )
        self.reset_duration = max(
            0.1, float(rospy.get_param("~reset_duration", 1.0))
        )
        self.activate_duration = max(
            0.1, float(rospy.get_param("~activate_duration", 2.0))
        )
        self.activate_on_start = bool(rospy.get_param("~activate_on_start", False))

        self._lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._last_status = None
        self._last_status_wall_time = 0.0

        self.publisher = rospy.Publisher(
            self.command_topic,
            Robotiq2FGripper_robot_output,
            queue_size=1,
            latch=True,
        )
        self.status_subscriber = rospy.Subscriber(
            self.status_topic,
            Robotiq2FGripper_robot_input,
            self._status_callback,
            queue_size=10,
        )

        self.activate_service = rospy.Service(
            "/right_gripper/activate", Trigger, self._activate_callback
        )
        self.open_service = rospy.Service(
            "/right_gripper/open", Trigger, self._open_callback
        )
        self.close_service = rospy.Service(
            "/right_gripper/close", Trigger, self._close_callback
        )
        self.set_open_service = rospy.Service(
            "/right_gripper/set_open", SetBool, self._set_open_callback
        )
        self.status_service = rospy.Service(
            "/right_gripper/status", Trigger, self._status_service_callback
        )
        self.stop_service = rospy.Service(
            "/right_gripper/stop", Trigger, self._stop_callback
        )

        rospy.loginfo(
            "Right gripper services ready: command=%s status=%s open=%d close=%d speed=%d force=%d",
            self.command_topic,
            self.status_topic,
            self.open_position,
            self.close_position,
            self.speed,
            self.force,
        )

        if self.activate_on_start:
            rospy.logwarn("activate_on_start=true: resetting, activating, and opening gripper")
            response = self.activate()
            log = rospy.loginfo if response.success else rospy.logerr
            log("Automatic gripper activation: %s", response.message)

    def _status_callback(self, message):
        with self._status_lock:
            self._last_status = message
            self._last_status_wall_time = time.monotonic()

    def latest_status(self):
        with self._status_lock:
            return self._last_status, self._last_status_wall_time

    @staticmethod
    def make_command(active, position, speed, force, go_to=True):
        command = Robotiq2FGripper_robot_output()
        command.rACT = 1 if active else 0
        command.rGTO = 1 if active and go_to else 0
        command.rATR = 0
        command.rPR = clamp_byte(position)
        command.rSP = clamp_byte(speed)
        command.rFR = clamp_byte(force)
        return command

    def wait_for_driver(self):
        deadline = time.monotonic() + self.driver_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            status, _ = self.latest_status()
            if self.publisher.get_num_connections() > 0 and status is not None:
                return True
            rospy.sleep(0.05)
        return False

    def publish_repeated(self, command, duration=None):
        if duration is None:
            repeats = self.command_repeats
        else:
            repeats = max(1, int(round(duration / self.command_interval)))
        for _ in range(repeats):
            if rospy.is_shutdown():
                break
            self.publisher.publish(command)
            rospy.sleep(self.command_interval)

    @staticmethod
    def status_text(status):
        if status is None:
            return "no status received"
        return (
            "gACT={} gSTA={} gOBJ={} gFLT={} gPR={} gPO={} gCU={}".format(
                status.gACT,
                status.gSTA,
                status.gOBJ,
                status.gFLT,
                status.gPR,
                status.gPO,
                status.gCU,
            )
        )

    def wait_for_terminal_status(self, requested_position, after_wall_time=0.0):
        deadline = time.monotonic() + self.motion_timeout
        last = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            last, received_at = self.latest_status()
            if last is not None and received_at >= after_wall_time:
                if last.gFLT != 0:
                    return False, "gripper fault: {}".format(self.status_text(last))
                if last.gACT == 1 and last.gSTA == 3 and last.gOBJ in (2, 3):
                    if (
                        last.gOBJ == 2
                        or abs(int(last.gPO) - int(requested_position))
                        <= self.position_tolerance
                    ):
                        return True, self.status_text(last)
            rospy.sleep(0.05)
        return False, "motion timeout: {}".format(self.status_text(last))

    def command_position(self, position, label):
        if not self.wait_for_driver():
            return TriggerResponse(
                success=False,
                message="gripper driver/status unavailable on {} and {}".format(
                    self.command_topic, self.status_topic
                ),
            )
        status, _ = self.latest_status()
        if status.gFLT != 0:
            return TriggerResponse(
                success=False,
                message="refusing {} while fault is active: {}".format(
                    label, self.status_text(status)
                ),
            )
        if status.gACT != 1 or status.gSTA != 3:
            return TriggerResponse(
                success=False,
                message="gripper is not activated; call /right_gripper/activate first: {}".format(
                    self.status_text(status)
                ),
            )

        requested = clamp_byte(position)
        command_started_at = time.monotonic()
        self.publish_repeated(
            self.make_command(True, requested, self.speed, self.force)
        )
        success, details = self.wait_for_terminal_status(
            requested, after_wall_time=command_started_at
        )
        if label == "close" and success:
            status, _ = self.latest_status()
            result = "object detected" if status.gOBJ == 2 else "closed without object"
            details = "{}; {}".format(result, details)
        return TriggerResponse(success=success, message="{}: {}".format(label, details))

    def activate(self):
        with self._lock:
            if not self.wait_for_driver():
                return TriggerResponse(
                    success=False,
                    message="gripper driver/status unavailable on {} and {}".format(
                        self.command_topic, self.status_topic
                    ),
                )
            self.publish_repeated(
                self.make_command(False, 0, self.speed, self.force, go_to=False),
                self.reset_duration,
            )
            rospy.sleep(0.3)
            activation_started_at = time.monotonic()
            self.publish_repeated(
                self.make_command(
                    True, self.open_position, self.speed, self.force, go_to=True
                ),
                self.activate_duration,
            )
            success, details = self.wait_for_terminal_status(
                self.open_position, after_wall_time=activation_started_at
            )
            return TriggerResponse(
                success=success, message="activate/open: {}".format(details)
            )

    def _activate_callback(self, _request):
        return self.activate()

    def _open_callback(self, _request):
        with self._lock:
            return self.command_position(self.open_position, "open")

    def _close_callback(self, _request):
        with self._lock:
            return self.command_position(self.close_position, "close")

    def _set_open_callback(self, request):
        with self._lock:
            response = self.command_position(
                self.open_position if request.data else self.close_position,
                "open" if request.data else "close",
            )
        return SetBoolResponse(success=response.success, message=response.message)

    def _status_service_callback(self, _request):
        status, received_at = self.latest_status()
        if status is None:
            return TriggerResponse(success=False, message="no gripper status received")
        age = time.monotonic() - received_at
        healthy = status.gACT == 1 and status.gSTA == 3 and status.gFLT == 0
        return TriggerResponse(
            success=healthy,
            message="{} age={:.3f}s".format(self.status_text(status), age),
        )

    def _stop_callback(self, _request):
        with self._lock:
            status, _ = self.latest_status()
            position = status.gPO if status is not None else self.open_position
            self.publish_repeated(
                self.make_command(True, position, self.speed, self.force, go_to=False)
            )
            return TriggerResponse(
                success=True,
                message="stop command sent; {}".format(self.status_text(status)),
            )


def main():
    rospy.init_node("right_arm_gripper_control")
    RightArmGripperControl()
    rospy.spin()


if __name__ == "__main__":
    main()
