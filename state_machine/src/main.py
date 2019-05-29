#!/usr/bin/python
# -*-coding:utf-8-*-
import rospy
from smartcar_msgs.msg import State, CloudInterface, UserInterface, CrossLock


class VehicleState:
    INTERRUPT_BY_CLOUD = False
    INTERRUPT_BY_OBSTACLE = False
    INTERRUPT_BY_USER = False
    INTERRUPT_BY_CROSS = False

    def state_ok(self):
        if not self.INTERRUPT_BY_CLOUD and not self.INTERRUPT_BY_OBSTACLE and not self.INTERRUPT_BY_USER and not self.INTERRUPT_BY_CROSS:
            return True
        return False


class APP:
    def __init__(self):
        rospy.init_node("state_machine", log_level=rospy.INFO)
        self.state = VehicleState()

    def _add_sub(self):
        rospy.Subscriber("CloudInterface", CloudInterface, self.cloudInterface_cb)
        rospy.Subscriber("UserInterface", UserInterface, self.userInterface_cb)
        rospy.Subscriber("CrossLock", CrossLock, self.crossLock_cb)

    def _add_pub(self):
        self.state_pub = rospy.Publisher("SmartcarState", State, queue_size=1)
        self.cloud_lock_pub = rospy.Publisher(
            "ToCloudLock", CrossLock, queue_size=1)

    def cloudInterface_cb(self, msg):
        assert isinstance(msg, CloudInterface)
        if msg.data == CloudInterface.NORMAL_RUN:
            self.state.INTERRUPT_BY_CLOUD = False
        elif msg.data == CloudInterface.NORMAL_PAUSE:
            self.state.INTERRUPT_BY_CLOUD = True
        elif msg.data == CloudInterface.CROSS_PASS:
            self.state.INTERRUPT_BY_CROSS = False
        elif msg.data == CloudInterface.CROSS_PAUSE:
            self.state.INTERRUPT_BY_CROSS = True
        self.check_state()

    def userInterface_cb(self, msg):
        assert isinstance(msg, UserInterface)
        if msg.data == UserInterface.NORMAL_PAUSE:
            self.state.INTERRUPT_BY_USER = True
        elif msg.data == UserInterface.NORMAL_RUN:
            self.state.INTERRUPT_BY_USER = False
        self.check_state()

    def crossLock_cb(self, msg):
        assert isinstance(msg, CrossLock)
        if msg.type == CrossLock.REQUEST_LOCK:
            self.state.INTERRUPT_BY_CROSS = True
            self.check_state()
            # 请求云端锁
            # print("[state-machine] now request lock from cloud")
            self.request_cross_lock(msg)
        elif msg.type == CrossLock.RELEASE_LOCK:
            self.state.INTERRUPT_BY_CROSS = False
            self.check_state()
            # 释放云端锁
            # print("[state-machine] now release lock to cloud")
            self.release_cross_lock(msg)

    def request_cross_lock(self, msg):
        self.cloud_lock_pub.publish(msg)

    def release_cross_lock(self, msg):
        self.cloud_lock_pub.publish(msg)

    def check_state(self):
        state = State()
        if self.state.state_ok():
            state.main_state = State.RUN
        else:
            state.main_state = State.PAUSE
        self.state_pub.publish(state)

    def run(self):
        self._add_sub()
        self._add_pub()
        rospy.spin()

    def shutdown(self):
        state = State()
        state.main_state = State.PAUSE
        self.state_pub.publish(state)


if __name__ == '__main__':
    app = APP()
    try:
        app.run()
    except rospy.ROSInterruptException, e:
        app.shutdown()
        rospy.loginfo(e)
