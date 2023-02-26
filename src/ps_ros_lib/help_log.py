import rospy
import datetime
from ps_ros_lib.msg import log

class help_log:
    def __init__(self):
        self._pub_log = rospy.Publisher('/log', log, queue_size=0)

        self._node_namespace = rospy.get_namespace()
        self._node_name = rospy.get_name()

    # --------------------------------------------------------------------------
    # ==========================================================================

    def log(self, header, body):
        msg_log = log()
        msg_log.node_namespace = self._node_namespace
        msg_log.node_name = self._node_name
        msg_log.log_datetime = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        msg_log.log_header = header
        msg_log.log_body = body
        self._pub_log.publish(msg_log)

    # --------------------------------------------------------------------------
    # ==========================================================================

    def debug(self, body):
        self.log('DEBUG', body)

    def info(self, body):
        self.log('INFO', body)

    def warn(self, body):
        self.log('WARN', body)

    def error(self, body):
        self.log('ERROR', body)

    def fatal(self, body):
        self.log('FATAL', body)
