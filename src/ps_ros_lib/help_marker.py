import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class help_marker:
    def __init__(self):
        self._pub_marker = rospy.Publisher('/marker', Marker, queue_size=0)

    # --------------------------------------------------------------------------
    # ==========================================================================

    def arrow(self, frame_id, ns, id, points, r, g, b, a, scale_shaft, scale_head):
        if id == 0:
            return

        if len(points) != 2:
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.ARROW
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.orientation.w = 1.0
        msg_marker.scale.x = scale_shaft
        msg_marker.scale.y = scale_head
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        msg_marker.points = points
        # -----
        self._pub_marker.publish(msg_marker)

    # --------------------------------------------------------------------------
    # ==========================================================================

    def cube(self, frame_id, ns, id, position, orientation, r, g, b, a, scale_x, scale_y, scale_z):
        if id == 0:
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.CUBE
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.position = position
        msg_marker.pose.orientation = orientation
        msg_marker.scale.x = scale_x
        msg_marker.scale.y = scale_y
        msg_marker.scale.z = scale_z
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        self._pub_marker.publish(msg_marker)

    def sphere(self, frame_id, ns, id, position, orientation, r, g, b, a, scale_x, scale_y, scale_z):
        if id == 0:
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.SPHERE
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.position = position
        msg_marker.pose.orientation = orientation
        msg_marker.scale.x = scale_x
        msg_marker.scale.y = scale_y
        msg_marker.scale.z = scale_z
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        self._pub_marker.publish(msg_marker)

    def cylinder(self, frame_id, ns, id, position, orientation, r, g, b, a, scale_x, scale_y, scale_z):
        if id == 0:
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.CYLINDER
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.position = position
        msg_marker.pose.orientation = orientation
        msg_marker.scale.x = scale_x
        msg_marker.scale.y = scale_y
        msg_marker.scale.z = scale_z
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        self._pub_marker.publish(msg_marker)

    # --------------------------------------------------------------------------
    # ==========================================================================

    def line_strip(self, frame_id, ns, id, points, r, g, b, a, scale):
        if id == 0:
            return

        if len(points) < 2:
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.LINE_STRIP
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.orientation.w = 1.0
        msg_marker.scale.x = scale
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        msg_marker.points = points
        # -----
        self._pub_marker.publish(msg_marker)

    def line_list(self, frame_id, ns, id, points, r, g, b, a, scale):
        if id == 0:
            return

        if len(points) < 2:
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.LINE_LIST
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.orientation.w = 1.0
        msg_marker.scale.x = scale
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        msg_marker.points = points
        # -----
        self._pub_marker.publish(msg_marker)

    def cube_list(self, frame_id, ns, id, positions, r, g, b, a, scale_x, scale_y, scale_z):
        if id == 0:
            return

        if len(positions < 1):
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.CUBE_LIST
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.orientation.w = 1.0
        msg_marker.scale.x = scale_x
        msg_marker.scale.y = scale_y
        msg_marker.scale.z = scale_z
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        msg_marker.points = positions
        # -----
        self._pub_marker.publish(msg_marker)

    def sphere_list(self, frame_id, ns, id, positions, r, g, b, a, scale_x, scale_y, scale_z):
        if id == 0:
            return

        if len(positions < 1):
            return

        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a

        msg_marker = Marker()
        # -----
        msg_marker.header.frame_id = frame_id
        msg_marker.header.stamp = rospy.Time.now()
        msg_marker.ns = ns
        msg_marker.id = abs(id)
        # -----
        msg_marker.type = Marker.SPHERE_LIST
        msg_marker.action = Marker.ADD if id > 0 else Marker.DELETE
        # -----
        msg_marker.pose.orientation.w = 1.0
        msg_marker.scale.x = scale_x
        msg_marker.scale.y = scale_y
        msg_marker.scale.z = scale_z
        msg_marker.color = color
        msg_marker.frame_locked = True
        # -----
        msg_marker.points = positions
        # -----
        self._pub_marker.publish(msg_marker)
