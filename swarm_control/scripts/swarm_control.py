#!/usr/bin/env python
# coding=utf8

import math
import numpy as np
import sys
import yaml
from numpy import format_parser
import copy

import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import *
from virt_formation import create_virtual_structure
from swarm_msgs.msg import FormationParam, CommonParams
from swarm_msgs.srv import *

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Global params
tag = "drone_"
max_vel = 1.0
use_yaml = False

# Параметры планировщика
use_field = True
allow_z_field = False
r_safe = 1.2  # дальность действия поля отталкивания
r_kor = 0.3  # коридор нулевых сил
force_rep = 0.8  # коэффициент ф-ии отталкивания

size_of_drone = 0

param_path = "../param/config.yaml"

drone_offset_list = list()  # [name :str, offset :Point, prevPoint: PoseStamped]
markers_goal = MarkerArray()
markers_goal_text = MarkerArray()
markers_lerp = MarkerArray()
markers_lerp_text = MarkerArray()

goal_common_msgs = PoseStamped()
state_init_flag = False


formation = FormationParam()


# tools
def get_yaw_from_quat(quat):
    roll, pitch, yaw = euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    return yaw

def get_quat_from_yaw(yaw):
    quat = Quaternion()

    res_quat = quaternion_from_euler(0.,0.,yaw)
    quat.x = res_quat[0]
    quat.y = res_quat[1]
    quat.z = res_quat[2]
    quat.w = res_quat[3]
    return quat

### Ros callback
###



def goal_clb(data):
    """
    Get common goal pose
    :type data:Goal
    :return:
    """
    global goal_common_msgs, state_init_flag

    goal_common_msgs = data

    if not state_init_flag:
        reset_pose()
        state_init_flag = True

def set_formation_srv(req):
    """
    Service callback of set formation
    :param req:
    :return:
    """
    global formation
    print "get form", req
    if req.formation.tag != "":
        formation = req.formation
        load_params(req.formation)
        reset_pose()
        print ("DONE: change formation")

        return True
    else:
        print ("ERROR: change formation")

        return False


def set_field_srv(req):
    """
    Service callback of set field
    :param req:
    :return:
    """
    global use_field, allow_z_field, r_safe, force_rep
    print "set field param", req
    try:
        allow_z_field = req.field.allow_z_field
        use_field = req.field.use_field
        force_rep = req.field.force_rep
        r_safe = req.field.r_safe
        print ("DONE: change field param")
        return True
    except:
        return False



def set_max_vel(req):
    """set max velocity of group"""

    global max_vel
    if req.data >= 0.:
        max_vel = req.data
        print "set max vel", max_vel
        return True
    else:
        print "set max vel ERROR"
        return False


### methods

def rep_force(dist_to_obs):
    # вида 1/х
    return force_rep / dist_to_obs - force_rep / r_safe

def repel_from_near(lerp_point, drone_num, course, radius=1.0):
    """
    Отталкиваемся от дронов если они возши в заданный радиус
    :param name:
    :param list:
    :return:
    """
    repel = [0.0, 0.0, 0.0]

    for i in range(len(drone_offset_list)):
        if drone_offset_list[i][0] != drone_offset_list[drone_num][0]:
            dist, vec = get_distance(lerp_point, drone_offset_list[i][2])
            if dist < radius:
                if abs(course) <= np.deg2rad(90):
                    dir = 1.0
                else:
                    dir = -1.0
                # print("%s -> %s. dist:%s, vec:%s" % (drone_offset_list[drone_num][0],drone_offset_list[i][0],dist, vec))
                repel[0] += vec[0] * rep_force(dist)
                repel[1] += vec[1] * rep_force(dist)
                if vec[2] == 0.:
                    repel[2] += max_vel * rep_force(dist) * dir
                else:
                    repel[2] += vec[2] * rep_force(dist) * dir
    return repel

def load_params_from_path(path):
    """
    Load data of drones from yaml
    :type path: str
    :return:
    """
    global size_of_drone, drone_offset_list, markers_goal, goal_common_msgs, state_init_flag

    try:
        file = open(path, "r")
        data = str(file.read())
        file.close()
    except:
        rospy.logerr("Param error path")
        sys.exit()


    drone_offset_list = list()
    markers_goal.markers = list()
    markers_lerp.markers = list()
    markers_lerp_text.markers = list()
    markers_goal_text.markers = list()

    # set data "name" and "offset" to list
    size_of_drone = yaml.load(data)['size']
    i = 0
    while len(drone_offset_list) < size_of_drone:
        try:
            offset_str = yaml.load(data)[tag + str(i)]
            offset_data = Point()
            offset_data.x = offset_str['x']
            offset_data.y = offset_str['y']
            offset_data.z = offset_str['z']
            new_goal = PoseStamped()
            new_goal.header.frame_id = "map"
            new_goal.header.stamp = rospy.Time.now()
            new_goal.pose.position.x = offset_data.x
            new_goal.pose.position.y = offset_data.y
            new_goal.pose.position.z = offset_data.z
            new_goal.pose.orientation.w = 1.

            if state_init_flag:
                new_goal.pose.position.x += goal_common_msgs.pose.position.x
                new_goal.pose.position.y += goal_common_msgs.pose.position.y
                new_goal.pose.position.z += goal_common_msgs.pose.position.z

            drone_offset_list.append([tag + str(i), offset_data, new_goal])
            markers_goal.markers.append(Marker())
            markers_goal_text.markers.append(Marker())
            markers_lerp.markers.append(Marker())
            markers_lerp_text.markers.append(Marker())
        except:
            rospy.logerr("%s:param not found" % (tag + str(i)))
        i += 1

def load_params(form):
    """
    Load data of drones from yaml
    :type formation: FormationParam
    :return:
    """
    global size_of_drone, drone_offset_list, markers_goal, state_init_flag, goal_common_msgs

    del drone_offset_list[:]
    del markers_goal.markers[:]
    del markers_lerp.markers[:]
    del markers_lerp_text.markers[:]
    del markers_goal_text.markers[:]


    size_of_drone = 0
    struct, group_size = create_virtual_structure(form)

    size_of_drone = form.count
    # drone_offset_list [name :str, offset :Point, prevPoint: Goal]


    for i in range(size_of_drone):
        # try:
        offset_data = Point()
        offset_data.x = struct[i][0]
        offset_data.y = struct[i][1]
        offset_data.z = 0.
        new_goal = PoseStamped()
        new_goal.pose.position.x = offset_data.x
        new_goal.pose.position.y = offset_data.y
        new_goal.pose.position.z = offset_data.z

        drone_offset_list.append([form.tag + "_" + str(i), offset_data, new_goal])

        markers_goal.markers.append(Marker())
        markers_goal_text.markers.append(Marker())
        markers_lerp.markers.append(Marker())
        markers_lerp_text.markers.append(Marker())


def reset_pose():
    """
    Reset formation position

    :return:
    """
    global drone_offset_list, state_init_flag, goal_common_msgs

    for i in range(len(drone_offset_list)):
        offset = drone_offset_list[i][1]
        goal = drone_offset_list[i][2]
        offset_rot = rotate_point(offset, -get_yaw_from_quat(goal_common_msgs.pose.orientation))

        goal.pose.position.x = goal_common_msgs.pose.position.x + offset_rot.x
        goal.pose.position.y = goal_common_msgs.pose.position.y + offset_rot.y
        goal.pose.position.z = goal_common_msgs.pose.position.z + offset_rot.z
        drone_offset_list[i][2] = goal


### tools
def setup_market(name, point, id, colorRGBA, text_flag=False):
    """
    Настройка маркера для отображения в rviz
    :type point: Point
    :param point:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "marker"
    marker.id = id
    marker.action = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = colorRGBA[0]
    marker.color.g = colorRGBA[1]
    marker.color.b = colorRGBA[2]
    marker.color.a = colorRGBA[3]
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    if text_flag:
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = str(id)
        marker.pose.position.z = point.z + (marker.scale.z)
    else:
        marker.type = Marker.SPHERE
        marker.pose.position.z = point.z

    return marker

def rotate_vect(a, b, rot):
    """
    Поворачиваем b относительно a на угол rot
    :type a: list
    :type b: list
    :type rot: float
    :type dist: float
    :return: возвращаем точку повёрнутую на нужный угол

    """
    rotate = np.array([[math.cos(-rot), -math.sin(-rot)],
                       [math.sin(-rot), math.cos(-rot)]])

    pos = np.array([[a[0]],
                    [a[1]]])
    val = np.dot(rotate, pos)
    val[0] += b[0]
    val[1] += b[1]
    return [val[0][0], val[1][0], a[0]]

def publish_empty_markers():
    # publish to RVIZ

    pub_markers_goal.publish(MarkerArray())
    pub_markers_goal_text.publish(MarkerArray())
    pub_markers_goal_lerp.publish(MarkerArray())
    pub_markers_goal_lerp_text.publish(MarkerArray())

def rotate_point(point, rot):
    """

    :param point:
    :param cours:
    :return:
    """

    rotate = np.array([[math.cos(-rot), -math.sin(-rot)],
                       [math.sin(-rot), math.cos(-rot)]])

    pos = np.array([[point.x],
                    [point.y]])
    val = np.dot(rotate, pos)

    newPoint = Point()
    newPoint.x = val[0][0]
    newPoint.y = val[1][0]
    newPoint.z = point.z
    return newPoint

def speed_limit_goal(new_goal, prev_goal, dt, max_speed):
    """

    :type prev_goal: Goal
    :type _goal: Goal
    :param dt: float
    :param max_speed: float
    :return: limit goal
    """
    limit_goal = PoseStamped()
    limit_goal.header = prev_goal.header
    limit_goal.header.stamp = rospy.Time.now()


    vec = [new_goal.pose.position.x - prev_goal.pose.position.x,
           new_goal.pose.position.y - prev_goal.pose.position.y,
           new_goal.pose.position.z - prev_goal.pose.position.z]

    dist = np.linalg.norm(vec)

    if dist > max_vel * dt:
        res = vec / dist * max_speed * dt
        limit_goal.pose.position.x = res[0] + prev_goal.pose.position.x
        limit_goal.pose.position.y = res[1] + prev_goal.pose.position.y
        limit_goal.pose.position.z = res[2] + prev_goal.pose.position.z
        return limit_goal

    else:
        return new_goal

def speed_limit_vec(new_goal, prev_goal, dt, max_speed):
    """

    :type prev_goal: Goal
    :type _goal: Goal
    :param dt: float
    :param max_speed: float
    :return: limit goal
    """
    limit_goal = [0., 0., 0.]

    vec = [new_goal[0] - prev_goal[0],
           new_goal[1] - prev_goal[1],
           new_goal[2] - prev_goal[2]]

    dist = np.linalg.norm(vec)

    if dist > max_vel * dt:
        res = vec / dist * max_speed * dt
        limit_goal[0] = res[0] + prev_goal[0]
        limit_goal[1] = res[1] + prev_goal[1]
        limit_goal[2] = res[2] + prev_goal[2]
        return limit_goal

    else:
        return new_goal

def get_drone_pose(common_goal, offset):
    new_pose = PoseStamped()
    return new_pose

def get_distance(a, b):
    """
    Distance between 2 GOAL
    :type a: PoseStamped
    :type b: PoseStamped
    :return: float
    """
    vec = [a.pose.position.x - b.pose.position.x,
           a.pose.position.y - b.pose.position.y,
           a.pose.position.z - b.pose.position.z]

    return np.linalg.norm(vec), vec

def get_course(target_goal, current_goal):
    """
    :type target_goal: PoseStamped
    :type current_goal: PoseStamped
    :return:
    """
    vec = [target_goal.pose.position.x - current_goal.pose.position.x,
           target_goal.pose.position.y - current_goal.pose.position.y]

    res = math.atan2(vec[1], vec[0])
    return res

def rotate_goal(a, b, rot):
    """
    Поворачиваем b относительно a на угол rot
    :type a: Point
    :type b: Point
    :type rot: float
    :type dit: float
    :param a: common goal pose
    :param b: drone_0_offset of drone goal
    :param rot: rotate of current pose
    :return: возвращаем точку повёрнутую на нужный угол

    """
    new_goal = PoseStamped()
    new_goal.header.frame_id = "map"
    new_goal.header.stamp = rospy.Time.now()

    rotate = np.array([[math.cos(rot), -math.sin(rot)],
                       [math.sin(rot), math.cos(rot)]])

    pos = np.array([[b.x],
                    [b.y]])

    res = np.dot(rotate, pos)
    res[0] += a.x
    res[1] += a.y

    new_goal.pose.position.x = float(res[0])
    new_goal.pose.position.y = float(res[1])
    new_goal.pose.position.z = float(a.z + b.z)
    new_goal.pose.orientation = get_quat_from_yaw(rot)
    return new_goal


if __name__ == '__main__':
    rospy.init_node("swarm_contol_node")
    print("swarm_control: init")

    # get param
    use_yaml = rospy.get_param("~use_yaml", use_yaml)
    param_path = rospy.get_param("~path", param_path)
    r_safe = rospy.get_param("~r_safe", r_safe)
    max_vel = rospy.get_param("~max_vel", max_vel)
    use_field = rospy.get_param("~use_field", use_field)
    allow_z_field = rospy.get_param("~allow_z_field", allow_z_field)

    if use_yaml:
        print("yaml path:",param_path)
        if param_path == "":
            rospy.logerr("Param not set")
            sys.exit()

        load_params_from_path(param_path)
    else:
        # test
        formation.type = FormationParam.KLIN
        formation.count = 3
        formation.distance = 1.0
        formation.tag = "drone"
        load_params(formation)


    rospy.Subscriber("/goal", PoseStamped, goal_clb)
    srv_form = rospy.Service('swarm_contol/set_fotmation', FormationSrv, set_formation_srv)
    srv_field = rospy.Service('swarm_contol/set_field', FieldSrv, set_field_srv)
    srv_vel = rospy.Service('swarm_contol/set_max_velocity', FloatSrv, set_max_vel)

    pub_params = rospy.Publisher("swarm_contol/state", CommonParams, queue_size=10)

    # pub markers
    pub_markers_goal = rospy.Publisher("/goal_markers", MarkerArray, queue_size=10)
    pub_markers_goal_text = rospy.Publisher("/goal_markers/text", MarkerArray, queue_size=10)
    pub_markers_goal_lerp = rospy.Publisher("/goal_markers/lerp", MarkerArray, queue_size=10)
    pub_markers_goal_lerp_text = rospy.Publisher("/goal_markers/text", MarkerArray, queue_size=10)

    publish_empty_markers()

    rate = rospy.Rate(10)

    _drone_goal_msgs = PoseStamped()
    _drone_goal_msgs.header.frame_id = "map"

    old_time = rospy.get_time()

    common_params = CommonParams()
    try:
        while not rospy.is_shutdown():
            if not state_init_flag:
                rate.sleep()
                continue

            dt = rospy.get_time() - old_time
            # try:
                # Перебираем массив всех дронов
            for i in range(len(drone_offset_list)):
                _drone_goal_msgs.header.stamp = rospy.Time.now()

                name_of_drone = drone_offset_list[i][0]
                # получаем целевую точку куда нужно двигаться
                _drone_goal_msgs = rotate_goal(goal_common_msgs.pose.position, drone_offset_list[i][1],
                                               get_yaw_from_quat(goal_common_msgs.pose.orientation))
                _drone_goal_msgs.pose.orientation = goal_common_msgs.pose.orientation

                # Интерполируем точку от текущей до целевой
                if state_init_flag:
                    lerp_goal = speed_limit_goal(_drone_goal_msgs, drone_offset_list[i][2], dt, max_vel)
                    lerp_goal.pose.orientation = goal_common_msgs.pose.orientation
                else:
                    lerp_goal = _drone_goal_msgs

                # get course
                course = get_course(_drone_goal_msgs, lerp_goal)

                # отталкиваемся от соседей
                if use_field:
                    repel_vec = speed_limit_vec(repel_from_near(lerp_goal, i, course, r_safe), [0, 0, 0], dt, max_vel)
                    repel_rot_vec = rotate_vect(repel_vec, [0, 0, 0], np.deg2rad(-25))
                    lerp_goal.pose.position.x += repel_rot_vec[0]
                    lerp_goal.pose.position.y += repel_rot_vec[1]
                    if allow_z_field:
                        lerp_goal.pose.position.z += repel_rot_vec[2]
                # указываем интерполированную точку как предыдущую
                drone_offset_list[i][2] = lerp_goal

                markers_lerp.markers[i] = setup_market(name_of_drone,
                                                       lerp_goal.pose.position,
                                                       i, [0.0, 0.0, 1.0, 0.7])
                markers_lerp_text.markers[i] = setup_market(name_of_drone,
                                                       lerp_goal.pose.position,
                                                       i, [1.0, 0.0, 0.0, 1.0], text_flag=True)

                markers_goal.markers[i] = setup_market(name_of_drone,
                                                       _drone_goal_msgs.pose.position,
                                                       i, [0.0, 1.0, 0.0, 1.0])

                markers_goal_text.markers[i] = setup_market(name_of_drone,
                                                            _drone_goal_msgs.pose.position,
                                                            i, [0.0, 0.3, 0.0, 1.0], text_flag=True)
                rospy.Publisher(name_of_drone + "/goal", PoseStamped, queue_size=1).publish(lerp_goal)
            # except:
            #     print "error len"


            pub_markers_goal.publish(markers_goal)
            pub_markers_goal_text.publish(markers_goal_text)
            pub_markers_goal_lerp.publish(markers_lerp)
            pub_markers_goal_lerp_text.publish(markers_lerp_text)

            # pub params
            common_params.formation = formation
            common_params.max_vel = max_vel
            common_params.field.allow_z_field = allow_z_field
            common_params.field.use_field = use_field
            common_params.field.force_rep = force_rep
            common_params.field.r_safe = r_safe

            pub_params.publish(common_params)

            old_time = rospy.get_time()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
