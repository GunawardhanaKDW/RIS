#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

#robot state variables
position_ = Point()
yaw_ = 0
#machine state
state_ = 0
#goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0

#task two start
start_position_ = 0
end_position_ = 0

#parameters
yaw_precision_ = math.pi /90 # +/- degree allowed
dist_precision_ = 0.3

free_lane = 0
park = 0
step = 0
door = 0
door_check = 0
task = 1
angle_change = 1
state_description = ''
active_ = False
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'small_left_trun',
    4: 'bend',
}

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right': min(msg.ranges[-90],10),
		'fright': min(min(msg.ranges[-20:-1]),10),	
		'front': min(msg.ranges[0],10),
		'fleft': min(msg.ranges[45],10),
		'left': min(msg.ranges[90],10)
    }
    
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_,task, start_position_, end_position_, door, park
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    global state_description 
    
    d = 0.3
    if task == 3 or free_lane == 1:
        pass
    elif regions['left'] > 9 and regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        if regions['fleft'] == 10:
            print 'maze finished'
            task = 2
    elif regions['right'] > 9 and task == 2:
        print 'gap %s' %(start_position_ - 1.15)
        door_range = start_position_ - end_position_
        y = position_.y
        if y > start_position_ + 0.9 and y < start_position_ + 1.1:
            print '################Second Door###################'
            door = 2
        elif y > start_position_ + 0.49 and y < start_position_ + 0.51 :
            print '################First Door###################'
            door = 1
        elif y > start_position_ + 1.49 and y < start_position_ + 1.51:
            print '################Third Door###################'
            door = 3

    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        
        if regions ['right'] > 0.3:
            print '################Bend############'
            change_state(4)
        else:
            change_state(0)
        #elif regions ['left'] > 
    elif regions['front'] < 0.2 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(3)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
    rospy.loginfo(regions)

#########################################################################################################
def clbk_odem(msg):
    global position_
    global yaw_

    #position
    position_ = msg.pose.pose.position

    #yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
def change_door_check(door_check_):
    global door_check
    door_check = door_check_
    print 'Door check changed to [%s]' % door_check

def fix_yaw(des_pos):
    global yaw_, pub_, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.15 if err_yaw > 0 else -0.15

    pub_.publish(twist_msg)

    if math.fabs(err_yaw) <= yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_door_check(1)

def go_fo_straight_ahead(des_pos):
    global yaw_, pub_, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.0
        pub_.publish(twist_msg)
    else:
        print 'Position error: [%s]' % err_pos
        change_door_check(2)

    #state chanfe conitions
    if math.fabs(err_yaw) > yaw_precision_:
        print 'Yaw error: [%s]' % err_yaw
        change_door_check(0)



def done():
    
    global desired_position_, step, door, park,angle_change
    park = 1
    if door == 1:                                     
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.2
        pub_.publish(msg)
    elif door == 3:                                    
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = -0.2
        pub_.publish(msg)
    
    elif door == 2:
        if angle_change == 1:
            msg = Twist()
            msg.linear.x = 0.2
            msg.angular.z = 0.2
            pub_.publish(msg)
        
        elif angle_change == 2:
            msg = Twist()
            msg.linear.x = 0.2
            msg.angular.z = -0.2
            pub_.publish(msg)


        
    elif door == 0: 
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        pub_.publish(msg)
    else:
        rospy.logerr('door')

    
#####################################################################################################
def left_turn():
    msg = Twist()
    msg.linear.x = 0.02
    msg.angular.z = 0.02
    return msg

def find_wall():
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def small_left_trun():
    msg = Twist()
    msg.angular.z = 0.1
    return msg

def follow_the_wall():
    global regions_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.05
    return msg

def bend():
    global regions_
    msg = Twist()
    msg.angular.z = -0.3
    #msg.linear.x = 0.5
    return msg

def new_location():
    global desired_position_
    

def main():
    global pub_, active_, state_description, task, state_, door_check, desired_position_,start_position_, end_position_, door, angle_change, free_lane
    che = 0
    rospy.init_node('robot_motion')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odem)

    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():   
        
        if task == 1:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
                pass
            elif state_ == 3:
                msg = small_left_trun()
            elif state_ == 4:
                msg = bend()
            else:
                rospy.logerr('Unknown state!')
        
            rospy.loginfo(state_description)
        

        if task == 2:
            if che == 0:
                print '^^^^^^^^^^^^^^^^^^^^^^^^^^task %s ^^^^^^^^^^^^^^^^' % task
                desired_position_.x = position_.x 
                desired_position_.y = position_.y + 2
                start_position_ = position_.y
                end_position_ = desired_position_.y
                print 'x --------- [%s]' % desired_position_.x
                print 'y --------- [%s]' % desired_position_.y
                door_check = 0
                che = 1

            if door_check == 0:
                fix_yaw(desired_position_)
            elif door_check == 1:
                go_fo_straight_ahead(desired_position_)
            elif door_check == 2:
                if free_lane == 0:
                    print 'Door area finish'
                    che = 0
                    free_lane = 1
                    if door == 3:
                        desired_position_.y = position_.y + 3
                    else:
                        desired_position_.y = position_.y + 2
                    desired_position_.x = position_.x
                    
                elif free_lane == 1:
                    print 'Free 2m area finish'
                    done()
                    task = 3
                    che = 0
            else:
                rospy.logerr('Unknown state!')
                pass

        if task == 3:
            msg = Twist()
            print 'x~ --------- [%s]' % msg.linear.x
            print 'z~ --------- [%s]' % msg.angular.z
            x_position = float("%0.1f" % (position_.x))
            y_position = float("%0.1f" % (position_.y))
            if che == 0:
                print '^^^^^^^^^^^^^^^^^^^^^^^^^^task %s ^^^^^^^^^^^^^^^^' % task
                start_position_ = float("%0.1f" % (position_.x))
                end_position_ = float("%0.1f" % (position_.y))
                che = 1
            if door == 2:
                third_position = start_position_ - 1.000000 # X
                fourth_position = end_position_ + 2 # Y

                

            print '@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@'    
            print '^^^^^^^^^^^^^^^^^^^^^^^^^^Door %s ^^^^^^^^^^^^^^^^' % door
            print 'start_position_             --------- [%s]' % start_position_
            print 'end_position_               --------- [%s]' % end_position_
            print 'x                           --------- [%s]' % x_position
            print 'y                           --------- [%s]' % y_position
            if door == 2:
                
                print 'third_position                     --------- [%s]' % third_position
                print 'fourth_position                     --------- [%s]' % fourth_position
                print 'change_angle                     --------- [%s]' % angle_change
            print '@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@' 
            if door == 1 and x_position < start_position_ - 1 and end_position_ == y_position:
                print '----------------End--------------'
                door = 0
            elif door == 3 and x_position> start_position_ + 1 and end_position_ == y_position:
                print '----------------End--------------'
                door = 0
            elif door == 2 and third_position == x_position and angle_change == 1:
                print '----------------Right_conor--------------'
                angle_change = 2    
            elif door == 2 and str(fourth_position) == str(y_position) and angle_change == 2:
                print '----------------End--------------'
                door = 0
            else:
                print 'error park'
            done()
             
        rate.sleep()
        pub_.publish(msg)
if __name__ == '__main__':
    main()

