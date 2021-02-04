# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math

# You can also import scripts that you put into the folder with controller
import rcj_soccer_robot
import utils


class MyRobot(rcj_soccer_robot.RCJSoccerRobot):
    def goToPoint(self, robot_pos, point):

        pointAngle, robot_angle = self.get_angles(point, robot_pos)

        direction = utils.get_direction(pointAngle)

        if direction == 0:
            left_speed = -5
            right_speed = -5
        else:
            left_speed = -5
            right_speed = 5

        return left_speed, right_speed
    def goToPosition(self, robot_pos, point):
        point1 = {x:robot_pos["x"],y:point["y"]}
        point2 = {x:point["x"],y:robot_pos["y"]}

        pointAngle1, robot_angle = self.get_angles(point1, robot_pos)
        pointAngle2, robot_angle = self.get_angles(point2, robot_pos)

        direction = utils.get_direction(pointAngle1)
        direction = utils.get_direction(pointAngle2)
    def findGoalSlope(self, ball_pos):
        ball_distanceY = 0-ball_pos['y']
        ball_distanceX = 0.75-ball_pos['x']
        ball_slope = ball_distanceY/ball_distanceX

        return ball_slope





    def run(self):
        left_speed = 0
        right_speed = 0
        while self.robot.step(rcj_soccer_robot.TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()

                # Get the position of our robot
                robot_pos = data[self.name]
                # Get the position of the ball
                ball_pos = data['ball']
                ball_distanceX = abs(-ball_pos['x'])
                ball_distanceY = abs(robot_pos['y']-ball_pos['y'])
                goalPosition = {'x':-0.75,'y':0}

                # Get angle between the robot and the ball
                # and between the robot and the north
                ball_angle, robot_angle = self.get_angles(ball_pos, robot_pos)
                goal_angle, robot_angle = self.get_angles(goalPosition, robot_pos)



                # Compute the speed for motors
                direction = utils.get_direction(ball_angle)
                direction2 = utils.get_direction(goal_angle)

                left_speed, right_speed = self.goToPoint(robot_pos, ball_pos)

                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise
                
                if ball_pos["y"] > 0:
                    point1 = {'x':robot_pos['x'],'y':ball_pos["y"]}
                    point1_angle, robot_angle = self.get_angles(point1, robot_pos)
                    point_direction = utils.get_direction(point1_angle)
                    if robot_pos["y"] != ball_pos["y"]+0.5 and point_direction == 0:
                        left_speed = -10
                        right_speed = -10
                        print("going down")
                    else:
                        left_speed = point_direction * 10
                        right_speed = point_direction * -10
                        print("turning")
                elif ball_pos["y"] < -0:
                    point1 = {'x':robot_pos['x'],'y':ball_pos["y"]}
                    point1_angle, robot_angle = self.get_angles(point1, robot_pos)
                    point_direction = utils.get_direction(point1_angle)
                    if robot_pos["y"] != ball_pos["y"]+0.5 and point_direction == 0:
                        left_speed = -10
                        right_speed = -10
                        print("going up")
                    else:
                        left_speed = point_direction * 10
                        right_speed = point_direction * -10
                        print("turning")
                






                #Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)


my_robot = MyRobot()
my_robot.run()