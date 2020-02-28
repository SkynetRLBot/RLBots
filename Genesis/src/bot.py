import math
import time as t

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3


class MyBot(BaseAgent):

    def initialize_agent(self):
        # This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.is_zombie = False

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:

        ball_location = Vec3(packet.game_ball.physics.location)
        ball_velocity = Vec3(packet.game_ball.physics.velocity)

        if not self.is_zombie:
            other_car = packet.game_cars[not self.index]

        team = packet.game_cars[self.index].team
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)
  
        car_to_ball = ball_location - car_location
        car_ball_distance = distance2D(ball_location, car_location)
        # Find the direction of our car using the Orientation class
        car_orientation = Orientation(my_car.physics.rotation)
        car_direction = car_orientation.forward

        ball_future = Vec3(clamp(ball_location.x + ball_velocity.x * (car_ball_distance / 2000),-4096,4096),clamp(ball_location.y + ball_velocity.y * (car_ball_distance / 1500),-5020,5020),max(ball_location.z + ball_velocity.z * (car_ball_distance / 1500),-ball_location.z))
        # print(f"car_ball_distance: {car_ball_distance}")
        # print(f"ball_future: {ball_future}")
        # print(f"ball_location: {ball_location}")
        car_to_future = ball_future - car_location
        car_ball_future = distance2D(ball_future, car_location)

        enemy_goal = Vec3(0, -5120, 0) if team == 1 else Vec3(0, 5120, 0)
        team_goal = Vec3(0, -5120, 0) if not team == 1 else Vec3(0, 5120, 0)
        car_to_own_goal = team_goal - car_location
        ball_to_goal = distance2D(ball_future, team_goal)

        #print(ball_to_goal)

        steer_correction_radians = find_correction(car_direction, car_to_future)
        pi = math.pi
        try:
            other_car_d2b = distance2D(ball_future, Vec3(other_car.physics.location))
        except:
            pass

        #print(max((magnitude2D(Vec3(packet.game_ball.physics.velocity))/1000),1))

        #print(team_cars)
        try:
            if other_car_d2b < car_ball_future and ball_to_goal > 2500:
                steer_correction_radians = find_correction(car_direction, car_to_own_goal)
                heading_back = True
            else:
                heading_back = False
        except:
            pass

        distance_to_use = -200 if self.team == 0 else 200

        if ball_future.y < car_location.y - distance_to_use and ball_to_goal > 2500:
            steer_correction_radians = find_correction(car_direction, car_to_own_goal)
            heading_back = True
        else:
            heading_back = False

        if steer_correction_radians > 0.1:
            # Positive radians in the unit circle is a turn to the left.
            turn = -1.0  # Negative value for a turn to the left.
            action_display = "turn left"
        elif steer_correction_radians < -0.1:
            turn = 1.0
            action_display = "turn right"
        else:
            turn = 0
            action_display = "Go Forward"

        if abs(steer_correction_radians) > .7 and abs(steer_correction_radians) < (pi - 0.8):
            self.controller_state.handbrake = 1
        else:
            self.controller_state.handbrake = 0

        if not heading_back:
            speed = distance2D(ball_future, car_location) / 500 if ball_location.x != 0 and ball_location.y != 0 else 1
            if (speed >= 2 and abs(steer_correction_radians) < 0.1) or speed == 1:
                self.controller_state.boost = 1
            else:
                self.controller_state.boost = 0
            speed = min(speed, 1)
            if abs(steer_correction_radians) > (pi * 4 / 8):
                speed = max(((speed * -1) * car_ball_future/100) / 2,-1) if not heading_back else -1
                turn = turn * -1
        else:
            speed = min(distance2D(team_goal, car_location) // 500, 1)
        #print(speed)
        #print(speed)

        if 500 > ball_future.z > 200 and car_ball_distance < 500:
            self.controller_state.jump = 1
        else:
            self.controller_state.jump = 0

        self.controller_state.throttle = speed
        self.controller_state.steer = turn

        draw_debug(self.renderer, my_car, ball_future, action_display)

        return self.controller_state


def find_correction(current: Vec3, ideal: Vec3) -> float:

    # The in-game axes are left handed, so use -x
    current_in_radians = math.atan2(current.y, -current.x)
    ideal_in_radians = math.atan2(ideal.y, -ideal.x)

    diff = ideal_in_radians - current_in_radians

    # Make sure that diff is between -pi and +pi.
    if abs(diff) > math.pi:
        if diff < 0:
            diff += 2 * math.pi
        else:
            diff -= 2 * math.pi

    return diff


def draw_debug(renderer, car, ball, action_display):
    renderer.begin_rendering()
    # draw a line from the car to the ball
    renderer.draw_line_3d(car.physics.location, ball, renderer.white())
    #renderer.draw_polyline_3d(ball.predict.pos[:120:10], renderer.pink())
    # print the action that the bot is taking
    renderer.draw_string_3d(car.physics.location, 2, 2, action_display, renderer.white())
    renderer.end_rendering()


def distance2D(target: Vec3, car: Vec3):
    return math.sqrt((target.x - car.x)**2 + (target.y - car.y)**2)


def magnitude2D(vector: Vec3):
    return math.sqrt(vector.x**2 + vector.y**2)

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)