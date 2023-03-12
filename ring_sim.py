import math
import pygame
import random

class Particle:
    def __init__(self, x, y, vx, vy, mass, radius, color, friction):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.mass = mass
        self.radius = radius
        self.color = color
        self.friction = friction

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_vx(self):
        return self.vx

    def get_vy(self):
        return self.vy

    def update_position(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt

    def collide_with(self, other, moveSelf=True):
        if isinstance(other, Ground) or isinstance(other, Obstacle):
            return
        
        dx = other.get_x() - self.get_x()
        dy = other.get_y() - self.get_y()
        distance = math.sqrt(dx ** 2 + dy ** 2)
        if distance < (self.radius + other.radius):
            angle = math.atan2(dy, dx)
            total_mass = self.mass + other.mass
            self_vx = (self.get_vx() * (self.mass - other.mass) + 2 * other.mass * other.get_vx()) / total_mass
            self_vy = (self.get_vy() * (self.mass - other.mass) + 2 * other.mass * other.get_vy()) / total_mass
            other_vx = (other.get_vx() * (other.mass - self.mass) + 2 * self.mass * self.get_vx()) / total_mass
            other_vy = (other.get_vy() * (other.mass - self.mass) + 2 * self.mass * self.get_vy()) / total_mass
            self.vx, self.vy = self_vx*0.99, self_vy*0.99
            other.vx, other.vy = other_vx*0.99, other_vy*0.99
            overlap = 0.5 * (self.radius + other.radius - distance + 1e-6)

            if moveSelf:
                self.x -= overlap * math.cos(angle)
                self.y -= overlap * math.sin(angle)
                other.x += overlap * math.cos(angle)
                other.y += overlap * math.sin(angle)
            else:
                other.x += 2 * overlap * math.cos(angle)
                other.y += 2 * overlap * math.sin(angle)


class Obstacle(Particle):
    def __init__(self, x, y, angle, distance, frequency, radius, color, friction):
        super().__init__(x, y, 0, 0, 0.001, radius, color, friction)
        self.centerX = x
        self.centerY = y
        self.mAngle = angle
        self.mDistance = distance
        self.mFrequency = frequency

    def get_x(self):
        return self.centerX + self.mDistance * math.cos(self.mAngle)

    def get_y(self):
        return self.centerY + self.mDistance * math.sin(self.mAngle)

    def get_vx(self):
        return self.mFrequency * self.mDistance * math.sin(self.mAngle)

    def get_vy(self):
        return -self.mFrequency * self.mDistance * math.cos(self.mAngle)

    def update_position(self, dt):
        self.mAngle += self.mFrequency * dt
        self.x = self.centerX + self.mDistance * math.cos(self.mAngle)
        self.y = self.centerY + self.mDistance * math.sin(self.mAngle)


    def collide_with(self, other):
        if isinstance(other, Obstacle) or isinstance(other, Ground):
            return
        super().collide_with(other, False)
        self.vx, self.vy = 0, 0


class Ground(Obstacle):
    def __init__(self, x, y, angle, distance, frequency, radius, color, friction):
        super().__init__(x, y, angle, distance, frequency, radius, color, friction)

    def get_normal_to_center(self):
        return [math.cos(self.mAngle + math.pi), math.sin(self.mAngle + math.pi)]
    
    def collide_with(self, other):
        if isinstance(other, Ground) or isinstance(other, Obstacle):
            return
        dx = other.get_x() - self.get_x()
        dy = other.get_y() - self.get_y()
        distance = math.sqrt(dx ** 2 + dy ** 2)
        if distance < (self.radius + other.radius):
            angle = math.atan2(dy, dx)
            normal = self.get_normal_to_center()
            dot_product = other.get_vx() * normal[0] + other.get_vy() * normal[1]
            reflection = [other.get_vx() - 2 * dot_product * normal[0],
                          other.get_vy() - 2 * dot_product * normal[1]]
            
            other.vx = (reflection[0] * 0.5) - (self.get_vx() * self.friction)
            other.vy = (reflection[1] * 0.5) - (self.get_vy() * self.friction)

            overlap = 0.5 * (self.radius + other.radius - distance + 1e-6)
            other.x += 2 * overlap * math.cos(angle)
            other.y += 2 * overlap * math.sin(angle)

    
def create_ground_circle(centerX, centerY, distance, frequency, radius):
    circumference = 2 * math.pi * distance
    ground_count = int(circumference / (2 * radius))
    angle_increment = 2 * math.pi / ground_count
    ground_circle = []
    for i in range(ground_count):
        angle = i * angle_increment
        # x = centerX + distance * math.cos(angle)
        # y = centerY + distance * math.sin(angle)
        ground_circle.append(Ground(centerX, centerY, angle, distance, frequency, radius, (234, 221, 202), .4))
    return ground_circle


width = 800
height = 600

ring_radius = 120
centerX, centerY = width/2, height/2

pygame.init()
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Ring World Simulation")
clock = pygame.time.Clock()

particles = create_ground_circle(centerX,centerY, ring_radius, -0.008, 8)

numWater = round(len(particles)*2.5)
max_speed = 4
for i in range(numWater):
    x = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerX
    y = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerY
    vx = random.uniform(-max_speed, max_speed)
    vy = random.uniform(-max_speed, max_speed)

    particles.append(Particle(centerX, centerY, vx, vy, 2, 5, (0, 0, 255), 0.1))

for i in range(round(numWater/3)):
    x = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerX
    y = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerY
    vx = random.uniform(-max_speed, max_speed)
    vy = random.uniform(-max_speed, max_speed)

    particles.append(Particle(centerX, centerY, vx, vy, 0.2, 5, (0, 220, 140), 0.1))

running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    for particle in particles:
        particle.update_position(2)

    for pIdx1, p1 in enumerate(particles):
        for pIdx2, p2 in enumerate(particles):
            if pIdx1 == pIdx2:
                continue
            p1.collide_with(p2)

    # Draw the particles
    screen.fill((255, 255, 255))
    for particle in particles:
        pygame.draw.circle(screen, particle.color, (particle.x, particle.y), particle.radius)

    # Update the display and tick the clock
    pygame.display.flip()
    clock.tick(80)

# Quit Pygame
pygame.quit()