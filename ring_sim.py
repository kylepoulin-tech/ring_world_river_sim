import math
import pygame
import random


width = 800
height = 600

ring_radius = 100
centerX, centerY = width/2, height/2
dam_collides = True

class Particle:
    def __init__(self, x, y, vx, vy, mass, radius, color, friction, frequency):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.mass = mass
        self.radius = radius
        self.color = color
        self.friction = friction
        self.frequency = frequency

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

        distance_out_of_radius = math.hypot(self.x - centerX, self.y - centerY) - ring_radius
        if(distance_out_of_radius > 0):
            angle = math.atan2(self.y - centerY, self.x - centerX)
            normal = [math.cos(angle + math.pi), math.sin(angle + math.pi)]
            dot_product = self.get_vx() * normal[0] + self.get_vy() * normal[1]
            reflection = [self.get_vx() - 2 * dot_product * normal[0],
                          self.get_vy() - 2 * dot_product * normal[1]]
            
            angularVx = self.frequency * ring_radius * math.sin(angle)
            angularVy = -self.frequency * ring_radius * math.cos(angle)
            
            self.vx = (reflection[0] * 0.5) - (angularVx * self.friction)
            self.vy = (reflection[1] * 0.5) - (angularVy * self.friction)

            overlap = distance_out_of_radius + 1e-6
            self.x -= overlap * math.cos(angle)
            self.y -= overlap * math.sin(angle)


    def collide_with(self, other, moveSelf=True):
        if isinstance(other, Obstacle):
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
            self.vx, self.vy = self_vx, self_vy
            other.vx, other.vy = other_vx, other_vy
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
        super().__init__(x, y, 0, 0, 0.001, radius, color, friction, frequency)
        self.centerX = x
        self.centerY = y
        self.mAngle = angle
        self.mDistance = distance

    def get_x(self):
        return self.centerX + self.mDistance * math.cos(self.mAngle)

    def get_y(self):
        return self.centerY + self.mDistance * math.sin(self.mAngle)

    def get_vx(self):
        return self.frequency * self.mDistance * math.sin(self.mAngle)

    def get_vy(self):
        return -self.frequency * self.mDistance * math.cos(self.mAngle)

    def update_position(self, dt):
        self.mAngle += self.frequency * dt
        self.x = self.centerX + self.mDistance * math.cos(self.mAngle)
        self.y = self.centerY + self.mDistance * math.sin(self.mAngle)


    def collide_with(self, other):
        if isinstance(other, Obstacle) or not dam_collides:
            return
        super().collide_with(other, False)
        self.vx, self.vy = 0, 0


def rotate(origin, point, angle):
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


pygame.init()
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Ring World Simulation")
clock = pygame.time.Clock()


numWater = 200
max_speed = 4
frequency = -0.005
particle_size = 3.5
particles = []
for i in range(numWater):
    x = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerX
    y = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerY
    vx = random.uniform(-max_speed, max_speed)
    vy = random.uniform(-max_speed, max_speed)

    particles.append(Particle(centerX, centerY, vx, vy, 2, particle_size, (0, 0, 255), 0.4, frequency))

for i in range(round(numWater/5)):
    x = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerX
    y = random.uniform(-ring_radius*0.1, ring_radius*0.1) + centerY
    vx = random.uniform(-max_speed, max_speed)
    vy = random.uniform(-max_speed, max_speed)

    particles.append(Particle(centerX, centerY, vx, vy, 0.2, round(particle_size/2), (0, 220, 140), 0.01, frequency))

#particles.append(Obstacle(centerX, centerY, 0, ring_radius - (2*particle_size), frequency, particle_size, (234,221,202), 0))
particles.append(Obstacle(centerX, centerY, 0.5*math.pi, ring_radius - 2*(2*particle_size), frequency, particle_size, (234,221,202), 0))
particles.append(Obstacle(centerX, centerY, 0.5*math.pi, ring_radius - 3*(2*particle_size), frequency, particle_size, (234,221,202), 0))
particles.append(Obstacle(centerX, centerY, 0.5*math.pi, ring_radius - 4*(2*particle_size), frequency, particle_size, (234,221,202), 0))
particles.append(Obstacle(centerX, centerY, 0.5*math.pi, ring_radius - 5*(2*particle_size), frequency, particle_size, (234,221,202), 0))
particles.append(Obstacle(centerX, centerY, 0.5*math.pi, ring_radius - 6*(2*particle_size), frequency, particle_size, (234,221,202), 0))
particles.append(Obstacle(centerX, centerY, 0.5*math.pi, ring_radius - 7*(2*particle_size), frequency, particle_size, (234,221,202), 0))
particles.append(Obstacle(centerX, centerY, 0.5*math.pi, ring_radius - 8*(2*particle_size), frequency, particle_size, (234,221,202), 0))

dam_perspective = True
running = True
dt = 3
rotationAngle = 0
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            dam_collides = not dam_collides

    for particle in particles:
        particle.update_position(dt)

    for pIdx1, p1 in enumerate(particles):
        for pIdx2, p2 in enumerate(particles):
            if pIdx1 == pIdx2:
                continue
            p1.collide_with(p2)

    # Draw the particles
    screen.fill((255, 255, 255))
    circle_line_width = 2
    pygame.draw.circle(screen, (234,221,202), (centerX+ring_radius*1.25, centerY), ring_radius+particle_size+circle_line_width, circle_line_width)
    pygame.draw.circle(screen, (234,221,202), (centerX-ring_radius*1.25, centerY), ring_radius+particle_size+circle_line_width, circle_line_width)
    rotationAngle += frequency * dt
    for particle in particles:
            newParticleX, newParticleY = rotate((centerX, centerY), (particle.x, particle.y), -rotationAngle)
            pygame.draw.circle(screen, particle.color, (newParticleX+ring_radius*1.25, newParticleY), particle.radius)
            pygame.draw.circle(screen, particle.color, (particle.x-ring_radius*1.25, particle.y), particle.radius)


    # Update the display and tick the clock
    pygame.display.flip()
    clock.tick(120)

# Quit Pygame
pygame.quit()