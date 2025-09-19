import pygame
import sys
import math
import random
import time

# ---------- Config ----------
WIDTH, HEIGHT = 1100, 700
FPS = 60

# Missile parameters
MISSILE_BASE_SPEED = 240.0
MISSILE_MAX_SPEED = 520.0
DIST_SPEED_GAIN = 0.45
MISSILE_MAX_TURN = math.radians(350)  # rad/s

# PN parameters
NAV_CONSTANT_N = 4.0

# PID parameters (uncomment to use PID instead of PN)
# PID_KP = 2.0      # Proportional gain
# PID_KI = 0.1      # Integral gain  
# PID_KD = 0.5      # Derivative gain
# PID_MAX_INTEGRAL = 100.0  # Anti-windup limit

# Control method selection
USE_PID = False  # Set to True to use PID, False for Proportional Navigation

# Explosion
EXPLOSION_TIME = 0.7
EXPLOSION_MAX_RADIUS = 70

# Target inertia
TARGET_DAMPING = 0.995

# Visual
MISSILE_SIZE = 18
TARGET_RADIUS = 12

# Obstacle parameters
NUM_OBSTACLES = 6
CIRCLE_RATIO = 0.5  # fraction of circles

# ---------- Helpers ----------
def angle_normalize(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def clamp(x, a, b):
    return max(a, min(b, x))

def signed_angle_between(a_from, a_to):
    return angle_normalize(a_to - a_from)

# ---------- Classes ----------
class PIDController:
    """
    PID Controller for missile guidance
    Alternative to Proportional Navigation
    """
    def __init__(self, kp=2.0, ki=0.1, kd=0.5, max_integral=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        
        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.first_call = True
    
    def update(self, error, dt):
        """
        Calculate PID output based on angle error
        Args:
            error: angle error (radians) - normalized between -pi and pi
            dt: delta time (seconds)
        Returns:
            control_output: angular velocity command (rad/s)
        """
        if self.first_call:
            self.prev_error = error
            self.first_call = False
        
        # Proportional term: u(t) = Kp * e(t)
        proportional = self.kp * error
        
        # Integral term: Ki * ∫ e(t) dt
        self.integral += error * dt
        # Anti-windup: clamp integral to prevent excessive buildup
        self.integral = clamp(self.integral, -self.max_integral, self.max_integral)
        integral_term = self.ki * self.integral
        
        # Derivative term: Kd * de(t)/dt
        derivative = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        # Combined PID output: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
        control_output = proportional + integral_term + derivative
        
        return control_output
    
    def reset(self):
        """Reset PID state (call when missile respawns)"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.first_call = True

class Explosion:
    def __init__(self, x, y, start_time):
        self.x = x
        self.y = y
        self.start_time = start_time
        self.alive = True

    def draw(self, surf, now):
        t = (now - self.start_time)/EXPLOSION_TIME
        if t >= 1.0:
            self.alive = False
            return
        r = EXPLOSION_MAX_RADIUS * t
        alpha = int(255*(1-t))
        s = pygame.Surface((int(r*2)+4,int(r*2)+4), pygame.SRCALPHA)
        pygame.draw.circle(s,(255,160,0,alpha),(int(r)+2,int(r)+2),int(r),width=6)
        surf.blit(s,(self.x-r-2,self.y-r-2))
        for i in range(6):
            ang = i*math.pi/3 + t*6
            sx = self.x + math.cos(ang)*(r*0.6+6*math.sin(t*10+i))
            sy = self.y + math.sin(ang)*(r*0.6+6*math.cos(t*10+i*0.7))
            pygame.draw.circle(s,(255,220,80),(int(sx),int(sy)),max(1,int(3*(1-t))))

class Target:
    def __init__(self,x=WIDTH//2,y=HEIGHT//2):
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.launched = False
        self.alive = True

    def set_launch(self,x,y,vx,vy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.launched = True
        self.alive = True

    def update(self, dt):
        if not self.launched or not self.alive:
            return
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.vx *= TARGET_DAMPING
        self.vy *= TARGET_DAMPING
        if self.x<0: self.x=0; self.vx=-self.vx*0.6
        if self.x>WIDTH: self.x=WIDTH; self.vx=-self.vx*0.6
        if self.y<0: self.y=0; self.vy=-self.vy*0.6
        if self.y>HEIGHT: self.y=HEIGHT; self.vy=-self.vy*0.6

    def draw(self,surf):
        if not self.alive: return
        pygame.draw.circle(surf,(70,200,120),(int(self.x),int(self.y)),TARGET_RADIUS)
        tail_x = int(self.x - clamp(self.vx,-180,180)*0.06)
        tail_y = int(self.y - clamp(self.vy,-180,180)*0.06)
        pygame.draw.circle(surf,(255,200,50),(tail_x,tail_y),4)

class Missile:
    def __init__(self,x,y,angle=0.0):
        self.spawn_x=x; self.spawn_y=y
        self.x=x; self.y=y
        self.angle=angle
        self.speed=MISSILE_BASE_SPEED
        
        # PN state
        self.prev_los=None
        
        # PID controller (uncomment to use PID)
        # self.pid_controller = PIDController(PID_KP, PID_KI, PID_KD, PID_MAX_INTEGRAL)
        
        self.respawn_time=None
        self.alive=True

    def respawn(self):
        self.x=self.spawn_x; self.y=self.spawn_y
        dx=WIDTH/2-self.x; dy=HEIGHT/2-self.y
        self.angle=math.atan2(dy,dx)
        self.speed=MISSILE_BASE_SPEED
        self.prev_los=None
        
        # Reset PID controller when using PID
        # if USE_PID:
        #     self.pid_controller.reset()
        
        self.alive=True
        self.respawn_time=None

    def start_respawn_timer(self,delay):
        self.respawn_time=time.time()
        self.alive=False
        self.respawn_delay=delay

    def is_active(self):
        return self.alive

    def update(self,target,obstacles,dt):
        if not self.alive:
            if time.time()-self.respawn_time>=self.respawn_delay:
                self.respawn()
            else: return None
        
        # Calculate guidance command based on selected method
        if USE_PID:
            # ============= PID APPROACH =============
            # Calculate desired angle to target
            tx, ty = target.x, target.y
            desired_angle = math.atan2(ty - self.y, tx - self.x)
            
            # Calculate angle error (setpoint - measurement)
            angle_error = signed_angle_between(self.angle, desired_angle)
            
            # PID controller calculates required angular velocity
            # u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
            pn_rate = self.pid_controller.update(angle_error, dt)
            
            # Optional: Add lead angle compensation for moving targets
            # target_vel = math.hypot(target.vx, target.vy)
            # if target_vel > 10:  # Only if target is moving significantly
            #     time_to_intercept = math.hypot(tx - self.x, ty - self.y) / self.speed
            #     lead_x = tx + target.vx * time_to_intercept
            #     lead_y = ty + target.vy * time_to_intercept
            #     desired_angle = math.atan2(lead_y - self.y, lead_x - self.x)
            #     angle_error = signed_angle_between(self.angle, desired_angle)
            #     pn_rate = self.pid_controller.update(angle_error, dt)
            
        else:
            # ============= PROPORTIONAL NAVIGATION APPROACH =============
            # Calculate Line of Sight (LOS) to target
            tx, ty = target.x, target.y
            los = math.atan2(ty - self.y, tx - self.x)
            
            # Calculate LOS rate (λ̇ - lambda dot)
            if self.prev_los is None: 
                lambda_dot = 0.0
            else:
                lambda_dot = signed_angle_between(self.prev_los, los) / dt
            self.prev_los = los
            
            # Proportional Navigation: am = N * Vm * λ̇
            # Here we convert lateral acceleration to angular velocity
            pn_rate = NAV_CONSTANT_N * lambda_dot

        # Obstacle avoidance (same for both methods)
        avoid_turn=0.0
        for obs in obstacles:
            if obs['type']=='circle':
                ox,oy,r=obs['x'],obs['y'],obs['r']
                dx,dy=ox-self.x, oy-self.y
                dist=math.hypot(dx,dy)
                if dist>r+150: continue
                ang_to_obs=math.atan2(dy,dx)
                ang_error=signed_angle_between(self.angle,ang_to_obs)
                closeness=clamp((r+150-dist)/150,0.0,1.0)
                turn_dir=-math.copysign(1.0,ang_error)
                heading_toward=max(0.0, math.cos(ang_error))
                avoid_turn+=1.5*turn_dir*closeness*heading_toward*MISSILE_MAX_TURN
            elif obs['type']=='rect':
                rx,ry,w,h=obs['x'],obs['y'],obs['w'],obs['h']
                # center of rectangle
                cx=rx+w/2; cy=ry+h/2
                dx,dy=cx-self.x, cy-self.y
                dist=math.hypot(dx,dy)
                influence=max(w,h)/2+120
                if dist>influence: continue
                ang_to_obs=math.atan2(dy,dx)
                ang_error=signed_angle_between(self.angle,ang_to_obs)
                closeness=clamp((influence-dist)/influence,0.0,1.0)
                turn_dir=-math.copysign(1.0,ang_error)
                heading_toward=max(0.0, math.cos(ang_error))
                avoid_turn+=1.5*turn_dir*closeness*heading_toward*MISSILE_MAX_TURN

        # Combine guidance and avoidance
        ang_vel=clamp(pn_rate+avoid_turn,-MISSILE_MAX_TURN,MISSILE_MAX_TURN)
        self.angle+=ang_vel*dt; self.angle=angle_normalize(self.angle)
        
        # Update speed and position
        dist_to_target=math.hypot(tx-self.x,ty-self.y)
        speed_target=MISSILE_BASE_SPEED+DIST_SPEED_GAIN*dist_to_target
        self.speed=clamp(speed_target,MISSILE_BASE_SPEED,MISSILE_MAX_SPEED)
        self.x+=math.cos(self.angle)*self.speed*dt
        self.y+=math.sin(self.angle)*self.speed*dt
        
        return {'dist':dist_to_target,'ang_vel':ang_vel,'speed':self.speed}

    def draw(self,surf):
        if not self.alive: return
        pts=[(MISSILE_SIZE,0),(-MISSILE_SIZE*0.6,MISSILE_SIZE*0.6),(-MISSILE_SIZE*0.6,-MISSILE_SIZE*0.6)]
        rotated=[]; c=math.cos(self.angle); s=math.sin(self.angle)
        for px,py in pts:
            rx=px*c-py*s; ry=px*s+py*c
            rotated.append((self.x+rx,self.y+ry))
        pygame.draw.polygon(surf,(220,60,60),rotated)
        pygame.draw.circle(surf,(0,0,0),(int(self.x),int(self.y)),3)

# ---------- Main ----------
def main():
    pygame.init()
    screen=pygame.display.set_mode((WIDTH,HEIGHT))
    clock=pygame.time.Clock()
    font=pygame.font.SysFont(None,20)

    target=Target()
    missile=Missile(WIDTH*0.85,HEIGHT*0.25,math.pi)

    explosions=[]
    dragging=False; drag_start=(0,0); drag_end=(0,0)
    show_debug=True

    # create obstacles
    obstacles=[]
    random.seed(42)
    for _ in range(NUM_OBSTACLES):
        if random.random()<CIRCLE_RATIO:
            # circle
            ox=random.randint(100,WIDTH-100); oy=random.randint(100,HEIGHT-100)
            r=random.randint(25,60)
            obstacles.append({'type':'circle','x':ox,'y':oy,'r':r})
        else:
            # rectangle
            rw=random.randint(50,120); rh=random.randint(50,120)
            rx=random.randint(50,WIDTH-rw-50); ry=random.randint(50,HEIGHT-rh-50)
            obstacles.append({'type':'rect','x':rx,'y':ry,'w':rw,'h':rh})

    running=True
    while running:
        dt=clock.tick(FPS)/1000.0
        for event in pygame.event.get():
            if event.type==pygame.QUIT: running=False
            elif event.type==pygame.KEYDOWN:
                if event.key==pygame.K_d: show_debug=not show_debug
                if event.key==pygame.K_r: missile.respawn()
                if event.key==pygame.K_ESCAPE: running=False
                # Toggle between PID and PN (uncomment to enable)
                # if event.key==pygame.K_p: 
                #     global USE_PID
                #     USE_PID = not USE_PID
                #     print(f"Switched to {'PID' if USE_PID else 'Proportional Navigation'}")
                #     missile.respawn()  # Reset missile state
            elif event.type==pygame.MOUSEBUTTONDOWN and event.button==1:
                dragging=True; drag_start=pygame.mouse.get_pos(); drag_end=drag_start
            elif event.type==pygame.MOUSEMOTION and dragging: drag_end=pygame.mouse.get_pos()
            elif event.type==pygame.MOUSEBUTTONUP and event.button==1 and dragging:
                dragging=False
                sx,sy=drag_start; ex,ey=drag_end
                vx=(ex-sx)*4.0; vy=(ey-sy)*4.0
                if math.hypot(vx,vy)<1.0: vx,vy=0.0,0.0
                target.set_launch(sx,sy,vx,vy)
                if not missile.is_active(): missile.respawn()

        # update
        target.update(dt)
        info=missile.update(target,obstacles,dt)

        # check collisions
        if target.alive and missile.is_active():
            # missile <-> target
            dx=target.x-missile.x; dy=target.y-missile.y
            if math.hypot(dx,dy)<TARGET_RADIUS+MISSILE_SIZE*0.6:
                explosions.append(Explosion(target.x,target.y,time.time()))
                missile.start_respawn_timer(1.0)
                target.alive=False

            # missile <-> obstacles
            for obs in obstacles:
                if obs['type']=='circle':
                    dist=math.hypot(missile.x-obs['x'],missile.y-obs['y'])
                    if dist<obs['r']:
                        explosions.append(Explosion(missile.x,missile.y,time.time()))
                        missile.start_respawn_timer(1.0)
                elif obs['type']=='rect':
                    if obs['x']<missile.x<obs['x']+obs['w'] and obs['y']<missile.y<obs['y']+obs['h']:
                        explosions.append(Explosion(missile.x,missile.y,time.time()))
                        missile.start_respawn_timer(1.0)
            # target <-> obstacles
            for obs in obstacles:
                if not target.alive: break
                if obs['type']=='circle':
                    dist=math.hypot(target.x-obs['x'],target.y-obs['y'])
                    if dist<obs['r']:
                        explosions.append(Explosion(target.x,target.y,time.time()))
                        target.alive=False
                elif obs['type']=='rect':
                    if obs['x']<target.x<obs['x']+obs['w'] and obs['y']<target.y<obs['y']+obs['h']:
                        explosions.append(Explosion(target.x,target.y,time.time()))
                        target.alive=False

        now=time.time()
        explosions=[e for e in explosions if e.alive]

        # draw
        screen.fill((18,18,24))
        # obstacles
        for obs in obstacles:
            if obs['type']=='circle':
                pygame.draw.circle(screen,(120,90,40),(int(obs['x']),int(obs['y'])),int(obs['r']))
            elif obs['type']=='rect':
                pygame.draw.rect(screen,(120,90,40),pygame.Rect(obs['x'],obs['y'],obs['w'],obs['h']))

        # drag preview
        if dragging:
            sx,sy=drag_start; ex,ey=drag_end
            pygame.draw.circle(screen,(100,200,120),(sx,sy),TARGET_RADIUS,2)
            pygame.draw.line(screen,(255,200,60),(sx,sy),(ex,ey),3)
            ang=math.atan2(ey-sy,ex-sx)
            ahx=ex-math.cos(ang)*8; ahy=ey-math.sin(ang)*8
            pygame.draw.circle(screen,(255,200,60),(int(ahx),int(ahy)),5)

        # LOS
        if missile.is_active() and target.alive:
            pygame.draw.line(screen,(80,80,140),(missile.x,missile.y),(target.x,target.y),1)

        target.draw(screen)
        missile.draw(screen)

        for e in explosions: e.draw(screen,now)

        # debug
        if show_debug:
            x0,y0=8,8
            lines=[
                "Click+drag: place rocket and set velocity (release to launch)",
                "R: reset missile | D: toggle debug | Esc: quit",
                # f"Control method: {'PID' if USE_PID else 'Proportional Navigation'} (P to toggle)",  # Uncomment to show current method
                ""
            ]
            if info:
                lines.append(f"Missile active: {missile.is_active()}")
                lines.append(f"Distance to target: {info['dist']:.1f}")
                lines.append(f"Angular vel applied: {math.degrees(info['ang_vel']):+.1f} deg/s")
                lines.append(f"Missile speed: {info['speed']:.1f}")
                lines.append(f"Control method: {'PID' if USE_PID else 'Proportional Navigation'}")
            else:
                lines.append("Missile respawning...")
            for i,t in enumerate(lines):
                surf=font.render(t,True,(220,220,220))
                screen.blit(surf,(x0,y0+i*18))

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__=="__main__":
    main()