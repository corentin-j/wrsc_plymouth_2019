from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py



def draw_field_circle(xmin,xmax,ymin,ymax,e,eps,cx,cy):
    Mx    = arange(xmin,xmax,1)
    My    = arange(ymin,ymax,1)
    X1,X2 = meshgrid(Mx,My)
    X1, X2 = eps*(X1-cx)/r, (X2-cy)/r
    VX    = eps*(-X1**3-X1*(X2**2)+X1-X2)
    VY    = -X2**3-X2*(X1**2)+X1+X2
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()

def control(x, r, eps, cx, cy):
    x,y,th = x.flatten()
    x,y = eps*(x-cx)/r,(y-cy)/r
    a    = eps*(-x**3-x*(y**2)+x-y)
    b    = -y**3-y*(x**2)+x+y
    y = sawtooth(th - np.arctan2(b,a))
    print(x,y,eps)
    return -y
    
def draw(x):
    draw_tank(x,'darkblue',0.3)
    
def f(x,u):
    θ=x[2,0]
    return array([[cos(θ)], [sin(θ)],[u]])
           

x=array([[-2],[2],[3]])
dt= 0.5
a=10
ax=init_figure(-2*a,2*a,-a,a)
r = 5
c0,c1,c2 = [-7,0],[3,0],[12,0]

lc =         [c0,c1,c2,c1]
lswitch =    [1, 0, 1, 0 ]
ldirection = [-1,1,-1, 1 ]
n = len(lc)
state = 0
if x[1,0] > 0:
    switch = 1
else:
    switch = 0
sign = np.sign(x[1,0])

direction = -1
for t in arange(0,100,dt):
    clear(ax)
    draw_field_circle(-2*a,2*a,-a,a,r,ldirection[state%n],lc[state%n][0],lc[state%n][1])
    u=control(x,r,ldirection[state%n],lc[state%n][0],lc[state%n][1])
    x=x+dt*f(x,u)
    if np.sign(x[1,0])*sign == -1:
        switch += 1
    sign = np.sign(x[1,0])
    if switch > lswitch[state%n]:
        state+=1
        switch = 0
    
    draw(x)
    
pause(0.001)
