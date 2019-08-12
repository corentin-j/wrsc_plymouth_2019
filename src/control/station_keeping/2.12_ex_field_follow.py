from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py



def draw_field_vdp(xmin,xmax,ymin,ymax):
    Mx    = arange(xmin,xmax,1)
    My    = arange(ymin,ymax,1)
    X1,X2 = meshgrid(Mx,My)
    VX    = X2
    VY    = -(0.01*(X1**2)-1)*X2-X1
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()

def draw_field_line(xmin,xmax,ymin,ymax):
    Mx    = arange(xmin,xmax,1)
    My    = arange(ymin,ymax,1)
    X1,X2 = meshgrid(Mx,My)
    a,b = np.array([[-5],[5]]), np.array([[5],[-5]])
    m = np.array([[X1],[X2]])

    e = np.linalg.det(np.hstack(((b-a)/np.linalg.norm(b-a), m-a)))
    print(e,q)
    phi = np.arctan2(b[1,0]-a[1,0], b[0,0]-a[0,0])
    thetabar = phi - arctan(e/r)


    theta = 5#np.pi/4
    VX    = X2/X2#cos(-arctan(X2))
    VY    = X1/X1#sin(-arctan(X2))
    
    #vect = np.array([[VX],[VY]])
    #rot = np.array([[cos(theta),-sin(theta)],[sin(theta),cos(theta)]])
    #res = rot@vect
    #VX,VY = res[0,0],res[1,0]
    #VX    = cos(theta)*VX-sin(theta)*VY
    #VY    = sin(theta)*VX+cos(theta)*VY
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()

def draw_field_sin(xmin,xmax,ymin,ymax):
    Mx    = arange(xmin,xmax,1)
    My    = arange(ymin,ymax,1)
    X1,X2 = meshgrid(Mx,My)
    VX    = 1
    VY    = sin(X1)-X2+cos(X1)
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)
    return()

def trace_sin():
    dt= 0.05
    t = arange(-125,125,dt)
    x = np.array([t,sin(t)])
    plot(x[0], x[1],'b.')


def trace_vdp():
    x_vdp = np.array([[2.0],[2.0]])
    dt= 0.05
    for t in arange(0,250,dt):
        plot(x_vdp[0,0], x_vdp[1,0],'b.')
        x_vdp += dt*np.array([[x_vdp[1,0]],
                              [-(0.01*(x_vdp[0,0]**2)-1)*x_vdp[1,0]-x_vdp[0,0]]])


def define_u_vdp(x):
    #definition des variables de calcul
    x1, x2, x3 = x[0,0], x[1,0], x[2,0]
    dx1, dx2 = f(x,0)[0,0], f(x,0)[1,0]
    a = x2
    da = dx2
    b = -(0.01*x1*x1-1)*x2-x1
    db = -(0.02*dx1*x1*x2 + (0.01*x1*x1-1)*dx2) - dx1
    #fonction à minimiser
    y = sawtooth(x3 - np.arctan2(b,a))
    #u = 2*np.sign(-y) + (-b*da+a*db)/(a*a+b*b)
    u = -y
    return u

def define_u_sin(x):
    #definition des variables de calcul
    x1, x2, x3 = x[0,0], x[1,0], x[2,0]
    dx1, dx2 = f(x,0)[0,0], f(x,0)[1,0]
    a = 1
    da = 0
    b = sin(x1)-x2+cos(x1)
    db = dx1*cos(x1)-dx2-dx1*sin(x1)
    #fonction à minimiser
    y = sawtooth(x3 - np.arctan2(b,a))
    u = 5*np.sign(-y) + (-b*da+a*db)/(a*a+b*b)
    return u

def define_u_line(x):
    x = x.flatten()
    x1, x2, x3 = x[0], x[1], x[2]

    r = 1
    a,b = np.array([[-5],[5]]), np.array([[5],[-5]])
    m = np.array([[x1],[x2]])
    e = np.linalg.det(np.hstack(((b-a)/np.linalg.norm(b-a), m-a)))
    #print(e)
    phi = np.arctan2(b[1,0]-a[1,0], b[0,0]-a[0,0])

    eps, cx, cy, r = 1, 0, 0, 3
    x1, x2 = eps*(x1-cx)/r, (x2-cy)/r
    VX    = eps*(-x1**3-x1*(x2**2)+x1-x2)
    VY    = -x2**3-x2*(x1**2)+x1+x2
    R=sqrt(VX**2+VY**2)

    thetabar = np.arctan2(VY/R,VX/R)


    u = -10/pi*arctan(tan(0.5*(x3-thetabar)))
    return(u)
    
def draw(x):
    draw_tank(x,'darkblue',0.3)
    a,b = array([[-30],[0]]), array([[30],[0]])
    draw_segment(a,b,'red',2)
    
def f(x,u):
    θ=x[2,0]
    return array([[cos(θ)], [sin(θ)],[u]])
           

x=array([[-3],[10],[3]])
dt= 0.1#0.05
a=10
ax=init_figure(-a,a,-a,a)
#draw_field_sin(-a,a,-a,a)
#draw_field_line(-a,a,-a,a)
#trace_sin()


for t in arange(0,10,dt):
    plot(x[0,0], x[1,0],'r.')
    u=define_u_line(x) #define_u_sin(x)
    x=x+dt*f(x,u)
    pause(0.001)
