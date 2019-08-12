#available at https://www.ensta-bretagne.fr/jaulin/roblib.py 
# For help : https://www.ensta-bretagne.fr/jaulin/python.html  
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/kalmooc.html
# used in RobMOOC :  https://www.ensta-bretagne.fr/jaulin/robmooc.html
# used in KalMOOC :  https://www.ensta-bretagne.fr/jaulin/inmooc.html


import numpy as np
import matplotlib.pyplot as plt
from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from matplotlib.pyplot import *
from numpy.random import randn,rand
from numpy.linalg import inv, det, norm, eig
from scipy.linalg import sqrtm,expm,norm,block_diag

from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial
from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection



# Unicode https://en.wikipedia.org/wiki/List_of_Unicode_characters
# for instance to get θ : shift + ctr + U03B8  
# U+03B1 α alpha;  U+03B2 β beta; U+03B3;	 Gamma 	0419; U+03B4 δ Delta;
#U+03B5 ε Epsilon;  U+03B6 Zeta; U+03B7 Eta; U+03B8 θ Theta;
#U+03BB λ Lambda; U+03BC Mu; U+03BD Nu; U+03BE Xi; U+03C0 Pi; U+03C1 ρ Rho;
# U+03C3 Sigma; U+03C4 τTau; U+03C6 φ Phi; U+03C8 ψ Psi; U+03C9 Omega ω
# U+0393 Gamma Γ



    
def eulermat(φ,θ,ψ):
    Ad_i = array([[0, 0, 0],[0,0,-1],[0,1,0]])
    Ad_j = array([[0,0,1],[0,0,0],[-1,0,0]])
    Ad_k = array([[0,-1,0],[1,0,0],[0,0,0]])
    M = expm(ψ*Ad_k) @ expm(θ*Ad_j) @ expm(φ*Ad_i)
    return(M)    

def eulerderivative(φ,θ,ψ):
    cφ,sφ,cθ,sθ,tθ,cψ,sψ = cos(φ),sin(φ),cos(θ),sin(θ),sin(θ)/cos(θ),cos(ψ),sin(ψ)        
    return array([[1,sφ*tθ,cφ*tθ],[0, cφ,-sφ],[0,sφ/cθ,cφ/cθ]])    
    
def angle(x):
    x=x.flatten()
    return arctan2(x[1],x[0])
    
def adjoint(w):    
    w=w.flatten()
    return array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

    
def move_motif(M,x,y,θ):
    M1=ones((1,len(M[1,:])))
    M2=vstack((M, M1))
    R = array([[cos(θ),-sin(θ),x], [sin(θ),cos(θ),y]])
    return(R @ M2)    

def translate_motif(R,x,y,z):
    return   R + array([[x],[y],[z]]) @ ones((1,R.shape[1]))

def motif_circle3D(r):
    n = 10
    θ = linspace(0, 2*pi, n)
    x = r*cos(θ) + array(n*[0])
    y = r*sin(θ) + array(n*[0])
    z = zeros(n)
    return array([x,y,z])

def motif_auv3D(): #needed by draw_auv3d and sphere
    return array([ [0.0,0.0,10.0,0.0,0.0,10.0,0.0,0.0],
                   [-1.0,1.0,0.0,-1.0,-0.2,0.0,0.2,1.0],
                   [0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0]])

def motif_wheel3D(r):
    n = 20
    W=[[0.3,0],[0,0],[0,0]]
    for i in range(n+1):
        R=[[0],[r*cos(2*pi*i/n)],[r*sin(2*pi*i/n)]]
        W=hstack((W,R,[[0],[0],[0]],R)) 
    return array(W)
                   


    
def draw_wheel3D(ax,x,y,z,φ,θ,ψ,r=1,col='blue',size=1):
    M=motif_wheel3D(r)
    draw_motif3D(ax,M,x,y,z,φ,θ,ψ,col,1)
    p=array([[x],[y],[z]])+eulermat(φ,θ,ψ)@array([[0],[1],[0]])
    ax.scatter(*p,color='red')
    


    
def draw_auv3D(ax,x,y,z,φ,θ,ψ,col='blue',size=1):
    M=size*eulermat(φ,θ,ψ) @ motif_auv3D()
    M=translate_motif(M,x,y,z)
    ax.plot(M[0],M[1],1*M[2],color=col)
    ax.plot(M[0],M[1],0*M[2],color='grey')
    
def draw_arrow3D(ax,x,y,z,wx,wy,wz,col):  # initial point : x ; final point x+w 
    ax.quiver(x,y,z,wx,wy,wz,color=col,lw=1,pivot='tail',length=norm([wx,wy,wz]))

def draw_motif3D(ax,M,x,y,z,φ,θ,ψ,col,mirror=1):   #mirror=-1 in case z in directed downward
    M=eulermat(φ,θ,ψ) @ M
    M=translate_motif(M,x,y,z) 
    ax.plot(mirror*M[0],M[1],mirror*M[2],color=col)
    ax.plot(mirror*M[0],M[1],0*M[2],color='black')


def draw_axis3D(ax,x,y,z,R,zoom=1):
    ax.scatter(x,y,z,color='magenta')
    R=zoom*R
    draw_arrow3D(ax,x,y,z,R[0,0],R[1,0],R[2,0],"red")
    draw_arrow3D(ax,x,y,z,R[0,1],R[1,1],R[2,1],"green")
    draw_arrow3D(ax,x,y,z,R[0,2],R[1,2],R[2,2],"blue")
   
        
def draw_quadrotor3D(ax,x,α,l):
    Ca=hstack((motif_circle3D(0.3*l),[[0.3*l,-0.3*l],[0,0],[0,0]])) # the disc + the blades
    C0=eulermat(0,0,α[0])@Ca  # we rotate the blades    
    C1=eulermat(0,0,-α[1])@Ca
    C2=eulermat(0,0,α[2])@Ca
    C3=eulermat(0,0,-α[3])@Ca
    C0=translate_motif(C0,0,l,0)
    C1=translate_motif(C1,-l,0,0)
    C2=translate_motif(C2,0,-l,0)
    C3=translate_motif(C3,l,0,0)
    M = array([[l,-l,0,0, 0],[0,0,0,l,-l],[0,0,0,0,0]])
    x=x.flatten()        
    draw_motif3D(ax,C0,*x[0:6],'green',-1)  #right propeler 0
    draw_motif3D(ax,C1,*x[0:6],'black',-1)  #right propeler 1
    draw_motif3D(ax,C2,*x[0:6],'red',-1)  #right propeler 2
    draw_motif3D(ax,C3,*x[0:6],'blue',-1)  #right propeler 3
    draw_motif3D(ax,M,*x[0:6],'grey',-1)  #body    
    
    
    
    
def plot2D(M,col='black',w=1):
    plot(M[0, :], M[1, :], col, linewidth = w)         
    
def plot3D(ax,M,col='black',w=1):
    ax.plot(M[0, :], M[1, :],M[2, :], col, linewidth = w)         
    

def draw_segment(a,b,col='darkblue',w=1):
    plot2D(hstack((a,b)),col, w)
    #plot2D(a,'ro')
    #plot2D(b,'ro')      
  
def draw_ellipse(c,Γ,η,ax,col): # Gaussian confidence ellipse with artist
    #draw_ellipse(array([[1],[2]]),eye(2),0.9,ax,[1,0.8-0.3*i,0.8-0.3*i])
    if (norm(Γ)==0):
        Γ=Γ+0.001*eye(len(Γ[1,:]))
    A=sqrtm(-2*log(1-η)*Γ)    
    w, v = eig(A)    
    v1=array([[v[0,0]],[v[1,0]]])
    v2=array([[v[0,1]],[v[1,1]]])        
    f1=A @ v1
    f2=A @ v2      
    φ =  (arctan2(v1 [1,0],v1[0,0]))
    α=φ*180/3.14
    e = Ellipse(xy=c, width=2*norm(f1), height=2*norm(f2), angle=α)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.7)
    e.set_facecolor(col)
    
    

def draw_disk(c,r,ax,col,alph=0.7,w=1): 
    #draw_disk(array([[1],[2]]),0.5,ax,"blue")
    e = Ellipse(xy=c, width=2*r, height=2*r, angle=0,linewidth = w)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(alph)  # transparency
    e.set_facecolor(col)
    
    

def draw_box(x1,x2,y1,y2,ax,col): 
    c=array([[x1],[y1]])    
    rect = Rectangle(c, width=x2-x1, height=y2-y1, angle=0)
    rect.set_facecolor(array([0.4,0.3,0.6]))   
    ax.add_patch(rect)
    rect.set_clip_box(ax.bbox)
    rect.set_alpha(0.7)
    rect.set_facecolor(col)    

def draw_polygon(P,ax,col): 
    patches = []     
    patches.append(Polygon(P, True))    
    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4, color=col)
    ax.add_collection(p)


def draw_arc(c,a,θ,col):
    s = arange(0,abs(θ),0.01)
    s = sign(θ) * s
    d = a-c
    r = norm(d)
    alpha = angle(d)
    w = c@ones((1,size(s))) + r*array([[cos(alpha), -sin(alpha)],[sin(alpha), cos(alpha)]])@array([cos(s),sin(s)])
    plot2D(w,col,3)  
    
    
def draw_arrow(x,y,θ,L,col):
    e=0.2
    M1=L*array([[0,1,1-e,1,1-e],[0,0,-e,0,e]])
    M=np.append(M1,[[1,1,1,1,1]],axis=0)
    R=array([[cos(θ),-sin(θ),x],[sin(θ),cos(θ),y],[0,0,1]])
    plot2D(R@M,col)    
    
def draw_sailboat(x,δs,δr,ψ,awind):
    x=x.flatten()
    θ=x[2]
    hull=array([[-1,5,7,7,5,-1,-1,-1],[-2,-2,-1,1,2,2,-2,-2],[1,1,1,1,1,1,1,1]])
    sail=array([[-7,0],[0,0],[1,1]])
    rudder=array([[-1,1],[0,0],[1,1]])
    R=array([[cos(θ),-sin(θ),x[0]],[sin(θ),cos(θ),x[1]],[0,0,1]])
    Rs=array([[cos(δs),-sin(δs),3],[sin(δs),cos(δs),0],[0,0,1]])
    Rr=array([[cos(δr),-sin(δr),-1],[sin(δr),cos(δr),0],[0,0,1]])
    draw_arrow(x[0]+5,x[1],ψ,5*awind,'red')
    plot2D(R@hull,'black');       
    plot2D(R@Rs@sail,'red',2);       
    plot2D(R@Rr@rudder,'red',2);

def draw_tank(x,col='darkblue',r=1,w=2):
    x=x.flatten()
    M = r*array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M=move_motif(M,x[0],x[1],x[2])
    plot2D(M,col,w)

    	
def draw_car(x,col='darkblue',r=1,w=2):
    x=x.flatten();
    M = r*array([ [-1,  4,  5, 5, 4, -1, -1, -1,  0,  0, -1,  1,  0, 0, -1, 1, 0, 0, 3, 3,  3],  
                [-2, -2, -1, 1, 2,  2, -2, -2, -2, -3, -3, -3, -3, 3,  3, 3, 3, 2, 2, 3, -3],])
                
    M=move_motif(M,x[0],x[1],x[2])
    plot2D(M,col,w)          
    W = r*array([[-1, 1], [0, 0]]) #Front Wheel                
    Wr=move_motif(W,r*3,r*3,x[4])
    Wr=move_motif(Wr,x[0],x[1],x[2])
    Wl=move_motif(W,r*3,-r*3,x[4])
    Wl=move_motif(Wl,x[0],x[1],x[2])
    plot2D(Wr,col,2)
    plot2D(Wl,col,2)

def tondarray(M):
    if type(M)==float:
        return array([[M]])
    elif type(M)==int:
        return array([[M]])        
    else:
        return M    


def mvnrnd(xbar,Γ,n): 
    X=randn(2,n)
    X = (xbar @ ones((1,n))) + sqrtm(Γ) @ X
    return(X)    



def mvnrnd2(x,G): 
    n=len(x)
    x1=x.reshape(n)
    y = np.random.multivariate_normal(x1,G).reshape(n,1)
    return(y)    

def mvnrnd1(G):
    G=tondarray(G)
    n=len(G)
    x=array([[0]] * n)
    return(mvnrnd2(x,G))  
    

def kalman_predict(xup,Gup,u,Γα,A):
    Γ1 = A @ Gup @ A.T + Γα
    x1 = A @ xup + u    
    return(x1,Γ1)    

def kalman_correc(x0,Γ0,y,Γβ,C):
    S = C @ Γ0 @ C.T + Γβ        
    K = Γ0 @ C.T @ inv(S)           
    ytilde = y - C @ x0        
    Gup = (eye(len(x0))-K @ C) @ Γ0 
    xup = x0 + K@ytilde
    return(xup,Gup) 
    
def kalman(x0,Γ0,u,y,Γα,Γβ,A,C):
    xup,Gup = kalman_correc(x0,Γ0,y,Γβ,C)
    x1,Γ1=kalman_predict(xup,Gup,u,Γα,A)
    return(x1,Γ1)     


def place(A,B,poles):
    return place_poles(A,B,poles).gain_matrix
  
def demo_draw():  
    ax=init_figure(-15,15,-15,15)
    
    c=array([[5],[0]])
    e = Ellipse(xy=c, width=13.0, height=2.0, angle=45)  
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.9)
    e.set_facecolor(array([0.7,0.3,0.6]))   
    
    rect = Rectangle( (1,1), width=5, height=3)
    rect.set_facecolor(array([0.4,0.3,0.6]))   
    ax.add_patch(rect)    
        
    pause(0.2)    
    draw_tank(array([[-7],[5],[1]]))
    draw_tank(array([[-7],[5],[1]]),'red',0.2)

    
    draw_car(array([[1],[2],[3],[4],[0.5]]))   
    
    c = array([[-2],[-3]])
    G = array([[2,-1],[-1,4]])
    draw_ellipse(c,G,0.9,ax,[0.8,0.8,1])
    P=array([[5,-3],[9,-10],[7,-4],[7,-6]])
    draw_polygon(P,ax,'green')   
    draw_disk(array([[-8],[-8]]),2,ax,"blue")   
    draw_arc(array([[0],[5]]),array([[4],[6]]),2,'red')   
    show()  # only at the end. Otherwize, it closes the figure in a terminal mode

def loadcsv(file1):
    fichier = open(file1,'r')
    D = fichier.read().split("\n")
    fichier.close()
    for i in range(len(D)):
        D[i] = D[i].split(";")
    D = array([[float(elt) for elt in Ligne] for Ligne in D])
    return D


def init_figure(xmin,xmax,ymin,ymax): 
    fig = figure(0)
    ax = fig.add_subplot(111, aspect='equal')	
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax

def clear(ax):
    pause(0.001)
    cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)


    

def demo_animation():    
    ax=init_figure(-15,15,-15,15)
    for t in arange(0,5,0.1) :
        clear(ax)
        draw_car(array([[t],[2],[3+t],[4],[5+t]]))    
        c = array([[-2+2*t],[-3]])
        G = array([[2+t,-1],[-1,4+t]])
        draw_ellipse(c,G,0.9,ax,[0.8,0.8,1])
#        if (t>50)&(k%2000==0):
#            fig.savefig('convoy'+str(k)+'.pdf', dpi=fig.dpi)
    show()


def demo_random():  
    N=1000
    xbar = array([[1],[2]])
    Γx = array([[3,1],[1,3]])
    X=randn(2,N)
    Y=rand(2,3)
    print("Y=",Y)
    X = (xbar @ ones((1,N))) + sqrtm(Γx) @ X
    xbar_ = mean(X,axis=1)
    Xtilde = X - xbar @ ones((1,N))
    Γx_ = (Xtilde @ Xtilde.T)/N
    ax=init_figure(-20,20,-20,20)
    draw_ellipse(xbar,Γx,0.9,ax,[1,0.8,0.8])
    pause(0.5)    
    ax.scatter(X[0],X[1])    
    pause(0.3)
    plot()  



    
def sawtooth(x):
    return (x+pi)%(2*pi)-pi   # or equivalently   2*arctan(tan(x/2))



if __name__ == "__main__":
    
    np.set_printoptions(threshold=np.nan)  # print vectors in the console without "..."
    R=zeros((3,4))
    x=[[1],[2],[3]]
    R1=translate_motif(R,1,2,3)
    print('R1=',R1)
          
    demo_draw() 
    demo_animation()    
    demo_random()


    M=array([[1,2],[5,6],[9,10]])
    print(M)
    x=array([[1], [2]])    
    x2= M@x  #multiplication dans Python 3
    
    A=motif_circle3D(4)
    print (A)
#
    G = array([[1, 0], [0, 1]])
    x3=mvnrnd2(x,G)
    print("x3=",x3)
#    
#    x4=mvnrnd1(G)
#    print(x4)
#    
#    
#    print(K)
#    
#    
